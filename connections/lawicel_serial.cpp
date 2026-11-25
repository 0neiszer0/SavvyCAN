#include <QObject>
#include <QDebug>
#include <QCanBusFrame>
#include <QSerialPortInfo>
#include <QSettings>
#include <QStringBuilder>

#include "lawicel_serial.h"
#include "utility.h"

LAWICELSerial::LAWICELSerial(QString portName, int serialSpeed, int lawicelSpeed, bool canFd, int dataRate, bool listenOnly, bool active) :
    CANConnection(portName, "LAWICEL", CANCon::LAWICEL, serialSpeed, lawicelSpeed, canFd, dataRate, 3, 4000, true, listenOnly, active),
    mTimer(this) /*NB: set this as parent of timer to manage it from working thread */
{
    sendDebug("LAWICELSerial()");

    serial = nullptr;
    //isAutoRestart = false;
    rebuildLocalTimeBasis();

    readSettings();
}

QString LAWICELSerial::getLabelKey()
{
    //key the label on the USB serial number so it follows the hardware across
    //USB sockets / COM-port renumbering. Fall back to the port if none is reported.
    const QString sn = QSerialPortInfo(getPort()).serialNumber();
    return sn.isEmpty() ? getPort() : sn;
}


LAWICELSerial::~LAWICELSerial()
{
    stop();
    sendDebug("~LAWICELSerial()");
}

void LAWICELSerial::sendDebug(const QString debugText)
{
    qDebug() << debugText;
    debugOutput(debugText);
}

void LAWICELSerial::sendToSerial(const QByteArray &bytes)
{
    if (serial == nullptr)
    {
        sendDebug("Attempt to write to serial port when it has not been initialized!");
        return;
    }

    if (serial && !serial->isOpen())
    {
        sendDebug("Attempt to write to serial port when it is not open!");
        return;
    }

    QString buildDebug;
    buildDebug = "Write to serial -> ";
    foreach (int byt, bytes) {
        byt = (unsigned char)byt;
        buildDebug = buildDebug % QString::number(byt, 16) % " ";
    }
    sendDebug(buildDebug);

    if (serial) serial->write(bytes);
}

void LAWICELSerial::piStarted()
{    
    connectDevice();
}


void LAWICELSerial::piSuspend(bool pSuspend)
{
    /* update capSuspended */
    setCapSuspended(pSuspend);

    /* flush queue if we are suspended */
    if(isCapSuspended())
        getQueue().flush();
}


void LAWICELSerial::piStop()
{
    // close the channel
    sendToSerial("C\x0D");

    mTimer.stop();
    disconnectDevice();
}


bool LAWICELSerial::piGetBusSettings(int pBusIdx, CANBus& pBus)
{
    return getBusConfig(pBusIdx, pBus);
}


void LAWICELSerial::piSetBusSettings(int pBusIdx, CANBus bus)
{
    /* sanity checks */
    if( (pBusIdx < 0) || pBusIdx >= getNumBuses())
        return;

    /* copy bus config */
    setBusConfig(pBusIdx, bus);

    /* Apply updated bus settings (speed, CAN FD data rate, listen-only / active) immediately to the adapter */
    if (pBusIdx == 0 && serial != nullptr && serial->isOpen())
    {
        applyBusConfigToHardware();
    }
}


bool LAWICELSerial::piSendFrame(const CommFrame& frame)
{
    qDebug() << "Sending out lawicel frame on bus " << frame.getBus();

    if (serial == nullptr) return false;
    if (serial && !serial->isOpen()) return false;
    //if (!isConnected) return false;

    CANBus bus;
    if (getBusConfig(0, bus) && (!bus.isActive() || bus.isListenOnly())) {
        sendDebug("LAWICEL: TX blocked (bus inactive or listen-only mode)");
        return false;
    }

    // Doesn't make sense to send an error frame
    // to an adapter
    if (frame.frameId() & 0x20000000) {
        return true;
    }

    //The command letter (t/T, d/D, b/B) says whether the ID is 11 or 29 bits, so only the ID itself is
    //printed (setting bit 31 here, as before, printed extended IDs as e.g. 98DA0040).
    quint32 ID = frame.frameId() & (frame.hasExtendedFrameFormat() ? 0x1FFFFFFF : 0x7FF);

    int idx = 0;
    QString buildStr;
    //The DLC is one hex character, 0-F. FD lengths over 8 use codes 9-F, which %u printed as "10".."15".
    uint8_t dlcByte = LAWICELSerial::bytes_to_dlc_code(frame.payload().length());
    if(frame.hasFlexibleDataRateFormat()){
        if (frame.hasExtendedFrameFormat())
        {
            if (frame.hasBitrateSwitch())
                buildStr = QString::asprintf("B%08X%X", ID, dlcByte);
            else
                buildStr = QString::asprintf("D%08X%X", ID, dlcByte);
        }
        else
        {
            if (frame.hasBitrateSwitch())
                buildStr = QString::asprintf("b%03X%X", ID, dlcByte);
            else
                buildStr = QString::asprintf("d%03X%X", ID, dlcByte);
        }
    }
    else {
        if (frame.hasExtendedFrameFormat())
            buildStr = QString::asprintf("T%08X%X", ID, dlcByte);
        else
            buildStr = QString::asprintf("t%03X%X", ID, dlcByte);
    }

    QByteArray buffer;
    int sizeneed = buildStr.length() + (frame.payload().length()*2) +1;
    buffer.resize(sizeneed);
    foreach (QChar chr, buildStr)
    {
        buffer[idx] = chr.toLatin1();
        idx++;
    }

    for (qsizetype c = 0; c < frame.payload().length(); c++)
    {
        QString byt = Utility::formatByteAsHex(frame.payload()[c]);
        buffer[idx + (c * 2)] = byt[0].toLatin1();
        buffer[idx + (c * 2) + 1] = byt[1].toLatin1();
    }
    buffer[idx + (frame.payload().length() * 2)] = '\x0D'; //CR

    sendToSerial(buffer);

    return true;
}



/****************************************************************/

void LAWICELSerial::rebuildLocalTimeBasis()
{
    timeBasis = 0;
    lastHWTimestamp = -1;
    wrapAdder = 0;
}

void LAWICELSerial::readSettings()
{
    //QSettings settings;
}


void LAWICELSerial::connectDevice()
{
    QSettings settings;

    /* disconnect device */
    if(serial)
        disconnectDevice();

    /* open new device */

    qDebug() << "Serial port: " << getPort();

    serial = new QSerialPort(QSerialPortInfo(getPort()));
    if(!serial) {
        sendDebug("can't open serial port " + getPort());
        return;
    }
    sendDebug("Created Serial Port Object");

    /* connect reading event */
    connect(serial, &QSerialPort::readyRead, this, &LAWICELSerial::readSerialData);
    connect(serial, &QSerialPort::errorOccurred, this, &LAWICELSerial::serialError);

    /* configure */
    serial->setBaudRate(mSerialSpeed);
    serial->setDataBits(serial->Data8);

    serial->setFlowControl(serial->HardwareControl);
    //serial->setFlowControl(serial->NoFlowControl);
    if (!serial->open(QIODevice::ReadWrite))
    {
        //sendDebug("Error returned during port opening: " + serial->errorString());
    }
    else
    {
        //serial->setDataTerminalReady(true); //Seemingly these two lines used to be needed
        //serial->setRequestToSend(true);     //But, really both ends should automatically handle these
        deviceConnected();
    }
}

void LAWICELSerial::applyBusConfigToHardware()
{
    QByteArray output;

    output.append('C'); //close the bus before changing settings (or keep closed if inactive)
    output.append('\x0D');
    sendToSerial(output);
    output.clear();

    const CANBus &bus = this->mBusData[0].mBus;
    if (!bus.isActive())
    {
        sendDebug("LAWICEL: Bus disabled (closed)");
        return;
    }

    output.append('S'); //configure nominal/arbitration speed of bus
    switch (bus.getSpeed())
    {
    case 10000:
        output.append('0');
        break;
    case 20000:
        output.append('1');
        break;
    case 50000:
        output.append('2');
        break;
    case 100000:
        output.append('3');
        break;
    case 125000:
        output.append('4');
        break;
    case 250000:
        output.append('5');
        break;
    case 500000:
        output.append('6');
        break;
    case 800000:
        output.append('7');
        break;
    case 1000000:
        output.append('8');
        break;
    default:
        output.append('6');
        break;
    }
    output.append('\x0D');
    sendToSerial(output);
    output.clear();

    //CAN FD mode and data rate come from the bus settings (filled in from the connection dialog).
    const bool canFd = bus.isCanFD();
    const int dataRate = bus.getDataRate();
    if (canFd)
    {
        switch (dataRate)
        {
        case 500000:
            output.append("Y0");
            break;
        case 1000000:
            output.append("Y1");
            break;
        case 2000000:
            output.append("Y2");
            break;
        case 4000000:
            output.append("Y4");
            break;
        case 5000000:
            output.append("Y5");
            break;
        case 8000000:
            output.append("Y8");
            break;
        default:
            qWarning() << "LAWICEL: CAN FD data rate" << dataRate << "not supported, using 2 Mbit/s";
            output.append("Y2");
            break;
        }
        output.append('\x0D');
        sendToSerial(output);
        output.clear();
    }

    if (bus.isListenOnly())
    {
        output.append('L'); //open bus in listen-only (silent) mode
    }
    else
    {
        output.append('O'); //open bus in normal mode
    }
    output.append('\x0D');
    sendToSerial(output);
}

void LAWICELSerial::deviceConnected()
{
    sendDebug("Connecting to LAWICEL Device!");

    rebuildLocalTimeBasis();
    applyBusConfigToHardware();

    mNumBuses = 1;
    setStatus(CANCon::CONNECTED);
    CANConStatus stats;
    stats.conStatus = getStatus();
    stats.numHardwareBuses = mNumBuses;
    mBusData[0].mConfigured = true;
    emit status(stats);
}

void LAWICELSerial::disconnectDevice() {
    if (serial != nullptr)
    {
        if (serial->isOpen())
        {
            //serial->clear();
            serial->close();

        }
        serial->disconnect(); //disconnect all signals
        delete serial;
        serial = nullptr;
    }

    setStatus(CANCon::NOT_CONNECTED);
    CANConStatus stats;
    stats.conStatus = getStatus();
    stats.numHardwareBuses = mNumBuses;
    emit status(stats);
}

void LAWICELSerial::serialError(QSerialPort::SerialPortError err)
{
    QString errMessage;
    bool killConnection = false;
    switch (err)
    {
    case QSerialPort::NoError:
        return;
    case QSerialPort::DeviceNotFoundError:
        errMessage = "Device not found error on serial";
        killConnection = true;
        piStop();
        break;
    case QSerialPort::PermissionError:
        errMessage =  "Permission error on serial port";
        killConnection = true;
        piStop();
        break;
    case QSerialPort::OpenError:
        errMessage =  "Open error on serial port";
        killConnection = true;
        piStop();
        break;
    case QSerialPort::WriteError:
        errMessage = "Write error on serial port";
        piStop();
        break;
    case QSerialPort::ReadError:
        errMessage = "Read error on serial port";
        piStop();
        break;
    case QSerialPort::ResourceError:
        errMessage = "Serial port seems to have disappeared.";
        killConnection = true;
        piStop();
        break;
    case QSerialPort::UnsupportedOperationError:
        errMessage = "Unsupported operation on serial port";
        killConnection = true;
        break;
    case QSerialPort::UnknownError:
        errMessage = "Beats me what happened to the serial port.";
        killConnection = true;
        piStop();
        break;
    case QSerialPort::TimeoutError:
        errMessage = "Timeout error on serial port";
        killConnection = true;
        break;
    case QSerialPort::NotOpenError:
        errMessage = "The serial port isn't open";
        killConnection = true;
        piStop();
        break;
    }
    /*
    if (serial)
    {
        serial->clearError();
        serial->flush();
        serial->close();
    }*/
    if (errMessage.length() > 1)
    {
        sendDebug(errMessage);
    }
    if (killConnection)
    {
        qDebug() << "Shooting the serial object in the head. It deserves it.";
        disconnectDevice();
    }
}


void LAWICELSerial::connectionTimeout()
{
    //one second after trying to connect are we actually connected?
    if (CANCon::NOT_CONNECTED==getStatus()) //no?
    {
        //then emit the the failure signal and see if anyone cares
        sendDebug("Failed to connect to LAWICEL at that com port");

        disconnectDevice();
        connectDevice();
    }
    else
    {
        /* start timer */
        connect(&mTimer, &QTimer::timeout, this, &LAWICELSerial::handleTick);
        mTimer.setInterval(250); //tick four times per second
        mTimer.setSingleShot(false); //keep ticking
        mTimer.start();
    }
}

void LAWICELSerial::readSerialData()
{
    QByteArray data;
    QString debugBuild;

    if (serial) data = serial->readAll();

    //sendDebug("Got data from serial. Len = " % QString::number(data.length()));
    for (int i = 0; i < data.length(); i++)
    {
        unsigned char c = data.at(i);
        //qDebug() << c << "    " << QString::number(c, 16) << "     " << QString(c);
        debugBuild = debugBuild % QString::number(c, 16).rightJustified(2,'0') % " ";
        if (c == '\x07') // LAWICEL/SLCAN BEL (error) has no trailing CR; clear buffer so the next frame isn't corrupted
        {
            sendDebug("LAWICEL: Received BEL (0x07) error from adapter");
            mBuildLine.clear();
            continue;
        }
        if (c == '\x0A') // ignore stray LF
        {
            continue;
        }
        //procRXChar(c);
        mBuildLine.append(QChar(c));
        if (c == '\x0D') //all lawicel commands end in CR
        {
            //qDebug() << "Got CR!";

            CommFrame buildFrame;
            QByteArray buildData;

            buildFrame.setTimeStamp(CommFrame::TimeStamp(0, QDateTime::currentMSecsSinceEpoch() * 1000ll));
            if (!useSystemTime)
            {
                bool isFD = false;
                char offset;
                char len_index;
                switch(mBuildLine[0].toLatin1()) {
                    case 'b':
                    case 'd':
                        isFD = true;
                        [[fallthrough]];
                    case 't':
                        offset = 5;
                        len_index = 4;
                        break;
                    case 'B':
                    case 'D':
                        isFD = true;
                        [[fallthrough]];
                    case 'T':
                        offset = 10;
                        len_index = 9;
                        break;
                    default:
                        offset = 5;
                        len_index = 4;
                        break;
                }

                //the DLC is one hex digit: CAN FD codes A-F (16-64 bytes) read as decimal gave 0, so the
                //hardware timestamp was taken from the middle of the data for frames of 16 bytes or more
                int dlc = mBuildLine.mid(len_index, 1).toInt(nullptr, 16);
                int byteCount = isFD ? (int)dlc_code_to_bytes(dlc) : dlc;
                int frameStringLen = offset + byteCount * 2;
                if (mBuildLine.length() > frameStringLen + 1)
                {
                    qint64 hardwareMs = mBuildLine.mid(frameStringLen, 4).toInt(nullptr, 16);
                    if (lastHWTimestamp >= 0 && hardwareMs < lastHWTimestamp) { wrapAdder += 60000; }
                    lastHWTimestamp = hardwareMs;
                    qint64 unwrappedMs = wrapAdder + hardwareMs;
                    if (timeBasis == 0) { timeBasis = QDateTime::currentMSecsSinceEpoch() - unwrappedMs; }
                    buildFrame.setTimeStamp(CommFrame::TimeStamp(0, (timeBasis + unwrappedMs) * 1000ll));
                }
            };

            switch (mBuildLine[0].toLatin1())
            {
            case 't': //standard frame
                //tIIILDD
                buildFrame.setFrameId(mBuildLine.mid(1, 3).toInt(nullptr, 16));
                buildFrame.setReceived(true);
                buildFrame.setFrameType(CommFrame::FrameType::CANDataFrame);
                buildData.resize(mBuildLine.mid(4, 1).toInt());
                for (int c = 0; c < buildData.size(); c++)
                {
                    buildData[c] = mBuildLine.mid(5 + (c*2), 2).toInt(nullptr, 16);
                }
                buildFrame.setPayload(buildData);
                //addTimestamp(1 + 3 + 1 + buildData.size() * 2);
                if (!isCapSuspended())
                {
                    /* get frame from queue */
                    CommFrame* frame_p = getQueue().get();
                    if(frame_p) {
                        //qDebug() << "Lawicel got frame on bus " << frame_p->bus;
                        /* copy frame */
                        *frame_p = buildFrame;
                        checkTargettedFrame(buildFrame);
                        /* enqueue frame */
                        getQueue().queue();
                    }
                    else
                        qDebug() << "can't get a frame, ERROR";
                }
                break;
            case 'T': //extended frame
                //TIIIIIIIILDD.
                buildFrame.setFrameId(mBuildLine.mid(1, 8).toInt(nullptr, 16));
                buildFrame.setReceived(true);
                buildFrame.setFrameType(CommFrame::FrameType::CANDataFrame);
                buildFrame.setExtendedFrameFormat(true);
                buildData.resize(mBuildLine.mid(9, 1).toInt());
                for (int c = 0; c < buildData.size(); c++)
                {
                    buildData[c] = mBuildLine.mid(10 + (c*2), 2).toInt(nullptr, 16);
                }
                buildFrame.setPayload(buildData);
                //addTimestamp(1 + 8 + 1 + buildData.size() * 2);
                if (!isCapSuspended())
                {
                    /* get frame from queue */
                    CommFrame* frame_p = getQueue().get();
                    if(frame_p) {
                        //qDebug() << "Lawicel got frame on bus " << frame_p->bus;
                        /* copy frame */
                        *frame_p = buildFrame;
                        checkTargettedFrame(buildFrame);
                        /* enqueue frame */
                        getQueue().queue();
                    }
                    else
                        qDebug() << "can't get a frame, ERROR";
                }
                break;
            case 'b':
                buildFrame.setBitrateSwitch(true); //BRS enabled
                [[fallthrough]];
            case 'd': //standard fd frame, BRS disabled
                //tIIILDD
                buildFrame.setFlexibleDataRateFormat(true);
                buildFrame.setFrameId(mBuildLine.mid(1, 3).toInt(nullptr, 16));
                buildFrame.setReceived(true);
                buildFrame.setFrameType(CommFrame::FrameType::CANFDDataFrame); //keeps the FD flag (and BRS) set above
                buildData.resize(LAWICELSerial::dlc_code_to_bytes(mBuildLine.mid(4, 1).toInt(nullptr, 16)));
                for (int c = 0; c < buildData.size(); c++)
                {
                    buildData[c] = mBuildLine.mid(5 + (c*2), 2).toInt(nullptr, 16);
                }
                buildFrame.setPayload(buildData);
                if (!isCapSuspended())
                {
                    /* get frame from queue */
                    CommFrame* frame_p = getQueue().get();
                    if(frame_p) {
                        //qDebug() << "Lawicel got frame on bus " << frame_p->bus;
                        /* copy frame */
                        *frame_p = buildFrame;
                        checkTargettedFrame(buildFrame);
                        /* enqueue frame */
                        getQueue().queue();
                    }
                    else
                        qDebug() << "can't get a frame, ERROR";
                }
                break;
            case 'B':
                buildFrame.setBitrateSwitch(true); //BRS enabled
                [[fallthrough]];
            case 'D': //extended fd frame, BRS disabled ('B' falls through with BRS already set)
                //TIIIIIIIILDD.
                buildFrame.setFlexibleDataRateFormat(true);
                buildFrame.setFrameId(mBuildLine.mid(1, 8).toInt(nullptr, 16));
                buildFrame.setReceived(true);
                buildFrame.setFrameType(CommFrame::FrameType::CANFDDataFrame); //keeps the FD flag (and BRS) set above
                buildFrame.setExtendedFrameFormat(true);
                buildData.resize(LAWICELSerial::dlc_code_to_bytes(mBuildLine.mid(9, 1).toInt(nullptr, 16)));
                for (int c = 0; c < buildData.size(); c++)
                {
                    buildData[c] = mBuildLine.mid(10 + (c*2), 2).toInt(nullptr, 16);
                }
                buildFrame.setPayload(buildData);
                if (!isCapSuspended())
                {
                    /* get frame from queue */
                    CommFrame* frame_p = getQueue().get();
                    if(frame_p) {
                        //qDebug() << "Lawicel got frame on bus " << frame_p->bus;
                        /* copy frame */
                        *frame_p = buildFrame;
                        checkTargettedFrame(buildFrame);
                        /* enqueue frame */
                        getQueue().queue();
                    }
                    else
                        qDebug() << "can't get a frame, ERROR";
                }
                break;
            }
            mBuildLine.clear();
        }
    }
    debugOutput(debugBuild);
    //qDebug() << debugBuild;
}

//Debugging data sent from connection window. Inject it into Comm traffic.
void LAWICELSerial::debugInput(QByteArray bytes) {
   sendToSerial(bytes);
}

void LAWICELSerial::handleTick()
{
    //qDebug() << "Tick!";
}


// Convert a FDCAN_data_length_code to number of bytes in a message
uint8_t LAWICELSerial::dlc_code_to_bytes(int dlc_code)
{
    if (dlc_code<=8)
        return dlc_code;
    else{
        switch(dlc_code)
        {
            case 9:
                return 12;
            case 10:
                return 16;
            case 11:
                return 20;
            case 12:
                return 24;
            case 13:
                return 32;
            case 14:
                return 48;
            case 15:
                return 64;
            default:
                return 0;
        }
    }
}

uint8_t LAWICELSerial::bytes_to_dlc_code(uint8_t bytes)
{
    if (bytes<=8)
        return bytes;
    else{
        switch(bytes)
        {
            case 12:
                return 9;
            case 16:
                return 10;
            case 20:
                return 11;
            case 24:
                return 12;
            case 32:
                return 13;
            case 48:
                return 14;
            case 64:
                return 15;
            default:
                return 0;
        }
    }
}


