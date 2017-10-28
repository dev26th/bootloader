#include "comm.h"

#include <QDebug>

Comm::Comm()
  : timerId(0), state(State::Idle), dataTail(0)
{
}

void Comm::onStart()
{
    setState(State::Idle);

    ser = new QSerialPort(this);
    connect(ser, &QSerialPort::readyRead, this, &Comm::on_readyRead);
}

void Comm::startTheTimer()
{
    if(timerId) killTimer(timerId);
    timerId = startTimer(INTERVAL_MS, Qt::PreciseTimer);
}

void Comm::stopTheTimer()
{
    if(timerId) {
        killTimer(timerId);
        timerId = 0;
    }
}

void Comm::portConnect(QString portName)
{
    if(ser->isOpen()) portDisconnect();

    ser->setPortName(portName);
    ser->setBaudRate(115200);

    bool openSuccess = ser->open(QIODevice::ReadWrite);
    if(!openSuccess) {
        emit error("Cannot connect to the port");
    }
    else {
        startTheTimer();
        setState(State::Connecting);
    }
}

void Comm::setState(State state)
{
    this->state = state;
    emit stateChanged(state);
}

void Comm::portDisconnect()
{
    ser->close();
    stopTheTimer();
    setState(State::Idle);
}

Comm::Command Comm::commandResult(Command actual, Command exp)
{
    uint8_t a = (uint8_t)actual;
    uint8_t b = (uint8_t)exp;
    uint8_t r = a ^ b;
    if(r == (uint8_t)Command::OK)    return Command::OK;
    if(r == (uint8_t)Command::Error) return Command::Error;
    return Command::None;
}

void Comm::processDeviceInfo(QByteArray& d)
{
    QDataStream stream(&d, QIODevice::ReadOnly);
    stream.setByteOrder(QDataStream::ByteOrder::LittleEndian);

    uint32_t bootloaderVersion;
    uint32_t productId;

    stream >> bootloaderVersion;
    stream >> productId;

    setState(State::Connected);
    emit deviceInfo(bootloaderVersion, productId);
}

void Comm::timerEvent(QTimerEvent *event)
{
    if(!ser->isOpen()) return;

    (void)event;
    switch(state) {
        case State::Idle:
            break;

        case State::Connecting:
        case State::Connected:
            writeCmd(Command::GetVersion);
            // fallthrough

        case State::Starting:
        case State::Sending:
            if(aliveTimer.elapsed() >= ALIVE_TIMEOUT_MS)
                setState(State::Connecting);
            break;
    }
}

bool Comm::writeCmd(Command cmd)
{
    if(!ser->isOpen()) return false;

    ser->clear();

    char cmdBuf[] = { (char)cmd };
    qint64 n = ser->write(cmdBuf, sizeof(cmdBuf));

    return (n == 1);
}

void Comm::on_readyRead()
{
    while(!ser->atEnd()) {
        processRead();
    }
}

void Comm::onDownloadError()
{
    emit error("Unexpected error from the device. Download aborted.");
    setState(State::Connecting);
}

void Comm::processRead()
{
    char c;
    if(!ser->getChar(&c)) return;

    switch(state) {
        case State::Idle:
            break;

        case State::Connecting:
        case State::Connected:
            if(dataTail) {
                data.append(c);
                if(--dataTail == 0) {
                    setState(State::Connected);
                    processDeviceInfo(data);
                }
            }
            else {
                if(commandResult((Command)c, Command::GetVersion) == Command::OK) {
                    dataTail = 8;
                    data.clear();
                    aliveTimer.start();
                }
            }
            break;

        case State::Starting:
            if(commandResult((Command)c, Command::Start) == Command::OK) {
                sendNextPage();
                aliveTimer.start();
            }
            else if(commandResult((Command)c, Command::Start) == Command::Error) {
                onDownloadError();
            }
            break;

        case State::Sending:
            if(commandResult((Command)c, Command::NextBlock) == Command::OK) {
                sendNextPage();
                aliveTimer.start();
            }
            else if(commandResult((Command)c, Command::NextBlock) == Command::Error) {
                onDownloadError();
            }
            break;
    }
}

void Comm::sendNextPage()
{
    int pageLen = 1024;
    if(data.size() >= pageLen) {
        setState(State::Sending);

        QByteArray page = data.left(pageLen);
        data.remove(0, pageLen);

        writeCmd(Command::NextBlock);
        ser->write(page);
    }
    else {
        emit downloadDone();
        setState(State::Connected);
    }
}

void Comm::download(QByteArray data)
{
    setState(State::Starting);
    this->data = data;

    size_t headerLen = 4 + 4 + 4 + 4 + 16 + 4;
    QByteArray header = this->data.left(headerLen);
    this->data.remove(0, headerLen);

    writeCmd(Command::Start);
    ser->write(header);
}
