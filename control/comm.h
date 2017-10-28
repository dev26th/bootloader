#ifndef COMM_H
#define COMM_H

#include <QtSerialPort/QtSerialPort>

class Comm : public QObject {
    Q_OBJECT

public:
    enum class State {
        Idle,
        Connecting,
        Connected,
        Starting,
        Sending,
    };

public:
    Comm();

    static const int INTERVAL_MS = 500;
    static const int ALIVE_TIMEOUT_MS = 1000;

public slots:
    void onStart();
    void portConnect(QString portName);
    void portDisconnect();
    void download(QByteArray data);

signals:
    void error(QString msg);
    void downloadDone();
    void stateChanged(Comm::State state);
    void deviceInfo(qint64 bootloaderVersion, qint64 productId);

protected:
    void timerEvent(QTimerEvent *event) override;

private slots:
    void on_readyRead();

private:
    enum class Command : uint8_t {
        None       = 0x00,
        GetVersion = 0x01,
        Start      = 0x02,
        NextBlock  = 0x03,
        Reset      = 0x04,

        OK         = 0x40,
        Error      = 0x80,
    };

private:
    void setState(State state);
    void startTheTimer();
    void stopTheTimer();
    bool writeCmd(Command cmd);
    void processDeviceInfo(QByteArray& d);
    void processRead();
    void sendNextPage();
    void onDownloadError();
    Command commandResult(Command actual, Command exp);

private:
    QSerialPort *ser;
    int timerId;
    State state;
    QByteArray data;
    size_t dataTail;
    QElapsedTimer aliveTimer;
};

#endif // COMM_H

