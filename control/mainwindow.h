#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "comm.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_inputFileButton_clicked();

    void on_actionAbout_triggered();

    void on_actionAboutQt_triggered();

    void on_refreshButton_clicked();

    void on_serError(QString msg);

    void on_serDownloadDone();

    void on_serStateChanged(Comm::State state);

    void on_serDeviceInfo(qint64 bootloaderVersion, qint64 productId);

    void on_connectButton_clicked();

    void on_portBox_currentIndexChanged(int index);

    void on_downloadButton_clicked();

    void on_inputFileBox_currentIndexChanged(const QString &arg1);

signals:
    void portConnect(QString portName);
    void portDisconnect();
    void download(QByteArray data);

private:
    void updateFileInfo();
    void connectSer();
    void disconnectSer();
    void clearDeviceInfo();
    void checkDeviceMatchsFile();

private:
    Ui::MainWindow *ui;

    Comm *comm;
    QThread commThread;
    QString currentPort;
    bool isConnected;
    QByteArray fileContent;
};

#endif // MAINWINDOW_H
