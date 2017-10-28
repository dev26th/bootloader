#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QSerialPortInfo>

Q_DECLARE_METATYPE(Comm::State)

static const size_t PAGE_SIZE = 1024;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    isConnected(false)
{
    ui->setupUi(this);

    qRegisterMetaType<Comm::State>();

    // serial ports
    on_refreshButton_clicked();

    // comm
    comm = new Comm();
    comm->moveToThread(&commThread);
    connect(&commThread, &QThread::started, comm, &Comm::onStart);
    connect(&commThread, &QThread::finished, comm, &Comm::deleteLater);
    connect(this, &MainWindow::portConnect, comm, &Comm::portConnect);
    connect(this, &MainWindow::portDisconnect, comm, &Comm::portDisconnect);
    connect(this, &MainWindow::download, comm, &Comm::download);
    connect(comm, &Comm::error, this, &MainWindow::on_serError);
    connect(comm, &Comm::downloadDone, this, &MainWindow::on_serDownloadDone);
    connect(comm, &Comm::stateChanged, this, &MainWindow::on_serStateChanged);
    connect(comm, &Comm::deviceInfo, this, &MainWindow::on_serDeviceInfo);
    commThread.start();

    checkDeviceMatchsFile();
}

MainWindow::~MainWindow()
{
    commThread.quit();
    commThread.wait();

    delete ui;
}

void MainWindow::updateFileInfo() {
    ui->protocolEdit->setText("");
    ui->productIdEdit->setText("");
    ui->appVersionEdit->setText("");
    ui->sizeEdit->setText("");

    QString fileName = ui->inputFileBox->currentText();

    QFile inputFile(fileName);
    if(!inputFile.open(QFile::ReadOnly)) {
        QMessageBox::warning(this, tr("Bootloader Control"),
            tr("Cannot read from file %1:\n%2.").arg(fileName).arg(inputFile.errorString()));
    }
    else {
        try {
            fileContent = inputFile.readAll();

            QDataStream stream(&fileContent, QIODevice::ReadOnly);
            stream.setByteOrder(QDataStream::ByteOrder::LittleEndian);

            uint32_t protocolVersion;
            uint32_t productId;
            uint32_t appVersion;
            uint32_t pageCount;

            stream >> protocolVersion;
            stream >> productId;
            stream >> appVersion;
            stream >> pageCount;

            ui->protocolEdit->setText("0x" + QString("%1").arg(protocolVersion, 8, 16, QChar('0')).toUpper());
            ui->productIdEdit->setText("0x" + QString("%1").arg(productId, 8, 16, QChar('0')).toUpper());
            ui->appVersionEdit->setText("0x" + QString("%1").arg(appVersion, 8, 16, QChar('0')).toUpper());
            ui->sizeEdit->setText(QString("%1").arg(pageCount*PAGE_SIZE));
        }
        catch(char const* msg) {
            QMessageBox::critical(this, tr("Bootloader Control"), tr(msg));
        }
    }
    checkDeviceMatchsFile();
}

static void addToComboBox(QComboBox& box, const QString& item) {
    int p = box.findText(item, Qt::MatchExactly);
    if(p != 0) {
        box.removeItem(p);
        box.insertItem(0, item);
        box.setCurrentText(item);
    }
}

void MainWindow::on_inputFileButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Select input file"), "",
        tr("Binary Files (*.bin);;All Files (*)"));
    if(fileName.isEmpty()) return;

    fileName = QDir::toNativeSeparators(fileName);
    ui->inputFileBox->setCurrentText(fileName);
    addToComboBox(*ui->inputFileBox, fileName);
    updateFileInfo();
}

void MainWindow::on_inputFileBox_currentIndexChanged(const QString &arg1)
{
    (void)arg1;

    updateFileInfo();
}

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, tr("Bootloader Control"), tr("A tool to download firmware via the bootloader. Developed by Anatoli Klassen. Public domain."));
}

void MainWindow::on_actionAboutQt_triggered()
{
    QMessageBox::aboutQt(this, tr("Bootloader Control"));
}

void MainWindow::on_refreshButton_clicked()
{
    QString selPort = currentPort;
    ui->portBox->clear();

    QList<QSerialPortInfo> serPorts = QSerialPortInfo::availablePorts();
    std::sort(serPorts.begin(), serPorts.end(), [] (const QSerialPortInfo& a, const QSerialPortInfo& b) {
            return a.portName() < b.portName();
        });

    int count = 0;
    for(auto i = serPorts.begin(); i != serPorts.end(); ++i, ++count) {
        QString title = QString("%1: (%2) %3 %4").arg(i->portName()).arg(i->description()).arg(i->manufacturer()).arg(i->serialNumber());
        ui->portBox->addItem(title, QVariant(i->portName()));
        if(selPort == i->portName()) {
            ui->portBox->setCurrentIndex(count);
        }
    }
}

void MainWindow::on_serError(QString msg)
{
    this->disconnectSer();
    QMessageBox::critical(this, tr("Bootloader Control"), tr(msg.toUtf8()));
}

void MainWindow::on_serDownloadDone() {
    QMessageBox::information(this, tr("Bootloader Control"), tr("Download done successfully"));
}

void MainWindow::clearDeviceInfo()
{
    ui->devProductIdEdit->setText("");
    ui->bootloaderVersionEdit->setText("");
    checkDeviceMatchsFile();
}

void MainWindow::on_serStateChanged(Comm::State state)
{
    QString s;
    switch(state) {
        case Comm::State::Idle:
            s = "Idle";
            clearDeviceInfo();
            break;

        case Comm::State::Connecting:
            s = "Connecting";
            clearDeviceInfo();
            break;

        case Comm::State::Connected:
            s = "Connected";
            clearDeviceInfo();
            break;

        case Comm::State::Starting:
        case Comm::State::Sending:
            s = "Download";
            break;
    }
    ui->statusEdit->setText(s);
}

void MainWindow::checkDeviceMatchsFile()
{
    bool ok = true;
    if(!ui->protocolEdit->text().isEmpty() && !ui->bootloaderVersionEdit->text().isEmpty()
            && ui->protocolEdit->text() != ui->bootloaderVersionEdit->text())
    {
        ui->protocolEdit->setStyleSheet("QLineEdit{border: 2px solid red}");
        ui->bootloaderVersionEdit->setStyleSheet("QLineEdit{border: 2px solid red}");
        ok = false;
    }
    else {
        ui->protocolEdit->setStyleSheet("");
        ui->bootloaderVersionEdit->setStyleSheet("");
    }

    if(!ui->productIdEdit->text().isEmpty() && !ui->devProductIdEdit->text().isEmpty()
            && ui->productIdEdit->text() != ui->devProductIdEdit->text())
    {
        ui->productIdEdit->setStyleSheet("QLineEdit{border: 2px solid red}");
        ui->devProductIdEdit->setStyleSheet("QLineEdit{border: 2px solid red}");
        ok = false;
    }
    else {
        ui->productIdEdit->setStyleSheet("");
        ui->devProductIdEdit->setStyleSheet("");
    }

    if(ok && !ui->protocolEdit->text().isEmpty() && !ui->bootloaderVersionEdit->text().isEmpty()
            && !ui->productIdEdit->text().isEmpty() && !ui->devProductIdEdit->text().isEmpty())
    {
        ui->downloadButton->setEnabled(true);
    }
    else
    {
        ui->downloadButton->setEnabled(false);
    }
}

void MainWindow::on_serDeviceInfo(qint64 bootloaderVersion, qint64 productId)
{
    ui->devProductIdEdit->setText("0x" + QString("%1").arg(productId, 8, 16, QChar('0')).toUpper());
    ui->bootloaderVersionEdit->setText("0x" + QString("%1").arg(bootloaderVersion, 8, 16, QChar('0')).toUpper());
    checkDeviceMatchsFile();
}

void MainWindow::connectSer()
{
    emit portConnect(currentPort);

    ui->connectButton->setText("Disconnect");
    isConnected = true;
}

void MainWindow::disconnectSer()
{
    emit portDisconnect();

    ui->connectButton->setText("Connect");
    isConnected = false;
}

void MainWindow::on_connectButton_clicked()
{
    if(isConnected) disconnectSer();
    else            connectSer();
}

void MainWindow::on_portBox_currentIndexChanged(int index)
{
    (void)index;

    QVariant data = ui->portBox->currentData();
    currentPort = data.toString();
    if(isConnected) connectSer();
}

void MainWindow::on_downloadButton_clicked()
{
    emit download(fileContent);
}
