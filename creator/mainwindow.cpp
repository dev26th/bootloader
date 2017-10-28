#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QMessageBox>

#include "crypto.h"
#include "utils.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_keyGenerateButton_clicked()
{
    ui->keyEdit->setText(Utils::toHex(Crypto::random(Crypto::KEY_LEN)));
}

void MainWindow::on_inputFileButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Select input file"), "",
        tr("Binary Files (*.bin);;All Files (*)"));
    if(fileName.isEmpty()) return;

    ui->inputFileBox->setCurrentText(QDir::toNativeSeparators(fileName));
}

void MainWindow::on_outputFileButton_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this,
        tr("Select output file"), "",
        tr("Binary Files (*.bin);;All Files (*)"));
    if(fileName.isEmpty()) return;

    ui->outputFileBox->setCurrentText(QDir::toNativeSeparators(fileName));
}

static void addToComboBox(QComboBox& box, const QString& item) {
    int p = box.findText(item, Qt::MatchExactly);
    if(p != 0) {
        box.removeItem(p);
        box.insertItem(0, item);
        box.setCurrentText(item);
    }
}

static const size_t PAGE_SIZE = 1024;

static void generate(uint32_t productId, uint32_t appVersion, uint32_t protocolVersion, const QByteArray& key, QFile& inputFile, QFile& outputFile) {
    QByteArray iv = Crypto::random(Crypto::IV_LEN);

    QByteArray input = inputFile.readAll();
    if(input.size() % PAGE_SIZE != 0) {
        int pad = PAGE_SIZE - (input.size() % PAGE_SIZE);
        for(int i = 0; i < pad; ++i) input.append('\0');
    }

    QByteArray enc = Crypto::encrypt(input, key, iv);

    QDataStream stream(&outputFile);
    stream.setByteOrder(QDataStream::LittleEndian);

    stream << protocolVersion;
    stream << productId;
    stream << appVersion;
    stream << (uint32_t)((input.size() / PAGE_SIZE));
    stream.writeRawData(iv.data(), iv.size());
    stream << Utils::crc32(input);
    stream.writeRawData(enc.data(), enc.size());
}

void MainWindow::on_createButton_clicked()
{
    QByteArray key;
    try {
        key = Utils::fromHex(ui->keyEdit->text());
        if(key.size() != Crypto::KEY_LEN) throw "Wrong length";
    }
    catch(char const* msg) {
        QMessageBox::warning(this, tr("Bootloader Creator"), tr("The key must be 16 bytes in hex format"));
        return;
    }

    bool ok;

    uint32_t productId = 0;
    if(!ui->productIdEdit->text().isEmpty()) {
        productId = ui->productIdEdit->text().toUInt(&ok, 16);
        if(!ok) {
            QMessageBox::warning(this, tr("Bootloader Creator"), tr("Product ID must be an uint32 in hex format"));
            return;
        }
    }

    uint32_t appVersion = 0;
    if(!ui->appVersionEdit->text().isEmpty()) {
        appVersion = ui->appVersionEdit->text().toUInt(&ok, 16);
        if(!ok) {
            QMessageBox::warning(this, tr("Bootloader Creator"), tr("App version must be an uint32 in hex format"));
            return;
        }
    }

    QString inputName = ui->inputFileBox->currentText();
    if(inputName.isEmpty()) {
        QMessageBox::warning(this, tr("Bootloader Creator"), tr("Please enter input file name"));
        return;
    }
    QString outputName = ui->outputFileBox->currentText();
    if(outputName.isEmpty()) {
        QMessageBox::warning(this, tr("Bootloader Creator"), tr("Please enter output file name"));
        return;
    }

    QFile inputFile(inputName);
    if(!inputFile.open(QFile::ReadOnly)) {
        QMessageBox::warning(this, tr("Bootloader Creator"),
            tr("Cannot read from file %1:\n%2.").arg(inputName).arg(inputFile.errorString()));
        return;
    }

    QFile outputFile(outputName);
    if(!outputFile.open(QFile::WriteOnly | QFile::Truncate)) {
        QMessageBox::warning(this, tr("Bootloader Creator"),
            tr("Cannot write to file %1:\n%2.").arg(outputName).arg(outputFile.errorString()));
        return;
    }

    addToComboBox(*ui->inputFileBox, inputName);
    addToComboBox(*ui->outputFileBox, outputName);

    try {
        generate(productId, appVersion, 1, key, inputFile, outputFile);

        inputFile.close();
        outputFile.close();

        QMessageBox::information(this, tr("Bootloader Creator"), "File created successfully");
    }
    catch(char const* msg) {
        QMessageBox::critical(this, tr("Bootloader Creator"), tr(msg));
        return;
    }
}

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, tr("Bootloader Creator"), tr("An creator of input files for the 'Bootloader Control'. Developed by Anatoli Klassen. Public domain."));
}

void MainWindow::on_actionAboutQt_triggered()
{
    QMessageBox::aboutQt(this, tr("Bootloader Creator"));
}
