#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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
    void on_keyGenerateButton_clicked();

    void on_inputFileButton_clicked();

    void on_outputFileButton_clicked();

    void on_createButton_clicked();

    void on_actionAbout_triggered();

    void on_actionAboutQt_triggered();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
