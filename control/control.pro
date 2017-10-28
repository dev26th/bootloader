#-------------------------------------------------
#
# Project created by QtCreator 2017-10-24T22:01:02
#
#-------------------------------------------------

QMAKE_CXXFLAGS += -std=c++14
QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = control
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    comm.cpp

HEADERS  += mainwindow.h \
    comm.h

FORMS    += mainwindow.ui

RC_ICONS = app.ico

DISTFILES += \
    icon_16.png \
    icon_24.png \
    icon_32.png \
    icon_48.png \
    icon.svg \
    app.ico

RESOURCES += \
    resources.qrc
