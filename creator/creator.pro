#-------------------------------------------------
#
# Project created by QtCreator 2017-10-24T21:26:20
#
#-------------------------------------------------

QMAKE_CXXFLAGS += -std=c++14
QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = creator
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    tiny-AES-c/aes.c \
    crypto.cpp \
    utils.cpp

HEADERS  += mainwindow.h \
    tiny-AES-c/aes.h \
    crypto.h \
    utils.h

FORMS    += mainwindow.ui

RC_ICONS = app.ico

DISTFILES += \
    app.ico \
    icon_16.png \
    icon_24.png \
    icon_32.png \
    icon_48.png \
    icon.svg

RESOURCES += \
    resources.qrc
