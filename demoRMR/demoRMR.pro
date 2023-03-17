#-------------------------------------------------
#
# Project created by QtCreator 2018-02-11T14:35:38
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = demoRMR
TEMPLATE = app
win32 {
LIBS += -lws2_32
LIBS += -lWinmm
}
INCLUDEPATH += ../robot
LIBS += -L../bin -lrobot


SOURCES += main.cpp\
        mainwindow.cpp \
        p_controller_movement.cpp \
        p_controller_rotation.cpp

HEADERS  += mainwindow.h \
    p_controller_movement.h \
    p_controller_rotation.h

FORMS    += mainwindow.ui
