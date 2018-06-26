#-------------------------------------------------
#
# Project created by QtCreator 2018-06-19T15:32:26
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = siample-remogui
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

QMAKE_CXXFLAGS += -std=c++0x -Werror=return-type

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

unix:LIBS += -Wl,--start-group \
    -lswresample \
    -lavdevice \
    -lavformat \
    -lavcodec \
    -lavutil \
    -lswscale \
    -lavfilter \
    -Wl,--end-group

unix:LIBS += -lvdpau -lva -lX11 -lva-drm -lva-x11

unix:LIBS += -ldl -pthread -lm

unix:LIBS += -lboost_system -lboost_thread -lboost_filesystem -lboost_regex -lboost_chrono -lboost_date_time -lboost_atomic

unix:LIBS += -lglog


SOURCES += \
        main.cpp \
        mainwindow.cpp \
    MediaStreamProc.cpp \
    commlog.cpp \
    MediaStreamWidget.cpp

HEADERS += \
        mainwindow.h \
    MediaStreamProc.h \
    commlog.h \
    MediaStreamWidget.h \
    libavformat/rtsp.h \
    libavformat/rtpdec.h \
    config.h \
    ffmpeg-custom.h

FORMS += \
        mainwindow.ui
