TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

LIBS += -lgtest

INCLUDEPATH += \
        ../firmware/inc

HEADERS += \
        ../firmware/inc/commons.h

SOURCES += \
        ../firmware/src/commons.c \
        main.cpp \
        t_commons.cpp
