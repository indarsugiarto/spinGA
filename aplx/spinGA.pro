TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    spinGA.c \
    gamodel.c \
    MT19937.c \
    main.c \
    eventhandler.c \
    init.c \
    util.c

INCLUDEPATH += /opt/spinnaker_tools_134/include

DISTFILES += \
    Makefile \
    HOW.DOES.IT.WORK

HEADERS += \
    spinGA.h
