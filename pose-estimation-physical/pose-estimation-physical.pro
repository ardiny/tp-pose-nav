QT       += core

QT       -= gui

TARGET = pose-estimation-physical
CONFIG   += console
CONFIG += qdbus
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    calibration.cpp \
    parameters.cpp
SOURCES += Map.cpp
SOURCES += Consts.cpp
 # install
 target.path = $$[QT_INSTALL_EXAMPLES]/dbus/listnames
 sources.files = $$SOURCES $$HEADERS $$RESOURCES *.pro
 sources.path = $$[QT_INSTALL_EXAMPLES]/dbus/listnames
 INSTALLS += target sources

 symbian: include($$QT_SOURCE_TREE/examples/symbianpkgrules.pri)

HEADERS += \
    main.h \
    parameters.h \
    calibration.h
