#ifndef MAIN_H
#define MAIN_H
#include<iostream>
#include <vector>
#include <QtCore/QVariant>
#include <QtCore/QCoreApplication>
#include <QtCore/QDebug>
#include <QtCore/QStringList>
#include <QtDBus/QtDBus>
#include <QtDBus/QDBusInterface>
#include <QtDBus/QDBusMessage>
#include <QtDBus/QDBusAbstractInterface>
#include <QtCore/QFile>
#include <QtCore/QDebug>
#include <QtCore/QProcess>
#include <QtCore/QList>
#include <QObject>
#include <QtDBus/QDBusReply>
#include <QFile>
#include <QtCore/QTimer>
#include <sys/time.h>
#include <boost/thread/thread.hpp>
#include <unistd.h>
#include "../argos/controllers/marXbot/Map.h";
#include "../argos/controllers/marXbot/Consts.h";
#include "parameters.h";
#include"calibration.h"
calibration calib;
Parameters parameters;
    void initialize();
    void controller();
    bool distance_data_update(int tresholdvalue);
    void distance_data_clear();
    void obstacle_avoidance();


#endif // MAIN_H
