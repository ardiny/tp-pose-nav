#ifndef CALIBRATION_H
#define CALIBRATION_H
#include<iostream>
#include <QtCore/QDebug>
#include"parameters.h"
class calibration
{
public:
    Parameters parameters;
     int myArray[10];
    calibration();
    void tets();
    float getPos(const int* table, const int tableLength, const int value);
    float getNearest(float* tab, int angle) ;
    float recalculateDist(int angle);
    double getDist(int angle);
    double filter1(double distvalue,int angle);
    int a;
    const int sensor0LT[41];
    const int sensor1LT[41];
    // 20 to 150 cm included + 0 to 20 cm in another table
    const int sensor2LT[131];
    const int sensor2NearLT[20];
    const int sensor3LT[131];
    const int sensor3NearLT[20];
    float scannerDataClose[360];
    float scannerDataCloseRaw[360];
    float scannerDataFar[360];
    float scannerDataFarRaw[360];
    float worlddata[360];
    double dist_robot[360];
private:
    //calibration parameters

};

#endif // CALIBRATION_H
