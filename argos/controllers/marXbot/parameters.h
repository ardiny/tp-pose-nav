#ifndef __PARAMETERS_H
#define __PARAMETERS_H
#include "Consts.h"

// Map, contain the representation of the environment from the perspective of the robot
class Parameters
{
public:
    const int treshold;//this variable limit a number of distance scanner data
    double worldhistangle[360][400],histangle[360];
    double posx[400],posy[400],posz[400],state1[400];
    int worldhistcount;
    int Destx,Desty;
    int timer;
    Parameters();
    bool goToDest;
    //if updatestate=keep, it means the robot keeps distance data until you reset data by distance_data_clear()
    //It provides you to  make sure that you have enough data to do pose estimation and naviagtion as well, if
    // number of data exceeds a "treshold" value, it return "true"  but it makes delay
    //if updatestate=update, robot keeps updating distance data  and always return "true"
    enum Updatestate{
    keep,
    update
    };
    Updatestate updatestate;

private:

	/*Map();
	int test3();
	void computeGradientFromDestination(int destX, int destY);
	private:
	void updateGradientCell(int cellX, int cellY);*/
};

#endif
