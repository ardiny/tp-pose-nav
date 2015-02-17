#ifndef PARAMETERS_H
#define PARAMETERS_H
#include <QtCore/QVariant>


class Parameters
{
public:
    const int treshold;//this variable limit a number of distance scanner data
    double worldhistangle[360][400],histangle[360],histweight[360];
    double posx[400],posy[400],posz[400],state1[400];
    int worldhistcount;
    int Destx,Desty;
    int timer;
    bool goToDest;
    unsigned int interval_control_function;
    QVariantList speedvalueleft,speedvalueright ;
    Parameters();
    enum Updatestate{
    keep,
    update
    };
    Updatestate updatestate;
    enum Calibration_state{
    just_robot_distance,
    just_recalculate_distance,
    both
    };
    Calibration_state calibration_state;


private:
};
#endif // PARAMETERS_H
