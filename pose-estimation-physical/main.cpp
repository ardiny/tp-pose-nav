//in the simulation
//	      front
//	        0
//left 1		3 right
//	        2
//	      back
//      battery
//in the real
//	      front
//	        0
//left 3		2 right
//	        1
//	      back
//      battery
#include"main.h"
using namespace std;
//unsigned int collect_distance_data();
unsigned counter;
int histcount;
int laserop[5];
const string newline[]="";
int k=0;
int numberangle=0;
int no=-1;
Map map1;
int x,y,prex,prey;
double mostProbDist=1000000;
//replacement of sensors arrangement
void initialize(){
    int MAP_COUNTX=(int)(arenalon/arenares+1);
    int MAP_COUNTY=(int)(arenawidth/arenares+1);
    for (int x = 0; x < MAP_COUNTX; x++)
        for (int y = 0; y < MAP_COUNTY; y++)
           {
            map1.gradientMap[x][y]=3;
           };
    distance_data_clear();
     ///////////////////////
    //reading offline map//
    //////////////////////
    QFile fin("../map/offline.txt");
    QTextStream in(&fin);
    fin.open(QIODevice::ReadOnly | QIODevice::Text);
    QString line;
    QStringList  fields;
    while(!in.atEnd()) {
               line = in.readLine();
               fields = line.split(",");
               no= fields.at(0).toInt();
               parameters.posx[no]= fields.at(1).toFloat();
               parameters.posy[no]= fields.at(2).toFloat();
               parameters.posz[no]= fields.at(3).toFloat();
               parameters.state1[no]= fields.at(4).toInt();
               histcount=0;
               for (int n = 6; n < fields.size()-1; n=n+2){
                       if ((fields.at(n-1).toInt()>=0)&&(fields.at(n-1).toInt()<=180))
                            parameters.worldhistangle[fields.at(n-1).toInt()][no]=fields.at(n).toFloat();
                       if ((fields.at(n-1).toInt()>=-180)&&(fields.at(n-1).toInt()<0))
                            parameters.worldhistangle[fields.at(n-1).toInt()+360][no]=fields.at(n).toFloat();
                       histcount++;
                       }

               if((parameters.state1[no]==0)||(parameters.state1[no]==1)||(parameters.state1[no]==2))
                                   map1.gradientMap[(int)(parameters.posx[no])][(int)(parameters.posy[no])]=parameters.state1[no];
    }
    parameters.worldhistcount=no;
     map1.computeGradientFromDestination(parameters.Destx,parameters.Desty);
}

int main(int argc, char **argv){
    ///////////////////////////
    // -- initializing --//
   ///////////////////////////
   int angle;
   int laser[4];
   int laserraw[4];
   QTime time;
   int angle0=-1;
   QCoreApplication app(argc, argv);
   if (!QDBusConnection::sessionBus().isConnected()) {
          fprintf(stderr, "Cannot connect to the D-Bus session bus.\n"
                  "To start it, run:\n"
                  "\teval `dbus-launch --auto-syntax`\n");
          return -1 ;
      }
   QDBusConnection bus = QDBusConnection::sessionBus();
   QDBusInterface dbus_iface("ch.epfl.mobots.Aseba", "/", "ch.epfl.mobots.AsebaNetwork",bus);
   //Load Aesl file on the Aseba via D-bus
   dbus_iface.call("LoadScripts","tp.aesl");
   initialize();
   qDebug() <<"Start";
   time.start();
   while (1){
       ///////////////////////////
       // -- Acquisition --//
      ///////////////////////////
        QDBusMessage input=dbus_iface.call( "GetVariable","sensor-turret","buf");
        //qDebug() <<time.elapsed()<<input;
       angle=input.arguments().value(8).toInt();
               if (angle!=angle0)
                    counter++;
                angle0=angle;
    /////////////using proximity sensors
    // QDBusMessage input_proximity=dbus_iface.call( "GetVariable","base-sensors","proximity.corrected");
   //   qDebug() <<time.elapsed()<<input_proximity.arguments().value(1).toInt();
        if ((parameters.calibration_state==parameters.just_recalculate_distance)||(parameters.calibration_state==parameters.both)){
                        //qDebug() <<dbus_iface.call( "GetVariable","sensor-turret","sharp.value");
                        for(size_t i=0;i<4;i++)
                              laserraw[i]=input.arguments().value(i).toInt();
                        laser[0]=calib.getPos(calib.sensor0LT, 41, laserraw[0]);
                        laser[1]=calib.getPos(calib.sensor1LT, 41, laserraw[1]);
                        laser[2]=calib.getPos(calib.sensor2LT, 131, laserraw[2]);
                        laser[3]=calib.getPos(calib.sensor3LT, 131, laserraw[3]);
                        calib.scannerDataClose[(angle + 180) % 360] = calib.getPos(calib.sensor0LT, 41, laserraw[0]);
                        calib.scannerDataCloseRaw[(angle + 180) % 360] =laserraw[0];
                        calib.scannerDataClose[(angle) % 360] = calib.getPos(calib.sensor1LT, 41, laserraw[1]);
                        calib.scannerDataCloseRaw[(angle) % 360] =laserraw[1];
                        calib.scannerDataFar[(angle + 90) % 360] = calib.getPos(calib.sensor2LT, 131, laserraw[2]);
                        calib.scannerDataFarRaw[(angle + 90) % 360] = laserraw[2];
                        calib.scannerDataFar[(angle + 270) % 360] = calib.getPos(calib.sensor3LT, 131, laserraw[3]);
                        calib.scannerDataFarRaw[(angle + 270) % 360] =laserraw[3];


        }
        if ((parameters.calibration_state==parameters.just_robot_distance)||(parameters.calibration_state==parameters.both)){
            for(size_t i=0;i<4;i++){
                    double temp=input.arguments().value(i+4).toInt();
                    if (parameters.updatestate=parameters.keep){
                            if (temp>0){
                                temp=temp-10.3+8.5-7;
                                  if (i==0)
                                      calib.dist_robot[(angle + 180) % 360]=temp; //close
                                  if (i==1)
                                      calib.dist_robot[(angle) % 360]=temp;//close
                                  if (i==2)
                                      calib.dist_robot[(angle + 90) % 360]=temp;//far
                                  if (i==3)
                                      calib.dist_robot[(angle + 270) % 360]=temp;//far
                             }
                    }
                    if (parameters.updatestate=parameters.update){
                        if (temp!=-1){
                            temp=temp-10.3+8.5-7;
                              if (i==2)
                                  calib.dist_robot[(angle + 90) % 360]=temp;//far
                              if (i==3)
                                  calib.dist_robot[(angle + 270) % 360]=temp;//far
                        }
                        if (temp!=-2){
                            temp=temp-10.3+8.5-7;
                            if (i==0)
                                calib.dist_robot[(angle + 180) % 360]=temp; //close
                            if (i==1)
                                calib.dist_robot[(angle) % 360]=temp;//close
                        }


                    }
               }

         }

        ////////////////////////////////
        // -- Call control function --//
       /////////////////////////////////
       if (time.elapsed()>parameters.interval_control_function){
           parameters.speedvalueright.clear();
           parameters.speedvalueleft.clear();
           controller();
           time.start();
           dbus_iface.call("SetVariable","treel-right", "motor.pid.target_speed",parameters.speedvalueright);
           dbus_iface.call("SetVariable","treel-left", "motor.pid.target_speed",parameters.speedvalueleft);

       }
     }
};
void controller(){
    parameters.timer++;
    bool okvalue=distance_data_update(parameters.treshold);
    ////////////////////////////////
    // ---  Pos Estimation mode ---//
    ////////////////////////////////
    //TODO Find position of a robot
    //In the Tp description file, we discussed about it in Section 3.1.2 and 3.2.2



    ///////////////////////////
    // --- Navigation mode --//
    ///////////////////////////
    //TODO Navigation and find Goal
    //In the Tp description file, we discussed about (Section 3.1.3) path propagation gradient algorithm, So you can access value of gradient map with following command: map1.gradientMap[x][y]
 
 ////Please use below speed commands to send your value to the robot
 ////Please note the robot multiply a coefficient the speed values and then convert it to  integer values , robot_speed=(int)(your_speed value*10.0/4.0) 
  //  parameters.speedvalueleft.clear();
  //  parameters.speedvalueright.clear();
 //   parameters.speedvalueleft << -0;
  //  parameters.speedvalueright << 0;
        return;
    }
bool distance_data_update(int tresholdvalue){
    if (parameters.updatestate==parameters.keep){
        if (counter>tresholdvalue){
               // qDebug() <<"counter="<<counter;
                for (int i=0;i<360;i++){
                         parameters.histangle[i]=calib.filter1(calib.getDist(i),i);
                           // qDebug() <<"angle["<<i<<"]="<<parameters.histangle[i];
                     }
                     return true;
            }
    }
    if (parameters.updatestate==parameters.update){
           //  qDebug() <<"counter="<<counter;
                 for (int i=0;i<360;i++){
                     parameters.histangle[i]=calib.filter1(calib.getDist(i),i);
                        //qDebug() <<"angle["<<i<<"]="<<parameters.histangle[i];
                 }

    }
        return true;
return false;
}
void distance_data_clear(){

            for (int i=0;i<360;i++){
                    calib.scannerDataClose[i] = -1;
                    calib.scannerDataFar[i] = -1;
                    calib.scannerDataCloseRaw[i] = -1;
                    calib.scannerDataFarRaw[i] = -1;
                    calib.dist_robot[i]=-1;
                    parameters.histangle[i]=-1;
                    parameters.histweight[i]=0;
            }
            counter=0;

};
void obstacle_avoidance(){
    parameters.speedvalueleft.clear();
    parameters.speedvalueright.clear();
    parameters.speedvalueleft << -0*pow(-1,parameters.timer);
    parameters.speedvalueright << 0*pow(-1,parameters.timer);
}



