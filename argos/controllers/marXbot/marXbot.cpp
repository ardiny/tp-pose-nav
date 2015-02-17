/* Include the controller definition */
#include "marXbot.h"
using namespace std;
using namespace boost;
/****************************************/
/****************************************/
marXbot::marXbot() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_act_scanner(NULL),
   m_scanner(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   myfile ("footbot.txt"),
   infile ("../map/offline.txt",ios::in),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}
CRadians Angle;

/****************************************/
/****************************************/

int counter(0);
string line,rr;
int k=0;
double MyFloat1,MyFloat2,nom=-9;
double hist[2][360/(6*RPM/10)*24/2+1000];
//double worldhist[2][360/(6*RPM/10)*24/2+1000][400];
const double PI = 4.0*atan(1.0);
//double worldhistangle[360][400],histangle[360];
int histcount=0,worldhistcount=0,histcounttotal=0;
int timer=0;
bool findvalue=false;
 //TODO Set destination position

bool firsttag=true;
int x,y,prex,prey;
void marXbot::buildgradientmap(){
    for (int x = 0; x < MAP_COUNT; x++)
        for (int y = 0; y < MAP_COUNT; y++)
            map1.gradientMap[x][y]=3;

}
void marXbot::Init(TConfigurationNode& t_node) {
         histcount=0;
        /*
        * Get sensor/actuator handles
        *
        * The passed string (ex. "footbot_wheels") corresponds to the XML tag of the device
        * whose handle we want to have. For a list of allowed values, type at the command
        * prompt:
        *
        * $ launch_argos -q actuators
        *
        * to have a list of all the possible actuators, or
        *
        * $ launch_argos -q sensors
        *
        * to have a list of all the possible sensors.
        *
        */
        m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
        m_scanner = GetSensor<CCI_FootBotDistanceScannerSensor> ("footbot_distance_scanner");
        m_act_scanner= GetActuator<CCI_FootBotDistanceScannerActuator>("footbot_distance_scanner");
        m_act_scanner->Enable();
        m_act_scanner->SetRPM(RPM);
        marXbot::buildgradientmap();
        m_pos=GetSensor  <CCI_PositioningSensor>("positioning");
        m_diff_sensor=GetSensor  <CCI_DifferentialSteeringSensor>("differential_steering");
        int no=0;
        /*
           *Read data from the refrence map and put them in the parameters.worldhistangle
           */
        vector <string> fields;
        while ( !infile.eof() )
            {
            getline (infile,line);
            split( fields, line, is_any_of( "," ) );
            histcounttotal=histcount;
            histcount=0;
            //std::cout<<"short-dist("<<histcount<<"--<"<< fields.size()<< std::endl;
            if (fields.size()>1)
            for (int n = 6; n < fields.size()-2; n=n+2){//*** -2
                istringstream is1 (fields[n-1]);
                istringstream isno (fields[0]);
                isno >> nom;
                no=int(nom);
                istringstream isposx (fields[1]);
                isposx >> parameters.posx[no];
                istringstream isposy (fields[2]);
                isposy >> parameters.posy[no];
                istringstream sposz (fields[3]);
                sposz >> parameters.posz[no];
                istringstream isstat1 (fields[4]);
                isstat1 >> parameters.state1[no];
                is1 >> MyFloat1;
                istringstream is2 (fields[n]);
                is2 >> MyFloat2;
                int angletempworld=(int)(MyFloat1+0.00);

                if ((angletempworld<0)&&(angletempworld>=-180))
                    angletempworld=angletempworld+360;
                parameters.worldhistangle[angletempworld][no]=MyFloat2;
                histcount++;
                    }

                if((parameters.state1[no]==0)||(parameters.state1[no]==1)||(parameters.state1[no]==2))
                    map1.gradientMap[(int)(parameters.posx[no])][(int)(parameters.posy[no])]=parameters.state1[no];
            }

        parameters.worldhistcount=no;
        //Initalize parameters.histangle
        distance_data_clear();

        map1.computeGradientFromDestination(parameters.Destx,parameters.Desty);
        GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
        m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
        GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
        GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
        const CCI_FootBotDistanceScannerSensor::TReadingsMap& tScannerRead=m_scanner->GetReadingsMap() ;
        CCI_FootBotDistanceScannerSensor::TReadingsMap::iterator ii;

}

bool marXbot::distance_data_update(int tresholdvalue){
    CCI_FootBotDistanceScannerSensor::TReadingsMap tScannerRead=m_scanner->GetReadingsMap();
    CCI_FootBotDistanceScannerSensor::TReadingsMap tScannerReadshort=m_scanner->GetShortReadingsMap();
    CCI_FootBotDistanceScannerSensor::TReadingsMap tScannerReadlong=m_scanner->GetLongReadingsMap();
    CCI_FootBotDistanceScannerSensor::TReadingsMap::iterator ii;
    tScannerRead.size();
        ///////////////////////////////////////////////////
        //receiving and sorting rotation distance scanner//
        //////////////////////////////////////////////////
      if (parameters.updatestate==parameters.keep){
          for(ii=tScannerRead.begin();ii!=tScannerRead.end(); ++ii){
                counter++;
               //std::cout<<"angle("<<counter<<")="<<(ii)->first.GetValue()*180/PI<< std::endl;
               //std::cout<<"dist("<<counter<<")="<<(ii)->second<< std::endl;
              //method 2 produce sorted histangle[i], worldhistangle[i][j]------------ i is angle(integer) between 0 and 360
                int angletemp=(int)((ii)->first.GetValue()*180.0/PI+0.01);
                if ((angletemp<0)&&(angletemp>=-180))
                    angletemp=angletemp+360;
                if ((ii)->second/1.0>0)
                        parameters.histangle[angletemp]=(ii)->second/1.0;
                if (counter>=tresholdvalue){
                return true;
                        break;
                }
        }
      }
      if (parameters.updatestate==parameters.update){
          for(ii=tScannerReadshort.begin();ii!=tScannerReadshort.end(); ++ii){
                counter++;
              // std::cout<<"short-angle("<<counter<<")="<<(ii)->first.GetValue()*180/PI<< std::endl;
               //std::cout<<"short-dist("<<counter<<")="<<(ii)->second<< std::endl;
              //method 2 produce sorted histangle[i], worldhistangle[i][j]------------ i is angle(integer) between 0 and 360
                int angletemp=(int)((ii)->first.GetValue()*180.0/PI+0.01);
                if ((angletemp<0)&&(angletemp>=-180))
                    angletemp=angletemp+360;
                if ((ii)->second/1.0!=-2.0)
                        parameters.histangle[angletemp]=(ii)->second/1.0;
          }
          for(ii=tScannerReadlong.begin();ii!=tScannerReadlong.end(); ++ii){
               counter++;
               //std::cout<<"long-angle("<<counter<<")="<<(ii)->first.GetValue()*180/PI<< std::endl;
               //std::cout<<"long-dist("<<counter<<")="<<(ii)->second<< std::endl;
              //method 2 produce sorted histangle[i], worldhistangle[i][j]------------ i is angle(integer) between 0 and 360
                int angletemp=(int)((ii)->first.GetValue()*180.0/PI+0.01);
                if ((angletemp<0)&&(angletemp>=-180))
                    angletemp=angletemp+360;
                if ((ii)->second/1.0!=-1.0)
                        parameters.histangle[angletemp]=(ii)->second/1.0;
          }
      return true;
      }
return false;
};
void marXbot::distance_data_clear(){
    for (int k=0;k<360;k++)
        parameters.histangle[k]=-1;
    counter=0;
};
void marXbot::obstacle_avoidance(){
const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
           /* Sum them together */
           CVector2 cAccumulator;
           for(size_t i = 0; i < tProxReads.size(); ++i) {
              cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
           }
           cAccumulator /= tProxReads.size();
           /* If the angle of the vector is small enough and the closest obstacle is far enough,
              continue going straight, otherwise curve a little */
           CRadians cAngle = cAccumulator.Angle();
           if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
              cAccumulator.Length() < m_fDelta ) {
              /* Go straight */
              m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
           }
           else {
              /* Turn, depending on the sign of the angle */
              if(cAngle.GetValue() > 0.0f) {
             m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
             }
              else {
             m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
              }
           }
           /* Set LED colors */
     //      m_pcLEDs->SetAllColors(CColor::BLACK);
      //     cAngle.UnsignedNormalize();
    //       UInt32 unIndex = static_cast<UInt32>(cAngle * 12.0f / CRadians::TWO_PI);
     //      m_pcLEDs->SetSingleColor(unIndex, CColor::RED);

};
/****************************************/
/****************************************/
void marXbot::ControlStep() {
        parameters.timer++;
       bool okvalue=distance_data_update(parameters.treshold);

        ////////////////////////////////
        // ---  Pos Estimation mode ---//
        ////////////////////////////////
        //TODO Find position of a robot
        //In the Tp description file, we discussed about it in Section 3.1.2

         ///////////////////////////
        // --- Navigation mode --//
        ///////////////////////////
        //TODO Navigation and find Goal
        //In the Tp description file, we discussed about (Section 3.1.3) path propagation gradient algorithm, So you can access value of gradient map with following command: map1.gradientMap[x][y]

/****************************************/
/****************************************/
 }
/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(marXbot, "marXbot_controller")
