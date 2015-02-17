/* Include the controller definition */
#include "marXbot.h"
//#include <iostream>
#include <fstream>
/* Function definitions for XML parsing */
//#include <argos2/common/utility/configuration/argos_configuration.h>
/* 2D vector definition */
//#include <argos2/common/utility/math/vector2.h>
#include <stdio.h>
//#include <argos2/common/utility/math/angles.h>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <vector>
#include <string>
#include "Consts.h"
#include <sstream>
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
   infile ("final.txt",ios::in),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}
CRadians Angle;

/****************************************/
/****************************************/
int timer=0;
int uu;
int jj(0);
string line,rr;
int k=0;
const int limits = 2;
double hist[2][360/(6*RPM/10)*24/2+1000];
const double PI = 4.0*atan(1.0);
vector <string> fields;
double MyFloat1,MyFloat2,no,posx,posy,posz1,stat1;
double worldhist[2][360/(6*RPM/10)*24/2+1];
int histcount=0,kj=0;
size_t n;
double dist=0,v=0;
int minDist;
int bestDir;
double mostProbDist=1000000;
int subj,subi;
double temp1,temp2;
bool findvalue=false;
void marXbot::Init(TConfigurationNode& t_node) {
histcount=0;
kj=0;
 dist=0,v=0;
 minDist;
 bestDir;
 mostProbDist=1000000; 
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
    * NOTE: ARGoS creates and initializes actuators and sensors internally, on the basis of
    *       the lists provided the XML file at the <controllers><footbot_diffusion><actuators>
    *       and <controllers><footbot_diffusion><sensors> sections. If you forgot to
    *       list a device in the XML and then you request it here, an error occurs.
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
 /*
    * Parse the XML
    *
    * The user defines this part. Here, the algorithm accepts three parameters and it's nice to
    * put them in the XML so we don't have to recompile if we want to try other settings. footbot_distance_scanner
    */
  
 m_act_scanner->Enable();
 m_act_scanner->SetRPM(RPM); 

for (int k=0;k<360/(6*RPM/10)*24/2+1000;k++){
hist[0][k]=400;
hist[1][k]=0;
};
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
   const CCI_FootBotDistanceScannerSensor::TReadingsMap& tScannerRead=m_scanner->GetReadingsMap() ;
   CCI_FootBotDistanceScannerSensor::TReadingsMap::iterator ii;
}
/****************************************/
/****************************************/
void marXbot::ControlStep() {
cout<<timer++<<" ";
const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
CCI_FootBotDistanceScannerSensor::TReadingsMap tScannerRead=m_scanner->GetReadingsMap();
CCI_FootBotDistanceScannerSensor::TReadingsMap::iterator ii;
tScannerRead.size();
for(ii=tScannerRead.begin();ii!=tScannerRead.end(); ++ii){
 	//std::cout<<"f("<<jj<<")="<<(ii)->first.GetValue()*180/PI<< std::endl;   
	//std::cout<<"s("<<jj<<")="<<(ii)->second<< std::endl;
	jj++;   

	findvalue=false; 
	
 			for (int subk=0;subk<=k;subk++)
				if (abs(hist[0][subk]-((ii)->first.GetValue()*180/PI))<0.01){		
				if (hist[1][subk]<0){

				hist[1][subk]=((ii)->second);
				}
				findvalue=true;
				break;				
				}
		        if(!findvalue)
			{
				
				hist[0][k]=((ii)->first.GetValue()*180/PI);
				hist[1][k]=((ii)->second/1.0);
				k++;
				
			}
			for (int subi=0;subi<=k;subi++)
				for (int subj=subi;subj<=k;subj++)
				{
					if(hist[0][subj]<=hist[0][subi]){
					temp1=hist[0][subi];
					temp2=hist[1][subi];
					hist[0][subi]=hist[0][subj];
					hist[1][subi]=hist[1][subj];
					hist[0][subj]=temp1;
					hist[1][subj]=temp2;
					}

				}	
// --- used only in make offline map mode ----
  /* */      	if (myfile.is_open())
  		{
			if (jj>=360/(6*RPM/10)*24+2){
			for (int subk=0;subk<k+1;subk++){

			myfile <<hist[0][subk]<<","<<hist[1][subk]<<",";
			};

			myfile.close();
			exit(1);
			}; 
		}
  }
}
/****************************************/
/****************************************/
/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(marXbot, "marXbot_controller")
