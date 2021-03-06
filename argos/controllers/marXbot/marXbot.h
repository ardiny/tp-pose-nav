/*
 * 
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around. The LEDs are used to visually show to
 * the user the angle where closest obstacle was detected.
 *
 *    
 */

#ifndef MARXBOT_H
#define MARXBOT_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>

/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot distance scanner sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_distance_scanner_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_distance_scanner_sensor.h>


/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/angles.h>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <unistd.h>

#include "Consts.h"
#include "Map.h"
#include "parameters.h"
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class marXbot : public CCI_Controller {
public:
    Map map1;
    Parameters parameters;
    std::ofstream myfile;
    std::ifstream infile;
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
   /* Class constructor. */
   marXbot();

   /* Class destructor. */
   virtual ~marXbot() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML file
    * in the <controllers><footbot_diffusion_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Destroy() {}
   void buildgradientmap();
private:
    
   /* Pointer to the foot-bot wheels actuator */
  CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the foot-bot LEDs actuator */
  // CCI_FootBotLedsActuator*  m_pcLEDs;
   CCI_FootBotDistanceScannerActuator* m_act_scanner;
//   CCI_FootBotOmnidirectionalCameraSensor* m_pcCamera;
   /* Pointer to the foot-bot distance sensor */
   CCI_FootBotDistanceScannerSensor* m_scanner;
   CCI_PositioningSensor* m_pos;
   CCI_DifferentialSteeringSensor* m_diff_sensor;
/*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section.
    */

   /* Maximum tolerance for the angle between
    * the robot heading direction and
    * the closest obstacle detected. */
   CDegrees m_cAlpha;
   /* Maximum tolerance for the proximity reading between
    * the robot and the closest obstacle.
    * The proximity reading is 0 when nothing is detected
    * and grows exponentially to 1 when the obstacle is
    * touching the robot.
    */
   Real m_fDelta;
   /* Wheel speed. */
   Real m_fWheelVelocity;
   /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
   CRange<CRadians> m_cGoStraightAngleRange;

   //distance_data_update function could be used to keep update the histogram array and tresholdvalue is a limtit to make sure a robot obtained enought distance data
   bool distance_data_update(int tresholdvalue);
   //To reset histogram array
   void distance_data_clear();
   //this function enables a robot to perform obstacle-avoidance algorithm in the area
   void obstacle_avoidance();
};

#endif
