/**
 * @file   copter_interface.cpp
 * @author Silvio Maeta
 * @date   04/28/2016
 * @brief  Interface with copter onboard system
 *
 * @copyright
 * Copyright (C) 2016.
 */

#ifndef _MAV_COPTER_INTERFACE_H_
#define _MAV_COPTER_INTERFACE_H_

#include <string>

#include <nav_msgs/Odometry.h>

#define HORIZ_ATT 0x00
#define HORIZ_VEL 0x40
#define HORIZ_POS 0x80
#define VERT_VEL 0x00
#define VERT_POS 0x10
#define VERT_TRU 0x20
#define YAW_ANG 0x00
#define YAW_RATE 0x08
#define HORIZ_GND 0x00
#define HORIZ_BODY 0x02
#define YAW_GND 0x00
#define YAW_BODY 0x01

namespace mavfcs {

enum FlightState {
  INVALID = -1,
  ON_GROUND = 0,
  TAKING_OFF = 1,
  IN_AIR = 2,
  LANDING = 3
};

class CopterInterface {

public:
  CopterInterface(ros::NodeHandle & nh);

  ~CopterInterface(void);

  int initialize(void);
  
  void terminate(void);
   
  bool isEngaged(void);
  
  FlightState getFlightState(void);

  int takeOff(void);

  int land(void);

  int startVideo(void);

  int stopVideo(void);

  int setVelocityCommand(double vx, double vy, double vz, double yawrate);
  
  int setGimbalCommand(double r_rate, double p_rate, double y_rate);

  nav_msgs::Odometry getOdometry(void);

  int getBatteryLevel(void);

  std::string getStatusDescriptionStr(void);
  
private:

  bool _is_initialized;

};

}

#endif /* _MAV_COPTER_INTERFACE_H_ */

