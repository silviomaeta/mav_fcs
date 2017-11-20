/*
* Copyright (c) 2016 Carnegie Mellon University, Silvio Maeta <smaeta@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#ifndef _MAV_COPTER_INTERFACE_H_
#define _MAV_COPTER_INTERFACE_H_

#include <string>

#include <nav_msgs/Odometry.h>

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

  int setVelocityCommand(double vx, double vy, double vz, double yaw);

  nav_msgs::Odometry getOdometry(void);

  int getBatteryLevel(void);

  std::string getStatusDescriptionStr(void);
  
private:

  bool _is_initialized;

};

}

#endif /* _MAV_COPTER_INTERFACE_H_ */

