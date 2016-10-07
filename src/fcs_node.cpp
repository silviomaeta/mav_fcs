/**
 * @file   fcs_node.cpp
 * @author Silvio Maeta
 * @date   04/24/2016
 * @brief  
 *
 * @copyright
 * Copyright (C) 2016.
 */

#include <limits>
#include <math.h>

#include <atomic>
#include <thread>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "mav_fcs/fcs_interface.h"
#include "mav_fcs/fcs_processor.h"

using namespace mavfcs;

//==============================================================================

void updateCopterInterfaceThread(FcsProcessor *fcsproc) 
{
  //Copter interface updates at 50 Hz - DJI requirement
  double update_rate = 50.0;
  ros::Rate rate(update_rate);
  while (ros::ok())
  {
    //ROS_INFO_STREAM_THROTTLE(1.0, "[FCS] Thread loop - update copter interface");
    //Send commands for DJI onboard control system
    fcsproc->updateCopterInterface();
    rate.sleep();
  }
}


//==============================================================================

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "fcs");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  //----------------------------------------------------------------------------
  //Initialization

  bool isSimulation = true;
  pnh.getParam("simulation", isSimulation);
  ROS_INFO("Simulation: %s", isSimulation?"True":"False");

  FcsInterface fcsint(pnh, ros::Time::now(), isSimulation);

  FcsProcessor fcsproc(pnh, nh, &fcsint);

  //Setup Subscribers
  
  //Subscribe to external odometry (world frame)
  ros::Subscriber odom_sub = pnh.subscribe(
      "pose_estimation", 10, &FcsInterface::setOdometry, &fcsint);
  
  //Subscribe to path waypoints (world frame)
  ros::Subscriber path_sub = pnh.subscribe(
      "path", 10, &FcsInterface::setPath, &fcsint);
  
  //Subscribe to user commands (takeoff / land / start / stop / pause / resume)
  ros::Subscriber cmd_sub = pnh.subscribe(
      "user_cmd", 10, &FcsInterface::setUserCmd, &fcsint);
  
  

  //Setup Publishers

  //Publish FCS Odometry
  ros::Publisher odometry_pub = pnh.advertise<nav_msgs::Odometry>("odometry", 10, false);

  //Publish FCSStatus
  ros::Publisher status_pub = pnh.advertise<mav_gcs_msgs::FCSStatus>("fcs_status", 10, false);
  
  
  //Initialization of FCS objects
 
  fcsproc.initialize();

  //----------------------------------------------------------------------------
  //Initialize thread that supports copter interface
  sleep(5.0);

  std::thread updateCopterThread(updateCopterInterfaceThread, &fcsproc);

  //----------------------------------------------------------------------------
  //Main loop
  //double update_rate = 10.0;
  //double dt = 1.0/update_rate;
  //fcsproc.setPeriod(dt);

  //Update state machine and copter interface status
  fcsproc.update();

  //int counter = 0;
  ros::Rate rate(50.0);
  while (ros::ok())
  {
    //Spin at 100 Hz to get updated topics
    ros::spinOnce();
  
    try
    {
      //Run status update at 20 Hz
      //counter++;
      //if (counter == 5) {
      //counter = 0;

      //ros::Time now = ros::Time::now();

      //------------------------------------------------------------------------
      //Update state step
     
      //Update state machine and copter interface status
      fcsproc.update();

      //------------------------------------------------------------------------
      //Publish data step

      odometry_pub.publish(fcsint.getFcsOdometry());

      status_pub.publish(fcsint.getFcsStatus());
            
      //}
    }
    catch (const std::exception & ex)
    {
      ROS_ERROR_THROTTLE(5.0, "[FCS] Cannot update simulation: %s", ex.what());
    }

    rate.sleep();
  }

  updateCopterThread.join();
  
  return 0;
}

