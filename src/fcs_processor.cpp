/**
 * @file   fcs_processor.cpp
 * @author Silvio Maeta
 * @date   04/27/2016
 * @brief  
 *
 * @copyright
 * Copyright (C) 2016.
 */

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "mav_fcs/fcs_processor.h"

using namespace mavfcs;

//==============================================================================

bool getCoordinateTransform(const tf::TransformListener &listener,
                            std::string fromFrame,
                            std::string toFrame,
                            double waitTime,
                            tf::StampedTransform &transform) {
  bool flag = true;
  double timeResolution = 0.001;
  unsigned int timeout = ceil(waitTime/timeResolution);
  unsigned int count = 0;

  while(flag) {
    flag = false;
    try {
      count++;
      listener.lookupTransform(toFrame,fromFrame,ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      flag = true;
      if(count>timeout) {
        flag = false;
        ROS_ERROR_STREAM("Cannot find transform from::"<<fromFrame << " to::"<< toFrame);
        return flag;
      }
      ros::Duration(timeResolution).sleep();
    }
  }
  return (!flag);
}

void transformPoint(const tf::Transform &transform,
                    geometry_msgs::Point& point) {
  tf::Point pAux,pOut;
  tf::pointMsgToTF(point, pAux);
  pOut = transform * pAux;
  tf::pointTFToMsg(pOut, point);
}

/*
void transformXYZYaw(const tf::TransformListener &listener,
                     std::string fromFrame,
                     std::string toFrame,
                     NEA::small_copter_interface::XYZYaw &pos) {

    tf::StampedTransform stampedTransform;
    if(!getCoordinateTransform(listener, fromFrame, toFrame, 0.1, stampedTransform)) {
        ROS_WARN_STREAM("[FcsProcessor] Couldn't find transform between::"<<fromFrame<<" to "<<toFrame);
        return;
    }
    tf::Transform transform = stampedTransform;

    geometry_msgs::Point point;
    point.x = pos.x;
    point.y = pos.y;
    point.z = pos.z;

    transformPoint(transform, point);

    pos.x = point.x;
    pos.y = point.y;
    pos.z = point.z;
    
    //@TODO: check if this orientation conversion is correct for point
    tf::Quaternion q = transform.getRotation();
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    pos.yaw = pos.yaw + yaw;
}
*/

//==============================================================================


FcsProcessor::FcsProcessor(ros::NodeHandle & traj_controller_nh, ros::NodeHandle & copter_nh, FcsInterface *fcsint)
{
  _fcs_interface = fcsint;
  _state_machine = new FcsStateMachine(&_fsm_data);  
  _copter_interface = new CopterInterface(copter_nh);

  _user_cmd   = mav_gcs_msgs::UserCmd::CMD_NONE;
  _last_event = mav_gcs_msgs::FCSStatus::EVT_NONE;

  initializeTrajectoryControl(traj_controller_nh);
}


FcsProcessor::~FcsProcessor(void)
{
  delete _controller;
  delete _state_machine;
  delete _copter_interface;
  //if (_curr_traj != NULL) delete _curr_traj;
}


int FcsProcessor::initialize(void)
{
    _copter_interface->initialize();
    return 0;
}


void FcsProcessor::terminate(void) 
{
    _copter_interface->terminate();
}


void FcsProcessor::update(void)
{
  /*
  if (!_copter_interface->isEngaged()) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "FcsProcessor::update() - copter is not engaged.");
    return;
  }
  */

  //Update if new trajectory is available
  trajectoryUpdate();
 
  //Update speed control command based on current state
  if (_has_valid_traj && (!_traj_completed)) {
    generateSpeedControlCommand();
  }

  //Update odometry that will be published by FCS Interface
  _fcs_interface->setFcsOdometry(_copter_interface->getOdometry());

  //Update FCS status
  _fcs_interface->setFcsStatus( getFcsStatus() );

  //_fcs_interface->setTrajectoryTime( getCurrentTrajectoryTime() );
  
  //Updating received user command
  if (_fcs_interface->hasUserCmd()) {
    _user_cmd = _fcs_interface->getUserCmd().user_cmd;
  }

  _state_machine->update();

  //Show state machine status
  if (_state_machine->getCurrentState() == SM_ONGROUND) {
    ROS_INFO_STREAM_THROTTLE(1.0, "[StateMachine] SM_ONGROUND");
  }
  else if (_state_machine->getCurrentState() == SM_TAKINGOFF) {
    ROS_INFO_STREAM_THROTTLE(1.0, "[StateMachine] SM_TAKINGOFF");
  }
  else if (_state_machine->getCurrentState() == SM_EXECUTINGCOMMAND) {
    ROS_INFO_STREAM_THROTTLE(1.0, "[StateMachine] SM_EXECUTINGCOMMAND");
  }
  else if (_state_machine->getCurrentState() == SM_LANDING) {
    ROS_INFO_STREAM_THROTTLE(1.0, "[StateMachine] SM_LANDING");
  }
  else if (_state_machine->getCurrentState() == SM_HOVER) {
    ROS_INFO_STREAM_THROTTLE(1.0, "[StateMachine] SM_HOVER");
  }
  else {
    ROS_INFO_STREAM_THROTTLE(1.0, "[StateMachine] UNKNOWN");
  }
}


void FcsProcessor::updateCopterInterface(void) {

  std::string desc = _copter_interface->getStatusDescriptionStr();
  ROS_INFO_STREAM_THROTTLE(2.0, desc);
  if (!_copter_interface->isEngaged())
  {
  }


  //Update state machine - engaged state or not
  if (_copter_interface->isEngaged()) _state_machine->engage();
  else                                _state_machine->disengage();

  //Update state machine - aircraft ON_GROUND ro IN_AIR
  FlightState copter_state = _copter_interface->getFlightState();
  if (copter_state == ON_GROUND) _state_machine->onGround();
  else                           _state_machine->inAir();


  FlightState state = _copter_interface->getFlightState();
  bool commanded_takeoff = false;
  bool commanded_land = false;
  switch (state) {
    //--------------------------------------------------------------------------
    case ON_GROUND:
      ROS_INFO_STREAM_THROTTLE(1.0, ">>> Copter State: ON_GROUND");
      
      //Update copter interface
      if (_user_cmd == mav_gcs_msgs::UserCmd::CMD_TAKEOFF) {
        _copter_interface->takeOff();
        commanded_takeoff = true;
        _user_cmd = mav_gcs_msgs::UserCmd::CMD_NONE;
      }

      //Update state machine
      if (_state_machine->getCurrentState() == SM_LANDING) {
        _state_machine->landingCompleted();
      }

    break;
    //--------------------------------------------------------------------------
    case TAKING_OFF:
      ROS_INFO_STREAM_THROTTLE(1.0, ">>> Copter State: TAKING_OFF");

      //Update state machine
      if (_state_machine->getCurrentState() == SM_ONGROUND) {
        _state_machine->takeOffCommanded();
      }

    break;
    //--------------------------------------------------------------------------
    case IN_AIR:
      ROS_INFO_STREAM_THROTTLE(1.0, ">>> Copter State: IN_AIR");

      //Update copter interface
      if (_user_cmd == mav_gcs_msgs::UserCmd::CMD_LAND) {
        _copter_interface->land();
        commanded_land = true;
        _user_cmd = mav_gcs_msgs::UserCmd::CMD_NONE;
      }
      else if (_user_cmd == mav_gcs_msgs::UserCmd::CMD_MISSION_START) {
        ROS_WARN_STREAM("Follow Trajectory");
        _follow_traj = true;
        _user_cmd = mav_gcs_msgs::UserCmd::CMD_NONE;
      }
      else if (_user_cmd == mav_gcs_msgs::UserCmd::CMD_MISSION_STOP) {
        _follow_traj = false;
        _user_cmd = mav_gcs_msgs::UserCmd::CMD_NONE;
      }
      
      if ((_has_valid_traj) && (_follow_traj) && (!_traj_completed)) {
        //generate smooth speed control - use current speed, target speed and vel/acc limits
        nav_msgs::Odometry dji_odometry_msg = _copter_interface->getOdometry();
        //ROS_INFO_STREAM_THROTTLE(1.0, "[FcsProcessor] TargetSpeedCmd: vx=" << _speed_cmd.velocity[0] << " / vy=" << _speed_cmd.velocity[1] << " / vz=" << _speed_cmd.velocity[2] << " / yaw=" << _speed_cmd.heading);
        VelAccPsi cmd = ComputeSpeedControl(_period, dji_odometry_msg, _speed_control_params, _speed_cmd);
        double vx = cmd.velocity[0];
        double vy = cmd.velocity[1];
        double vz = cmd.velocity[2];
        double yaw = cmd.heading;
        _copter_interface->setVelocityCommand(vx, vy, vz, yaw);
        //ROS_INFO_STREAM_THROTTLE(1.0, "[FcsProcessor] SpeedCmd: vx=" << vx << " / vy=" << vy << " / vz=" << vz << " / yaw=" << yaw);
        
        if (isInsideCaptureRadius()) {
          _traj_completed = true;
          _has_valid_traj = false;
          _follow_traj = false;
        }
        
      }
      else {
        nav_msgs::Odometry odom = _copter_interface->getOdometry();
        tf::Quaternion q;
        q[0] = odom.pose.pose.orientation.x; 
        q[1] = odom.pose.pose.orientation.y; 
        q[2] = odom.pose.pose.orientation.z; 
        q[3] = odom.pose.pose.orientation.w; 
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        _copter_interface->setVelocityCommand(0.0, 0.0, 0.0, yaw);
      }
      
      //Update state machine
      if (_state_machine->getCurrentState() == SM_TAKINGOFF) {
        _state_machine->takeOffCompleted();
      }

      if (_state_machine->getCurrentState() == SM_HOVER) {
        if ((_has_valid_traj) && (_follow_traj) && (!_traj_completed)) {
          _state_machine->trajectoryCommanded();
        }
      }
      
      if (_state_machine->getCurrentState() == SM_EXECUTINGCOMMAND) {
        if (_traj_completed) {
          _state_machine->trajectoryCompleted();
        }
      }

      
    break;
    //--------------------------------------------------------------------------
    case LANDING:
      ROS_INFO_STREAM_THROTTLE(1.0, ">>> New Copter State: LANDING");

      //Update state machine
      if ((_state_machine->getCurrentState() == SM_HOVER) ||
          (_state_machine->getCurrentState() == SM_EXECUTINGCOMMAND)) {
        _state_machine->landingCommanded();
      }

    break;
    //--------------------------------------------------------------------------
    default:
      ROS_INFO_STREAM(">>> Copter State: UNKNOWN");
    break;
  }
}


mav_gcs_msgs::FCSStatus FcsProcessor::getFcsStatus(void) {
  static int msg_counter = 0;
  mav_gcs_msgs::FCSStatus fcsStatus;
  fcsStatus.header.stamp = ros::Time::now();
  fcsStatus.index = msg_counter++;
  if (_copter_interface->isEngaged()) fcsStatus.mode = mav_gcs_msgs::FCSStatus::AUTONOMOUS;
  else                                fcsStatus.mode = mav_gcs_msgs::FCSStatus::MANUAL;
  FlightState state = _copter_interface->getFlightState();
  switch (state) {
    case ON_GROUND:
      fcsStatus.state = mav_gcs_msgs::FCSStatus::ONGROUND;
    break;
    case TAKING_OFF:
      fcsStatus.state = mav_gcs_msgs::FCSStatus::TAKINGOFF;
    break;
    case IN_AIR:
      if ((_has_valid_traj) && (_follow_traj) && (!_traj_completed)) {
        fcsStatus.state = mav_gcs_msgs::FCSStatus::EXECUTINGCOMMAND;
      }
      else {
        fcsStatus.state = mav_gcs_msgs::FCSStatus::HOLDINGPOSITION;
      }
    break;
    case LANDING:
      fcsStatus.state = mav_gcs_msgs::FCSStatus::LANDING;
    break;
    default:
    break;
  }
  fcsStatus.last_event    = _last_event;
  fcsStatus.battery_level = _copter_interface->getBatteryLevel();
  fcsStatus.rc_comm_ok    = true;
  
    
  return fcsStatus;
}


//==============================================================================
// Trajectory handling methods
//==============================================================================

void FcsProcessor::initializeTrajectoryControl(ros::NodeHandle & traj_controller_nh) {
  
  //Set default period as 50Hz
  _period = 1.0/50.0;
  
  // Load parameters
  if(!_controller_parameters.LoadParameters(traj_controller_nh)) {
    ROS_ERROR_STREAM("[FcsProcessor] Failed to load path control parameters");
    return;
  }
  // Initialize path tracking control
  _controller = new PathTrackingControl(_controller_parameters);
  
  if (!_speed_control_params.LoadParameters(traj_controller_nh)) {
    ROS_ERROR_STREAM("[FcsProcessor] Failed to load speed control parameters");
    return;
  }
  
  _has_valid_traj = false;
  _follow_traj = false;
  _traj_completed = false;

  _wp_path = PathXYZVPsi(); 
  _speed_cmd = VelAccPsi();

  _traj_start_time = 0.0;
  _waypoint_index = -1.0;
  
}

void FcsProcessor::trajectoryUpdate(void) {
  if (_fcs_interface->hasPath()) {
  
    setNewTrajectory(_fcs_interface->getPath());
    
    _controller_state.Reset();
  }
}
  
void FcsProcessor::generateSpeedControlCommand(void) {

  bool valid_cmd = false;
  nav_msgs::Odometry curr_pose = _fcs_interface->getOdometry();

  //Update current speed control command
  nav_msgs::Odometry lookahead_pose;
  _speed_cmd = _controller->ComputeControl(_period, curr_pose, _wp_path, _controller_state, lookahead_pose, valid_cmd);
  
  if (!valid_cmd) {
    ROS_INFO_STREAM("Not valid cmd");
    return;
  }
      
  //ROS_ERROR_STREAM_THROTTLE(2.0, "[FcsProcessor] CurrPose: x=" << curr_pose.pose.pose.position.x << " / y=" << curr_pose.pose.pose.position.y <<
  //                               " - LookaheadPose: x=" << lookahead_pose.pose.pose.position.x << " / y=" << lookahead_pose.pose.pose.position.y);

  //ROS_ERROR_STREAM_THROTTLE(1.0, "[FcsProcessor] Command:  vx=" << _speed_cmd.velocity[0] << " / vy=" << _speed_cmd.velocity[1] << " / vz=" << _speed_cmd.velocity[2] << " / heading=" << _speed_cmd.heading);
  //ROS_ERROR_STREAM_THROTTLE(1.0, "Odometry: vx=" << curr_pose.twist.twist.linear.x << " / vy=" << curr_pose.twist.twist.linear.y << " / vz=" << curr_pose.twist.twist.linear.z);

  //Update current spline command type
  //Update current command type - uses closest waypoint index to find time information
  //With time information it is possible to recover command type from the spline 
  unsigned int index = (int)ceil(_controller_state.closest_idx);  
  int pathSize = _wp_path.path.size();
  ROS_INFO_STREAM_THROTTLE(2.0, "[FcsProcessor] Path Index=" << index << " [" << _wp_path.path.size() << "] / Time=" << _wp_path.path[index].time);
  
  //Check if reached end of trajectory command
  index = (unsigned int)round(_controller_state.closest_idx);  
  if ((index+1) >= _wp_path.path.size()) {
    _traj_completed = true;
    ROS_INFO_STREAM_THROTTLE(1.0, "[FcsProcessor] Trajectory completed");
  }
}


bool FcsProcessor::isInsideCaptureRadius(void) {
    
    nav_msgs::Odometry curr_pose = _copter_interface->getOdometry();
    double dx = curr_pose.pose.pose.position.x - _path_end_point[0];
    double dy = curr_pose.pose.pose.position.y - _path_end_point[1];
    double dist = sqrt(dx*dx + dy*dy);
    if (dist < _controller_parameters.land_capture_radius) {
        return true;
    }
    return false;
}


//==============================================================================
// Trajectory support methods

void FcsProcessor::setNewTrajectory(ca_nav_msgs::PathXYZVPsi path) {
    ROS_INFO_STREAM("[FcsProcessor] Got new trajectory command.");

    ros::Time now = ros::Time::now();

    _waypoint_index = 0.0;
    _traj_start_time = now.toSec();
 

    _wp_path = PathXYZVPsi();
    
/*    
    //If trajectory is empty, then create an hover point for the copter
    if (duration == 0) {
      nav_msgs::Odometry dji_odometry_msg = _copter_interface->getOdometry();
      
      XYZVPsi hover_point;
      hover_point.position[0] = dji_odometry_msg.pose.pose.position.x;
      hover_point.position[1] = dji_odometry_msg.pose.pose.position.y;
      hover_point.position[2] = dji_odometry_msg.pose.pose.position.z;

      OdometryEuler curr_pose_eig;
      curr_pose_eig.SetMsg(dji_odometry_msg);
      hover_point.heading = curr_pose_eig.orientation.z();

      _wp_path.path.push_back(hover_point);
      
      _has_valid_traj = false;
      _traj_completed = false;
      
      return;      
    }
*/
    for (auto wp : path.waypoints) {
        XYZVPsi xyzvpsi;
        xyzvpsi.time        = 0.0;
        xyzvpsi.type        = 0;
        xyzvpsi.position[0] = wp.position.x;
        xyzvpsi.position[1] = wp.position.y;
        xyzvpsi.position[2] = wp.position.z;
        xyzvpsi.heading     = wp.heading;
        xyzvpsi.vel         = wp.vel;
        _wp_path.path.push_back(xyzvpsi);        
    }
        
    _has_valid_traj = true;
    _traj_completed = false;

    int size = _wp_path.path.size();
    if (size > 1) {
        XYZVPsi lastPnt = _wp_path.path[size-1];
        _path_end_point = lastPnt.position;
    }
    
    return;
}


float FcsProcessor::getCurrentTrajectoryTime(void) {
    unsigned int index = (unsigned int)floor(_controller_state.closest_idx);  
    unsigned int pathSize = _wp_path.path.size();
    if (index < pathSize) {
        float curr_time = _wp_path.path[index].time;
        //Interpolate time if possible
        float diff = _controller_state.closest_idx - index;
        if ((diff > 0.0) && (index < (pathSize-1))) {
            curr_time += (_wp_path.path[index+1].time - _wp_path.path[index].time) * diff;
        }
        return curr_time;
    }
    return 0.0;
}


