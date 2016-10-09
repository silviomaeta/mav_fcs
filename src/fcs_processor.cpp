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
  delete _state_machine;
  delete _copter_interface;
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

  if (!_copter_interface->isEngaged()) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "FcsProcessor::update() - copter is not engaged.");
    return;
  }

  //Update if new trajectory is available
  trajectoryUpdate();
 
  //Update speed control command based on current state
  if (_has_valid_traj && (!_traj_completed)) {
    //generateSpeedControlCommand();
  }

  //Update odometry that will be published by FCS Interface
  _fcs_interface->setFcsOdometry(_copter_interface->getOdometry());

  //Update FCS status
  _fcs_interface->setFcsStatus( getFcsStatus() );

  
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
      //ROS_INFO_STREAM_THROTTLE(1.0, "has_valid_traj=" << _has_valid_traj);
      //ROS_INFO_STREAM_THROTTLE(1.0, "follow_traj=" << _follow_traj);
      //ROS_INFO_STREAM_THROTTLE(1.0, "traj_completed=" << _traj_completed);

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

        double roll, pitch;
        double vx, vy, vz;
        double yawrate;
        
        _inspect_ctrl->get_cmd(roll, pitch, vz, yawrate);
        //ROS_INFO_STREAM_THROTTLE(0.2, "[FcsProcessor] Cmd: roll=" << roll << " / pitch=" << pitch << " / vz=" << vz << " / yawrate=" << yawrate);
        
        vx = roll;
        vy = pitch;
        
        _copter_interface->setVelocityCommand(vx, vy, vz, yawrate);
        
        updateWaypointIndex();
        
        if (isLastWaypoint()) {
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
  
  _inspect_ctrl = new DjiInspectCtrl(traj_controller_nh);

  _has_valid_traj = false;
  _follow_traj = false;
  _traj_completed = false;

  _waypoints.clear(); 

  _target_capture_radius = 0.50;
  
  _has_pause_point = false;
  _pause_interval = 8.0;
  _pause_start_time = -1.0;
  
  _max_distance_waypoints = 1.5;
  
}

void FcsProcessor::trajectoryUpdate(void) {
  if (_fcs_interface->hasPath()) {
  
    setNewTrajectory(_fcs_interface->getPath());
    
  }
}


void FcsProcessor::updateWaypointIndex(void) {
    if (_waypoint_index >= _waypoints.size()) {
        return;
    }
    
    //If current position is close to the target then select next waypoint
    geometry_msgs::PoseStamped target = _waypoints[_waypoint_index];
    nav_msgs::Odometry curr_pose = _inspect_ctrl->get_odom();
    double dx = curr_pose.pose.pose.position.x - target.pose.position.x;
    double dy = curr_pose.pose.pose.position.y - target.pose.position.y;
    double dz = curr_pose.pose.pose.position.z - target.pose.position.z;
    double dist = sqrt(dx*dx + dy*dy + dz*dz);
    ROS_INFO_STREAM_THROTTLE(1.0, "Distance to target[" << _waypoint_index << "] : " << dist << " (dx=" << dx << " / dy=" << dy << ")");
    if (dist < _target_capture_radius) {
        
        //Check if it is a point it needs to make a pause (wait for some time before updating)
        bool update_index = true;
        if (_has_pause_point) {
            if ((_pause_point.pose.position.x == target.pose.position.x) && 
                (_pause_point.pose.position.y == target.pose.position.y) && 
                (_pause_point.pose.position.z == target.pose.position.z)) {
                double now = ros::Time::now().toSec();
                if (_pause_start_time < 0.0) {
                    _pause_start_time = now;
                    update_index = false;
                }
                else {
                    double diff = now - _pause_start_time;
                    ROS_INFO_STREAM_THROTTLE(0.5, "Waiting (pause interval) : " << diff);
                    if (diff < _pause_interval) {
                        update_index = false;
                    } 
                }
            }
        }
        
        //Update waypoint index
        if (update_index) {
            _waypoint_index++;
            if (_waypoint_index < _waypoints.size()) {
                _inspect_ctrl->set_target(_waypoints[_waypoint_index]);
            }
        }
    }
    
}

bool FcsProcessor::isLastWaypoint(void) {
    if (_waypoint_index >= _waypoints.size()) {
        return true;
    }
    return false;
}


//==============================================================================
// Trajectory support methods

void FcsProcessor::setNewTrajectory(ca_nav_msgs::PathXYZVPsi path) {
    ROS_INFO_STREAM("[FcsProcessor] Got new trajectory command.");

    _waypoints.clear();
    
    _has_pause_point = false;
    
    bool first = true;
    geometry_msgs::PoseStamped prev;
    for (auto wp : path.waypoints) {
        geometry_msgs::PoseStamped ps;

        tf::Quaternion aux_quat;
        aux_quat.setRPY(0.0, 0.0, wp.heading);
                
        ps.pose.position.x = wp.position.x;
        ps.pose.position.y = wp.position.y;
        ps.pose.position.z = wp.position.z;
        
        ps.pose.orientation.x = aux_quat[0];
        ps.pose.orientation.y = aux_quat[1];
        ps.pose.orientation.z = aux_quat[2];
        ps.pose.orientation.w = aux_quat[3];

        //Set point that has a pause interval (target point)
        if (wp.vel < 0.0) {
            _has_pause_point = true;
            _pause_point = ps;
            _pause_start_time = -1.0;
        }

        //Added intermediate points if waypoints are far from each other
        if (!first) {
            double dx = ps.pose.position.x - prev.pose.position.x;
            double dy = ps.pose.position.y - prev.pose.position.y;
            double dz = ps.pose.position.z - prev.pose.position.z;
            double dist = sqrt(dx*dx + dy*dy + dz*dz);
            if (dist > _max_distance_waypoints) {
                double num_pnts = ceil(dist/_max_distance_waypoints);
                for (int i=1; i<(int)num_pnts; i++) {
                    geometry_msgs::PoseStamped ips;
                    
                    ips.pose.position.x = prev.pose.position.x + i*(dx/num_pnts);
                    ips.pose.position.y = prev.pose.position.y + i*(dy/num_pnts);
                    ips.pose.position.z = prev.pose.position.z + i*(dz/num_pnts);
                    
                    ips.pose.orientation = ps.pose.orientation;
                    
                    //ROS_INFO_STREAM(ips.pose.position);
                    
                    _waypoints.push_back(ips);
                } 
            }
        }
     
        _waypoints.push_back(ps);
        prev = ps;
        first = false;
    }
        
    _has_valid_traj = true;
    _traj_completed = false;

    //Set initial target
    _waypoint_index = 0;
    if (_waypoints.size() > 0) {
        _inspect_ctrl->set_target( _waypoints[_waypoint_index] );
    }
    
    return;
}


