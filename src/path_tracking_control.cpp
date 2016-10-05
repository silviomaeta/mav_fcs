/*
 * path_tracking_control.cpp
 * Author: Carnegie Mellon University / Sanjiban Chowdury
 */

#include <vector>

#include "mav_fcs/path_tracking_control.h"


namespace mavfcs {


double Limit ( double limit, double value ) {
    if ( value>limit ) {
        return limit;
    } else {
        if ( value<-limit )
            return -limit;
        else
            return value;
    }
}


Eigen::Vector3d VectorToLineSegment(const Eigen::Vector3d &a,const Eigen::Vector3d &b, const Eigen::Vector3d &c, double &rv) {
  Eigen::Vector3d AB(b - a);
  Eigen::Vector3d AC(c - a);
  double ABl2 = AB.dot(AB) + std::numeric_limits<double>::epsilon();
  double r = AC.dot(AB) / ABl2;
  Eigen::Vector3d result;
  if(r<0) {
    result = a - c;
    // Close to  A
  } else if(r>1) {
    // Closer to B
    result = b - c;
  } else {
    Eigen::Vector3d mix((1.0-r) * a + r * b);
    result = mix - c;
  }
  rv = r;
  return result;
}



VelAccPsi PathTrackingControl::ComputeControl(double dt, const nav_msgs::Odometry &curr_pose,  const PathXYZVPsi &path_msg, PathTrackingControlState &control_state, nav_msgs::Odometry &look_ahead_pose, bool &valid_cmd) {

  // Lets convert the odom to a euler form
  OdometryEuler curr_state;
  curr_state.SetMsg(curr_pose);
  //ROS_ERROR_STREAM_THROTTLE(1.0, "*** current state - pos: " << curr_state.position[0] << " / " << curr_state.position[1] << " / " << curr_state.position[2]); 
  //ROS_ERROR_STREAM_THROTTLE(1.0, "*** current state - ori: " << curr_state.orientation[0] << " / " << curr_state.orientation[1] << " / " << curr_state.orientation[2]); 
  //ROS_ERROR_STREAM_THROTTLE(1.0, "*** current state - lin: " << curr_state.linear_velocity[0] << " / " << curr_state.linear_velocity[1] << " / " << curr_state.linear_velocity[2]); 
  //ROS_ERROR_STREAM_THROTTLE(1.0, "*** current state - ang: " << curr_state.angular_velocity[0] << " / " << curr_state.angular_velocity[1] << " / " << curr_state.angular_velocity[2]); 

  // Lets convert the path to eigen class
  PathXYZVPsi path;
  path = path_msg;

  // Create empty control
  VelAccPsi command;
  command.velocity = Eigen::Vector3d::Zero();
  command.acceleration = Eigen::Vector3d::Zero();
  command.heading = curr_state.orientation.z();
  command.heading_rate = 0.0;

  std::vector<double> lo;

  //Check precondition of inputs:
  if(!std::isfinite(dt) || !IsFinite(curr_state) || !IsFinite(path)) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[PathTrackControl] Received invalid inputs stopping.");
    if (std::isfinite(dt)) ROS_ERROR_STREAM("A");
    if (IsFinite(curr_state)) ROS_ERROR_STREAM("B");
    if (IsFinite(path)) ROS_ERROR_STREAM("C");
    look_ahead_pose = curr_pose;
    valid_cmd = false;
    return command;
  }
  else
  {
  }

  //Preconditions passed can proceed safely
  if(path.path.size() < 1) {
    ROS_INFO_STREAM_THROTTLE(5.0, "[PathTrackControl] Path does not contain any waypoints, no command issued");
    look_ahead_pose = curr_pose;
    valid_cmd = true;
    return command;
  }

  bool nearing_end, sharp_corner;
  XYZVPsi closest_state = ProjectOnPath(curr_state, path, control_state.closest_idx);
  //ROS_ERROR_STREAM_THROTTLE(1.0, "*** closest state: " << closest_state.position[0] << " / " << closest_state.position[1] << " / " << closest_state.position[2]); 

  //This is the state we will control to. This looks ahead based on the current speed to account for control reaction delays.
  double speed = curr_state.linear_velocity.norm();

  //lookahead distance - minimal value is 1.0 meter (@TODO: make it configurable?)
  double lookaheadDist = std::max(1.0, pr.look_ahead_time * speed);

  std::pair<XYZVPsi, Eigen::Vector3d> pursuit_state_pair = LookAheadMaxAngle(path, control_state.closest_idx, lookaheadDist, pr.look_ahead_angle, nearing_end );
  XYZVPsi pursuit_state = pursuit_state_pair.first;
  //ROS_ERROR_STREAM_THROTTLE(1.0, "*** pursuit state first: " << pursuit_state.position[0] << " / " << pursuit_state.position[1] << " / " << pursuit_state.position[2]);
  //ROS_ERROR_STREAM_THROTTLE(1.0, "*** pursuit state second: " << pursuit_state_pair.second[0] << " / " << pursuit_state_pair.second[1] << " / " << pursuit_state_pair.second[2]);

  //Next we also look up if we need to slow down based on our maximum acceleration.
  double stoppingDistance =  StoppingDistance(pr.deccel_max, pr.reaction_time, speed);

  double distanceToEnd = DistanceToEnd(path, control_state.closest_idx, 5.0 + stoppingDistance, pr.look_ahead_angle, sharp_corner);//Added an offset to prevent problems at low speed

  //max speed - configurable parameter
  double totalSpeed = std::max(pr.max_speed, pursuit_state.vel);

  if (nearing_end) {
    double maxDesiredSpeed = std::min(totalSpeed, StoppingSpeed(pr.deccel_max,pr.reaction_time,distanceToEnd));

    if(maxDesiredSpeed < 1.0 || sharp_corner) {
      double az = closest_state.position[2];
      closest_state = pursuit_state;
      closest_state.position[2] = az;
    }
    pursuit_state.vel = maxDesiredSpeed;
  }

  Eigen::Vector3d desired_velocity = pursuit_state.vel * pursuit_state_pair.second;
  //ROS_ERROR_STREAM_THROTTLE(1.0, "*** desired_velocity: " << desired_velocity[0] << " / " << desired_velocity[1] << " / " << desired_velocity[2]); 
  Eigen::Vector3d curr_to_closest = closest_state.position - curr_state.position;
  //ROS_ERROR_STREAM_THROTTLE(1.0, "*** curr_to_closest: " << curr_to_closest[0] << " / " << curr_to_closest[1] << " / " << curr_to_closest[2]); 

  if (curr_to_closest.norm() < 0.1) {
    curr_to_closest = pursuit_state.position - curr_state.position;
    //ROS_ERROR_STREAM_THROTTLE(1.0, "*** curr_to_closest: " << curr_to_closest[0] << " / " << curr_to_closest[1] << " / " << curr_to_closest[2]); 
  }

  double zError = curr_to_closest[2];
  if (zError < 0.05) {
    zError = pursuit_state.position[2] - curr_state.position[2];
  }

  //Separate x,y from z control
  if(curr_to_closest.norm() > pr.tracking_threshold) {
    ROS_WARN_STREAM_THROTTLE(5.0, "[PathTrackControl] Greater than" << pr.tracking_threshold << "meters from path, no command issued");
    look_ahead_pose = curr_pose;
    valid_cmd = true;
    return command;
  }

  curr_to_closest[2] = 0;
  if(desired_velocity.norm() > pr.max_speed) {
    ROS_WARN_STREAM_THROTTLE(5.0, "[PathTrackControl] Commanded path exceeds maximum allowed speed"<<desired_velocity.norm()<<" > "<<pr.max_speed);
    desired_velocity.normalize();
    desired_velocity *= pr.max_speed;
  }


  Eigen::Vector3d path_tangent = desired_velocity;
  if (path_tangent.norm())
    path_tangent.normalize();
  Eigen::Vector3d curr_to_path = curr_to_closest - path_tangent * (path_tangent.dot(curr_to_closest));
  Eigen::Vector3d path_normal = curr_to_path;
  if (path_normal.norm() > 0)
    path_normal.normalize();
  double cross_track_error = curr_to_closest.dot(path_normal);
  double cross_track_error_d = cross_track_error - control_state.prev_cross_track_error;


  control_state.cross_track_integrator += cross_track_error*dt;
  control_state.cross_track_integrator  = Limit(pr.cross_track_IMax, control_state.cross_track_integrator);

  double u_cross_track = pr.cross_track_P * cross_track_error +
                         pr.cross_track_I * control_state.cross_track_integrator +
                         pr.cross_track_D * cross_track_error_d/dt;

  control_state.prev_cross_track_error = cross_track_error;

  Eigen::Vector3d cmdVel = desired_velocity +  u_cross_track * path_normal;

  //Do separate terms for the z control:
  double z_trackerror =   zError - control_state.prev_z_track_error;
  control_state.prev_z_track_error = zError;

  double ztrack = pr.cross_track_PZ * zError + pr.cross_track_DZ * z_trackerror/dt;
  cmdVel[2] = desired_velocity[2] + ztrack;
  if(cmdVel.norm() > totalSpeed)
    cmdVel *= (totalSpeed/cmdVel.norm());
 
  //Set command velocity and heading   
  command.velocity = cmdVel;
  command.heading = pursuit_state.heading;

  //Check output invariants:
  bool command_is_finite = IsFinite(command);
  if(!command_is_finite) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[PathTrackControl] Calculated invalid command. Going to velocity hold.");
    command.velocity = Eigen::Vector3d::Zero();
    command.heading = curr_state.orientation[2];
    look_ahead_pose = curr_pose;
    valid_cmd = true;
    return command;
  }

  look_ahead_pose.pose.pose.position = ca::point_cast<geometry_msgs::Point>(pursuit_state.position);
  valid_cmd = true;
  return command;
}

bool PathTrackingControl::IsFinite(const Eigen::Vector3d &v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
};

bool PathTrackingControl::IsFinite(const OdometryEuler &o) {
  return IsFinite(o.position) && IsFinite(o.orientation) && IsFinite(o.linear_velocity) && IsFinite(o.angular_velocity);
}

bool PathTrackingControl::IsFinite(const PathXYZVPsi &p) {
  //for (auto it : p.path)
  for (std::vector<XYZVPsi>::const_iterator it = p.path.begin() ; it != p.path.end(); ++it)
    if (!(IsFinite(it->position) && std::isfinite(it->vel)))
      return false;
  return true;
}

bool PathTrackingControl::IsFinite(const VelAccPsi &p) {
  return IsFinite(p.acceleration) && IsFinite(p.velocity) && std::isfinite(p.heading) && std::isfinite(p.heading_rate);
}

XYZVPsi PathTrackingControl::Interpolate(const XYZVPsi &A, const XYZVPsi& B, double alpha) {
  XYZVPsi res;
  res.position =  (1.0 - alpha)*A.position + alpha*B.position ;
  res.vel = (1.0 - alpha)*A.vel + alpha*B.vel;
  res.heading = (1.0 - alpha)*A.heading + alpha*B.heading;
  return res;
}

XYZVPsi PathTrackingControl::SamplePath(const PathXYZVPsi &path, double index) {
  XYZVPsi sc;
  int prev = std::floor(index);
  int next = std::ceil(index);

  if(next > (int) path.path.size() - 1)
    return sc;

  XYZVPsi a = path.path[prev];
  XYZVPsi b = path.path[next];
  double prog = index - (double)prev;

  sc = Interpolate(a, b, prog);
  return sc;
}

XYZVPsi PathTrackingControl::ProjectOnPath(const OdometryEuler &state, const PathXYZVPsi &path, double &closest_idx) {
  XYZVPsi new_state;
  //Eigen::Vector3d min_vec = Eigen::Vector3d::Zero();
  double min_l = std::numeric_limits<double>::max();
  unsigned ts = path.path.size();
  if (ts == 0)
  {
    throw std::runtime_error("path size is 0");
  }
  double min_idx = 0.0;
  unsigned int start_i=0;
  min_idx = closest_idx;
  start_i = std::floor(min_idx);
  start_i = std::min(start_i,ts-1);
  new_state = path.path[ts-1];
  if(ts == 1) {
    min_idx = 0;
    new_state = path.path[0];
  } 
  else if(ts>1) {
    for(unsigned int it = start_i; it < (ts-1); it++) {
      Eigen::Vector3d vec;
      //bool onEnd;
      double rv;
      XYZVPsi A =  path.path[ it     ];
      XYZVPsi B =  path.path[ it + 1 ];
      vec = VectorToLineSegment(A.position, B.position, state.position, rv);
      double len = vec.norm();
      //Take the closest point
      if (len < min_l) {
        min_l = len;

        if(rv < 0.0) rv = 0.0;
        if(rv > 1.0) rv = 1.0;

        min_idx = it + rv;
        new_state = Interpolate(A, B, rv);
      }
      else if (min_l != std::numeric_limits<double>::max()) {
        //If it starts to get points that are not closer, then stops searching
        //Use the first closest point - don't scan the whole trajectory
        if(len >= min_l)
          break;
      }
    }
  }

  //ROS_INFO_STREAM("[PathTrackControl] min_idx=" << min_idx << " / closest_idx=" << closest_idx << " / total size=" << ts);
  closest_idx = min_idx;
  return new_state;
}


std::pair<XYZVPsi, Eigen::Vector3d> PathTrackingControl::LookAheadMaxAngle(const PathXYZVPsi &path, double idx, double dist, double max_angle_rad, bool &at_end ) {

  int next_idx = std::ceil(idx);
  if ((next_idx > idx) && (fabs(next_idx - idx) < 0.05))
    next_idx++;

  int pathSize = path.path.size();
  
  if(next_idx > (pathSize-1)) {
    at_end = true;
    Eigen::Vector3d vec = path.path[std::max(0,pathSize-1)].position - path.path[std::max(0,pathSize-2)].position;
    if (vec.norm() > 0)
      vec.normalize();
    return std::make_pair(path.path.back(), vec);
  }
  else
  {
  }

  //ROS_INFO_STREAM_THROTTLE(1.0, ">>> dist=" << dist);

  //If next_idx is too close to current idx, then increase index value
  XYZVPsi next = path.path[next_idx];
  XYZVPsi curr = SamplePath(path, idx);
  Eigen::Vector3d delta = next.position - curr.position;
  while ((delta.norm() < 0.1) && (next_idx < (pathSize-1))) {
    next_idx++;
    next = path.path[next_idx];
    delta = next.position - curr.position;
  }

  //ROS_INFO_STREAM_THROTTLE(1.0, ">>> idx=" << idx << " / next_idx=" << next_idx << " / delta=" << delta.norm() << " / total size=" << pathSize);
  
  XYZVPsi retState = curr;
  Eigen::Vector3d vec = Eigen::Vector3d::Zero();

  double dist_remaining = dist;
  bool bAtEnd = false;

  Eigen::Vector3d prevVector = next.position - curr.position;
  if (prevVector.norm() > 0)
    prevVector.normalize();

  while(!bAtEnd && dist_remaining > 0) {
    Eigen::Vector3d currVector = next.position - curr.position;
    double dist_on_seg = currVector.norm();
    if (currVector.norm() > 0)
      currVector.normalize();

    double currValue = fabs(acos(std::min(1.0,currVector.dot(prevVector))));
    prevVector = currVector;

    if(currValue >= max_angle_rad) {
      retState = path.path[std::max(0,next_idx-1)];
      vec = path.path[std::max(0,next_idx-1)].position - path.path[std::max(0,next_idx-2)].position;
      //ROS_INFO_STREAM_THROTTLE(1.0, ">>> angle: currValue=" << currValue << " / max_angle_rad=" << max_angle_rad);    
      break;
    }

    if(dist_on_seg > dist_remaining) {
      double prog = dist_remaining / dist_on_seg;
      retState = Interpolate(curr, next, prog);
      vec = next.position - curr.position;
      dist_remaining = 0;
    } else {
      if(next_idx == (int)path.path.size()-1) {
        bAtEnd = true;
        retState = path.path.back();
        vec = path.path[std::max(0,(int)path.path.size()-1)].position - path.path[std::max(0,(int)path.path.size()-2)].position;
        //ROS_INFO_STREAM_THROTTLE(1.0, ">>> at the end");    
      } else {
        curr = next;
        next_idx++;
        dist_remaining -= dist_on_seg;
        next = path.path[next_idx];
      }
    }
    //ROS_INFO_STREAM_THROTTLE(1.0, ">>> dist_remanining=" << dist_remaining);
  }

  at_end = bAtEnd;
  if (vec.norm() > 0)
    vec.normalize();

  return std::make_pair(retState, vec);
}

double PathTrackingControl::StoppingDistance(double acceleration,double reactionTime,double speed) {
  return speed * reactionTime + speed * speed/(2.0 * acceleration);
}

double PathTrackingControl::StoppingSpeed(double acceleration,double reactionTime, double distance) {
    double sf1 = 2 * acceleration * distance;
    double sf2 = acceleration * acceleration * reactionTime;
    return std::max(0.0,-acceleration * reactionTime + std::sqrt(sf1 + sf2));
}


double PathTrackingControl::DistanceToEnd(const PathXYZVPsi &path, double idx, double maxLookAhead, double maxAngleRad, bool &sharpCorner) {
  sharpCorner = false;
  int next_idx = std::ceil(idx);
  if(next_idx == idx)
    next_idx++;
  next_idx = std::min(next_idx, (int)path.path.size()-1);
  XYZVPsi next = path.path[next_idx];
  XYZVPsi curr = SamplePath(path, idx);

  double dist_remaining = maxLookAhead;
  bool bAtEnd = false;

  Eigen::Vector3d prevVector = next.position - curr.position;
  if (prevVector.norm() > 0)
    prevVector.normalize();
  double maxLookAheadM = maxLookAhead;

  while(!bAtEnd && dist_remaining > 0) {
    Eigen::Vector3d currVector = next.position - curr.position;
    double dist_on_seg = currVector.norm();
    if (currVector.norm() > 0)
      currVector.normalize();
    double currValue = fabs(acos(std::min(1.0,currVector.dot(prevVector))));
    prevVector = currVector;
    if(currValue > maxAngleRad) {
      sharpCorner = true;
      bAtEnd = true;
    }
    if(dist_on_seg > dist_remaining){
      dist_remaining = 0;
    }else {
      dist_remaining -= dist_on_seg;
      if(next_idx == (int)path.path.size()-1 ){
        bAtEnd = true;
      }else{
        curr = next;
        next_idx++;
        next = path.path[next_idx];
      }
    }
  }
  double leng = maxLookAheadM - dist_remaining;
  return leng;
}


}


