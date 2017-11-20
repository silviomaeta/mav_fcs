/*
* Copyright (c) 2016 Carnegie Mellon University, Silvio Maeta <smaeta@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#ifndef _MAV_FCS_PATH_TRACKING_CONTROL_H_
#define _MAV_FCS_PATH_TRACKING_CONTROL_H_

#include <ros/ros.h>
#include <tf/tf.h>

#include <Eigen/Core>

#include <nav_msgs/Odometry.h>

#include <geom_cast/geom_cast.hpp>

namespace mavfcs {

struct OdometryEuler {
  double time;
  Eigen::Vector3d position;
  Eigen::Vector3d orientation;
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;

  nav_msgs::Odometry GetMsg() {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time(time);
    odom.pose.pose.position = ca::point_cast<geometry_msgs::Point>(position);

    tf::Quaternion quat;
    quat.setRPY(orientation.x(), orientation.y(), orientation.z());
    odom.pose.pose.orientation = ca::rot_cast<geometry_msgs::Quaternion>(quat);

    odom.twist.twist.linear = ca::point_cast<geometry_msgs::Vector3>(linear_velocity);
    odom.twist.twist.angular = ca::point_cast<geometry_msgs::Vector3>(angular_velocity);
    return odom;
  }

  void SetMsg(const nav_msgs::Odometry &odom) {
    time = odom.header.stamp.toSec();
    position = ca::point_cast<Eigen::Vector3d>(odom.pose.pose.position);

    tf::Quaternion quat = ca::rot_cast<tf::Quaternion>(odom.pose.pose.orientation);
    tf::Matrix3x3(quat).getRPY(orientation[0], orientation[1], orientation[2]);

    linear_velocity = ca::point_cast<Eigen::Vector3d>(odom.twist.twist.linear);
    angular_velocity = ca::point_cast<Eigen::Vector3d>(odom.twist.twist.angular);
  }

};

struct VelAccPsi {
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  double heading;
  double heading_rate;

  bool IsFinite() const {
    return std::isfinite(velocity.x()) && std::isfinite(velocity.y()) && std::isfinite(velocity.z()) &&
        std::isfinite(acceleration.x()) && std::isfinite(acceleration.y()) && std::isfinite(acceleration.z()) &&
        std::isfinite(heading) && std::isfinite(heading_rate);
  }
};

struct XYZVPsi {
  double time;
  Eigen::Vector3d position;
  double vel;
  double heading;
  int type;
};

struct PathXYZVPsi {
  std::vector<XYZVPsi> path;
};


class PathTrackingControlParameters {
 public:
  double cross_track_P;
  double cross_track_I;
  double cross_track_D;
  double cross_track_PZ;
  double cross_track_DZ;
  double loop_rate;
  double max_speed;
  double look_ahead_time;
  double look_ahead_angle;
  double cross_track_IMax;
  double tracking_threshold;
  double deccel_max;
  double reaction_time;
  double land_capture_radius;
  PathTrackingControlParameters():
    cross_track_P(0.0),
    cross_track_I(0.0),
    cross_track_D(5.0),
    cross_track_PZ(0.0),
    cross_track_DZ(0.0),
    loop_rate(5.0),
    max_speed(5.0),
    look_ahead_time(0.5),
    look_ahead_angle(1.0),
    cross_track_IMax(5.0),
    tracking_threshold(10.),
    deccel_max(3.0),
    reaction_time(0.5),
    land_capture_radius(1.0) {
  }

  bool LoadParameters(ros::NodeHandle &n) {
    bool found = true;

    found = found && n.getParam("cross_track_P", cross_track_P);
    found = found && n.getParam("cross_track_I", cross_track_I);
    found = found && n.getParam("cross_track_D", cross_track_D);
    found = found && n.getParam("cross_track_IMax", cross_track_IMax);

    found = found && n.getParam("cross_track_PZ", cross_track_PZ);
    found = found && n.getParam("cross_track_DZ", cross_track_DZ);

    found = found && n.getParam("loop_rate", loop_rate);
    found = found && n.getParam("max_speed", max_speed);
    found = found && n.getParam("look_ahead_time", look_ahead_time);
    found = found && n.getParam("look_ahead_angle", look_ahead_angle);
    found = found && n.getParam("tracking_threshold", tracking_threshold);
    found = found && n.getParam("deccel_max", deccel_max);
    found = found && n.getParam("reaction_time", reaction_time);

    found = found && n.getParam("land_capture_radius", land_capture_radius);
    return found;
  }
};


class PathTrackingControl {
 public:
  struct PathTrackingControlState {
    double along_track_integrator;
    double cross_track_integrator;
    double prev_along_track_error;
    double prev_cross_track_error;
    double prev_z_track_error;
    double closest_idx;
    PathTrackingControlState():
      along_track_integrator(0.0),
      cross_track_integrator(0.0),
      prev_along_track_error(0.0),
      prev_cross_track_error(0.0),
      prev_z_track_error(0.0),
      closest_idx(0.0)
    {}
    void Reset(void) {
      along_track_integrator = 0.0;
      cross_track_integrator = 0.0;
      prev_along_track_error = 0.0;
      prev_cross_track_error = 0.0;
      prev_z_track_error = 0.0;
      closest_idx = 0.0;
    }
  };

  PathTrackingControl(const PathTrackingControlParameters &pr)
  : pr(pr) {}

  VelAccPsi ComputeControl(double dt, const nav_msgs::Odometry &curr_pose,  const PathXYZVPsi &path, PathTrackingControlState &control_state, nav_msgs::Odometry &look_ahead_pose, bool &valid_cmd);

 protected:

  bool IsFinite(const Eigen::Vector3d &v);

  bool IsFinite(const OdometryEuler &o);

  bool IsFinite(const PathXYZVPsi &p);

  bool IsFinite(const VelAccPsi &p);

  XYZVPsi Interpolate(const XYZVPsi &A, const XYZVPsi& B, double alpha);

  XYZVPsi SamplePath(const PathXYZVPsi &path, double index);

  XYZVPsi ProjectOnPath(const OdometryEuler &state, const PathXYZVPsi &path, double &closest_idx);

  std::pair<XYZVPsi, Eigen::Vector3d> LookAheadMaxAngle(const PathXYZVPsi &path, double idx, double dist, double max_angle_rad, bool &at_end );

  double StoppingDistance(double acceleration,double reactionTime,double speed);

  double StoppingSpeed(double acceleration,double reactionTime, double distance);

  double DistanceToEnd(const PathXYZVPsi &path, double idx, double maxLookAhead, double maxAngleRad, bool &sharpCorner);

  PathTrackingControlParameters pr;

};


}


#endif /* _MAV_FCS_PATH_TRACKING_CONTROL_H_ */
