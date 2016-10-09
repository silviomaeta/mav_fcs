#ifndef _INSPECT_CTRL_H_
#define _INSPECT_CTRL_H_

#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
//#include "dji_inspect_ctrl/pid_msgs.h"
//#include "trajectory_control/Command.h"
//#include "queue"

class DjiInspectCtrl {
public:
    DjiInspectCtrl(ros::NodeHandle &nh);
    ~DjiInspectCtrl() {}

    // Target pose callback function
    //void target_cb(const geometry_msgs::PoseStamped &msg);

    void set_target(geometry_msgs::PoseStamped &target);

    // Current pose/odometry callback function
    void laser_odom_cb(const nav_msgs::Odometry &msg);

    nav_msgs::Odometry get_odom(void) { return _laser_odom; };

    // DJI odometry callback
    //void dji_odom_cb(const nav_msgs::Odometry &msg);

    // Target velocity callback
    //void target_vel_cb(const geometry_msgs::Point &msg);

    // publish cmd
    void publish_cmd();
    void get_cmd(double &roll, double &pitch, double &vz, double &yawrate);

    // Update error
    void update_position_error();
    void update_velocity_error();

    // Update control input
    void update_position_control();
    void update_velocity_control();

    // Update delta t and current time
    void update_time();
    
private:
    ros::Publisher _dji_cmd_pub;
    //ros::Publisher _pid_msg_pub;
    
    ros::Subscriber _laser_odom_sub;
    //ros::Subscriber _dji_odom_sub;
    //ros::Subscriber _target_pose_sub;
    //ros::Subscriber _target_vel_sub;

    nav_msgs::Odometry _laser_odom;
    std::string _laser_odom_topic_name;

    // rotation/position: t:  target in map frame,
    //                    l:  laser  in map frame
    tf::Matrix3x3  _t_rot,  _l_rot;
    tf::Quaternion _t_quat, _l_quat;
    tf::Vector3    _t_pos,  _l_pos;
    tf::Vector3    _t_vel,  _l_vel;

    tf::Vector3 _err_pos, _err_pos_prev, _diff_err_pos, _intg_err_pos;
    tf::Vector3 _err_vel, _err_vel_prev, _diff_err_vel, _intg_err_vel;
    tf::Matrix3x3 _err_rot_body;
    double _err_roll, _err_pitch, _err_yaw;

    // Boundings for control command
    double _MAX_ROLL, _MAX_PITCH, _MAX_Z_RATE, _MAX_YAW_RATE;
    double _MIN_ROLL, _MIN_PITCH, _MIN_Z_RATE, _MIN_YAW_RATE;
    double _MAX_VEL_X, _MAX_VEL_Y, _MAX_VEL_Z;
    double _MAX_VEL_INTG, _MAX_POS_INTG;
    double _INIT_VEL_INTG_X, _INIT_VEL_INTG_Y;
    double _FW_ROLL, _FW_PITCH, _FW_Z_RATE;

    // Gains of control
    double _P_VEL_Y, _P_VEL_X;
    double _D_VEL_Y, _D_VEL_X;
    double _I_VEL_Y, _I_VEL_X;

    double _P_POS_X, _P_POS_Y, _P_POS_Z;
    double _I_POS_X, _I_POS_Y, _I_POS_Z;
    double _D_POS_X, _D_POS_Y, _D_POS_Z;
    double _P_ORI_Z;

    bool _init_pos_err, _init_vel_err, _init_time;
    double _dt, _prev_time, _curr_time;
    double _freq;

    std::vector<tf::Vector3> _vel_vec;
    std::vector<tf::Vector3> _diff_vel_vec;
    std::vector<tf::Vector3> _diff_pos_vec;
    int _vel_vec_idx;
    int _diff_vel_vec_idx, _diff_pos_vec_idx;
    // Compute desired roll, pitch, yaw-velocity, z-velocity
    double _roll, _pitch, _yaw_rate, _z_rate;
    
    //dji_inspect_ctrl::pid_msgs _pid_msg;

    //bool _DEBUG, _USE_DJI_VEL;
    double _YAW_OFFSET;
};


#endif
