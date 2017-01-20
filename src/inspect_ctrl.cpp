#include "mav_fcs/inspect_ctrl.h"

template <typename T> double sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

DjiInspectCtrl::DjiInspectCtrl(ros::NodeHandle &nh) {

    // Command boundings
    nh.param("MAX_ROLL",     _MAX_ROLL,     3.0);
    nh.param("MAX_PITCH",    _MAX_PITCH,    3.0);
    nh.param("MAX_YAW_RATE", _MAX_YAW_RATE, 5.0);
    nh.param("MAX_Z_RATE",   _MAX_Z_RATE,   0.2);
    nh.param("MIN_ROLL",     _MIN_ROLL,     0.5);
    nh.param("MIN_PITCH",    _MIN_PITCH,    0.5);
    nh.param("MIN_YAW_RATE", _MIN_YAW_RATE, 0.5);
    nh.param("MIN_Z_RATE",   _MIN_Z_RATE,   0.1);

    nh.param("MAX_VEL_X",    _MAX_VEL_X,    0.3);
    nh.param("MAX_VEL_Y",    _MAX_VEL_Y,    0.3);
    nh.param("MAX_VEL_Z",    _MAX_VEL_Z,    0.3);
    
    nh.param("MAX_VEL_INTG", _MAX_VEL_INTG, 0.0);
    nh.param("MAX_POS_INTG", _MAX_POS_INTG, 0.0);

    nh.param("FW_ROLL",      _FW_ROLL,      0.1);
    nh.param("FW_PITCH",     _FW_PITCH,     0.1);
    nh.param("FW_Z_RATE",    _FW_Z_RATE,    0.1);

    // Velocity control params
    nh.param("P_VEL_Y",      _P_VEL_Y,      1.0);
    nh.param("P_VEL_X",      _P_VEL_X,      1.0);
    nh.param("D_VEL_Y",      _D_VEL_Y,      0.0);
    nh.param("D_VEL_X",      _D_VEL_X,      0.0);
    nh.param("I_VEL_Y",      _I_VEL_Y,      0.0);
    nh.param("I_VEL_X",      _I_VEL_X,      0.0);

    nh.param("P_ORI_Z",      _P_ORI_Z,      0.5);

    nh.param("P_POS_X",      _P_POS_X,      1.0);
    nh.param("P_POS_Y",      _P_POS_Y,      1.0);
    nh.param("P_POS_Z",      _P_POS_Z,      1.0);
    nh.param("I_POS_X",      _I_POS_X,      1.0);
    nh.param("I_POS_Y",      _I_POS_Y,      1.0);
    nh.param("I_POS_Z",      _I_POS_Z,      1.0);
    nh.param("D_POS_X",      _D_POS_X,      1.0);
    nh.param("D_POS_Y",      _D_POS_Y,      1.0);
    nh.param("D_POS_Z",      _D_POS_Z,      1.0);

    nh.param("INIT_VEL_INTG_X", _INIT_VEL_INTG_X, 0.4);
    nh.param("INIT_VEL_INTG_Y", _INIT_VEL_INTG_Y, 1.4);

    nh.param("YAW_OFFSET", _YAW_OFFSET, 0.0);

    if (!nh.getParam("control_freq", _freq)){
        ROS_WARN("inspect_ctrl: can not get control freq param");
    }
    
    if (!nh.getParam("laser_odom_topic_name", _laser_odom_topic_name)) {
        ROS_WARN("inspect_ctrl: can not get laser odom topic name");
    }
    

    // Command publisher
    _dji_cmd_pub = nh.advertise<geometry_msgs::QuaternionStamped> ("/dji_inspect_ctrl/cmd", 100);
    _pid_msg_pub = nh.advertise<geometry_msgs::PointStamped> ("/dji_inspect_ctrl/pid_msgs", 100);
    _gim_tar_pub = nh.advertise<dji_sdk::Gimbal> ("/dji_inspect_ctrl/gimbal_target", 100);
    _gim_cmd_pub = nh.advertise<dji_sdk::Gimbal> ("/dji_inspect_ctrl/gimbal_command", 100);
    _cur_tar_pub = nh.advertise<visualization_msgs::Marker>("/dji_inspect_ctrl/current_target", 100);
    
    // Subscribers
    _laser_odom_sub = nh.subscribe(_laser_odom_topic_name, 50, &DjiInspectCtrl::laser_odom_cb, this);
    _gimbal_ang_sub = nh.subscribe("/dji_sdk/gimbal", 50, &DjiInspectCtrl::gimbal_ang_cb, this);
    _dji_odom_sub = nh.subscribe("/dji_sdk/odometry", 50, &DjiInspectCtrl::dji_odom_cb, this);
    //_target_pose_sub = nh.subscribe("/dji_inspect_ctrl/target_pose", 50, &DjiInspectCtrl::target_cb, this);

    _init_pos_err = true;
    _init_vel_err = true;
    _init_time = true;

    _t_rot.setIdentity();
    _l_rot.setIdentity();
    _t_pos.setZero();
    _l_pos.setZero();
    _t_vel.setZero();
    _l_vel.setZero();
    _d_rot.setIdentity();

    _err_pos.setZero();
    _err_pos_prev.setZero();
    _diff_err_pos.setZero();
    _intg_err_pos.setZero();

    _err_vel.setZero();
    _err_vel_prev.setZero();
    _diff_err_vel.setZero();
    _intg_err_vel.setZero();

    _vel_vec_idx = 0;
    _diff_vel_vec_idx = 0;
    _diff_pos_vec_idx = 0;

    _g_roll = 0.0;
    _g_pitch = 0.0;
    _g_yaw = 0.0;
    _gt_rot.setIdentity();
    _gt_quat.setRPY(0.0,0.0,0.0);
    _prev_yaw = 0.0;


}

void DjiInspectCtrl::set_target(geometry_msgs::PoseStamped &target) {
    tf::quaternionMsgToTF(target.pose.orientation, _t_quat);
    //_t_quat.setRPY(0.0, -85.0/180.0*M_PI, 0.0);
    _t_rot.setRotation(_t_quat);
    _gt_rot.setRotation(_t_quat);
    _gt_quat = _t_quat;

    double y, p, r;
    _t_rot.getRPY(r,p,y);

    if (fabs(r) > 85.0/180.0*M_PI || fabs(p) > 85.0/180.0*M_PI ) y = 0.0;//_prev_yaw;
    
    _t_quat.setRPY(0.0,0.0,y);
    _t_rot.setRPY(0.0,0.0,y);
    _t_pos.setValue(target.pose.position.x, target.pose.position.y, target.pose.position.z);
    _prev_yaw = y;


    ROS_INFO_STREAM_THROTTLE(1.0, "Target set: x=" << target.pose.position.x << " / y=" << target.pose.position.y << " / z=" << target.pose.position.z << " / yaw=" << y);
}

void DjiInspectCtrl::laser_odom_cb(const nav_msgs::Odometry &msg) {

    _laser_odom = msg;

    tf::quaternionMsgToTF(msg.pose.pose.orientation, _l_quat);
    
    // Assuming there is a offset between lidar frame and dji body frame
    tf::Matrix3x3 l_rot, delta_l_rot;
    tf::Quaternion delta_l_quat;
    l_rot.setRotation(_l_quat);
    delta_l_quat.setRPY(0.0,0.0,_YAW_OFFSET);
    delta_l_rot.setRotation(delta_l_quat);
    _l_rot = l_rot * delta_l_rot;
    
    //_l_rot.setRotation(_l_quat);
    _l_pos.setValue(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z);
    _l_vel.setValue(msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z);
    _l_vel = _l_rot.transpose() * _l_vel;
    
    //ROS_INFO_STREAM_THROTTLE(1.0, "Laser localization callback");
    update_time();
    update_position_error();
    update_position_control();
    update_velocity_error();
    update_velocity_control();
    publish_cmd();
}

void DjiInspectCtrl::gimbal_ang_cb(const dji_sdk::Gimbal &msg) {
    //ROS_INFO_STREAM_THROTTLE(1.0, "Gimbal callback called");
    _g_roll  = double(msg.roll)*M_PI/180.0;
    _g_pitch = double(msg.pitch)*M_PI/180.0;
    _g_yaw   = double(msg.yaw)*M_PI/180.0;
}

void DjiInspectCtrl::dji_odom_cb(const nav_msgs::Odometry &msg) {
    //ROS_INFO_STREAM_THROTTLE(1.0, "DJI odometry callback called");
    tf::quaternionMsgToTF(msg.pose.pose.orientation, _d_quat);
    _d_rot.setRotation(_d_quat);
}

void DjiInspectCtrl::publish_cmd() {

    // Publish command convert back to radius
    geometry_msgs::QuaternionStamped cmd;

    cmd.header.frame_id = "dji";
    cmd.header.stamp = ros::Time::now();
    cmd.quaternion.w = _yaw_rate;
    cmd.quaternion.x = _roll;
    cmd.quaternion.y = _pitch;
    cmd.quaternion.z = _z_rate;

    _dji_cmd_pub.publish(cmd);

    ROS_INFO_ONCE("CMD publish");

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "target";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = _t_pos.x();
    marker.pose.position.y = _t_pos.y();
    marker.pose.position.z = _t_pos.z();
    marker.pose.orientation.x = _gt_quat.x();
    marker.pose.orientation.y = _gt_quat.y();
    marker.pose.orientation.z = _gt_quat.z();
    marker.pose.orientation.w = _gt_quat.w();
    marker.scale.x = 0.4;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    _cur_tar_pub.publish(marker);


}


void DjiInspectCtrl::get_cmd(double &roll, double &pitch, double &vz, double &yawrate) {
    roll    = _roll;
    pitch   = _pitch;
    vz      = _z_rate;
    yawrate =  _yaw_rate;
}

void DjiInspectCtrl::get_gimbal(double &r_rate, double &p_rate, double &y_rate) {

    // Fake a target roll pitch yaw for debugging
    //tf::Quaternion q;
    //q.setRPY(0.0, 85.0/180.0*M_PI, -0.0/180.0*M_PI);
    //_gt_rot.setRotation(q);

    double d_roll, d_pitch, d_yaw;
    _d_rot.getRPY(d_roll, d_pitch, d_yaw);

    // Target roll pitch yaw of gimbal wrt dji world frame
    double t_roll, t_pitch, t_yaw;
    tf::Matrix3x3 t_rot_dw;
    t_rot_dw = _d_rot * _l_rot.inverse() * _gt_rot;
    t_rot_dw.getRPY(t_roll, t_pitch, t_yaw);

    if (fabs(t_pitch)>30.0/180.0*M_PI) t_yaw = d_yaw;

    dji_sdk::Gimbal tar;
    tar.roll  = 0.0;//t_roll*180.0/M_PI;
    tar.pitch = t_pitch*180.0/M_PI;
    tar.yaw   = t_yaw*180.0/M_PI;
    _gim_tar_pub.publish(tar);

    // Check for roll pitch limit
    //t_roll  = t_roll  >  M_PI/4.0 ?  M_PI/4.0 : t_roll;
    //t_roll  = t_roll  < -M_PI/4.0 ? -M_PI/4.0 : t_roll;
    t_pitch = t_pitch >  30.0/180.0*M_PI ?  30.0/180.0*M_PI : t_pitch;
    t_pitch = t_pitch < -89.0/180.0*M_PI ? -89.0/180.0*M_PI : t_pitch;

    // handle singularity
    if (t_yaw - _g_yaw >=  M_PI) t_yaw -= 2.0*M_PI;
    if (t_yaw - _g_yaw <= -M_PI) t_yaw += 2.0*M_PI;

    r_rate = sgn(t_roll  - _g_roll) *0.0;
    p_rate = sgn(t_pitch - _g_pitch)*100.0;
    y_rate = sgn(t_yaw   - _g_yaw)  *100.0;
    
    // Stabilize command
    if (fabs(t_roll  - _g_roll)  < 2.0/180.0*M_PI) r_rate = 0.0;
    if (fabs(t_pitch - _g_pitch) < 2.0/180.0*M_PI) p_rate = 0.0;
    if (fabs(t_yaw   - _g_yaw)   < 2.0/180.0*M_PI) y_rate = 0.0;

    dji_sdk::Gimbal cmd;
    cmd.roll  = r_rate;
    cmd.pitch = p_rate;
    cmd.yaw   = y_rate;
    _gim_cmd_pub.publish(cmd);
}

void DjiInspectCtrl::update_time()
{
    _curr_time = ros::Time::now().toSec();
    if(_init_time) {
        _dt = 1.0/_freq;
    }
    else {
        _dt = _curr_time - _prev_time;
    }
    _prev_time = _curr_time;

    if(fabs(_dt) > 2.0/_freq || fabs(_dt) < 0.5/_freq ) {
        ROS_WARN("inspect_ctrl: delta t of time is %0.4f", _dt);
    }
}
void DjiInspectCtrl::update_position_error()
{
    if(_init_pos_err) {
        _err_pos_prev = _err_pos;
        _init_pos_err = false;
    }

    // Position error in body frame
    _err_pos = _l_rot.transpose() * (_t_pos - _l_pos);
    _err_rot_body = _l_rot.transpose() * _t_rot;

    // Integral of position error
    _intg_err_pos = _intg_err_pos + _err_pos;
    //ROS_INFO("integral of position x: %0.4f", _intg_err_pos.x());

    // Derivative of position error
    _diff_err_pos = (_err_pos - _err_pos_prev) / _dt;
    _err_pos_prev = _err_pos;

    if(_diff_pos_vec.size() < 5) {
        _diff_pos_vec.push_back(_diff_err_pos);
    }
    else {

        tf::Vector3 sum;
        sum.setZero();
        for(int i=0; i<_diff_pos_vec.size(); i++) {
            sum += _diff_pos_vec[i];
        }
        sum += _diff_err_pos;
        sum /= ((double)_diff_pos_vec.size()+1.0);
        _diff_err_pos = sum;

    _diff_pos_vec[_diff_pos_vec_idx%_diff_pos_vec.size()] = _diff_err_pos;
    _diff_pos_vec_idx++;

	/*std::vector<double> x, y, z;
	for(int i=0; i<_diff_pos_vec.size(); i++) {
		x.push_back(_diff_pos_vec[i].x());
		y.push_back(_diff_pos_vec[i].y());
		z.push_back(_diff_pos_vec[i].z());
	}
	std::sort(x.begin(), x.end());
	std::sort(y.begin(), z.end());
	std::sort(z.begin(), z.end());
	_diff_err_pos.setX(x[x.size()%2]);
	_diff_err_pos.setY(y[y.size()%2]);
	_diff_err_pos.setZ(z[z.size()%2]);*/
	
    }

    // Error yaw pitch roll in body frame
    _err_rot_body.getRPY(_err_roll, _err_pitch, _err_yaw);

}

void DjiInspectCtrl::update_position_control()
{

    if(fabs(_intg_err_pos.x()) > _MAX_POS_INTG) _intg_err_pos.setX(sgn(_intg_err_pos.x()) * _MAX_POS_INTG);
    if(fabs(_intg_err_pos.y()) > _MAX_POS_INTG) _intg_err_pos.setY(sgn(_intg_err_pos.y()) * _MAX_POS_INTG);
    if(fabs(_intg_err_pos.z()) > _MAX_POS_INTG) _intg_err_pos.setZ(sgn(_intg_err_pos.z()) * _MAX_POS_INTG);

    
    // Setting desired velocity
    _t_vel.setZero();
    _t_vel[0] = _err_pos.x() * _P_POS_X + _intg_err_pos.x() * _I_POS_X - _diff_err_pos.x() * _D_POS_X;
    _t_vel[1] = _err_pos.y() * _P_POS_Y + _intg_err_pos.y() * _I_POS_Y - _diff_err_pos.y() * _D_POS_Y;
    _t_vel[2] = _err_pos.z() * _P_POS_Z + _intg_err_pos.z() * _I_POS_Z - _diff_err_pos.z() * _D_POS_Z;
    
    if(fabs(_t_vel[0]) > _MAX_VEL_X) _t_vel[0] = sgn(_t_vel[0]) * _MAX_VEL_X;
    if(fabs(_t_vel[1]) > _MAX_VEL_Y) _t_vel[1] = sgn(_t_vel[1]) * _MAX_VEL_Y;
    if(fabs(_t_vel[2]) > _MAX_VEL_Z) _t_vel[2] = sgn(_t_vel[2]) * _MAX_VEL_Z;

    // Stacking info into pid messages for debugging
/*    
    _pid_msg.position_p.x = _err_pos.x() * _P_POS_X;
    _pid_msg.position_p.y = _err_pos.y() * _P_POS_Y;
    _pid_msg.position_p.z = _err_pos.z() * _P_POS_Z;
    _pid_msg.position_i.x = _intg_err_pos.x() * _I_POS_X; 
    _pid_msg.position_i.y = _intg_err_pos.y() * _I_POS_Y;
    _pid_msg.position_i.z = _intg_err_pos.z() * _I_POS_Z ;
    _pid_msg.position_d.x = _diff_err_pos.x() * _D_POS_X;
    _pid_msg.position_d.y = _diff_err_pos.y() * _D_POS_Y;
    _pid_msg.position_d.z = _diff_err_pos.z() * _D_POS_Z;
    _pid_msg.velocity.x = _t_vel.x();
    _pid_msg.velocity.y = _t_vel.y();
    _pid_msg.velocity.z = _t_vel.z();
*/
    geometry_msgs::PointStamped pidmsg;
    pidmsg.header.stamp = ros::Time::now();
    pidmsg.point.x = _err_pos.z() * _P_POS_Z;
    pidmsg.point.y = _intg_err_pos.z() * _I_POS_Z;
    pidmsg.point.z = _t_vel[2];

    _pid_msg_pub.publish(pidmsg);
}

void DjiInspectCtrl::update_velocity_error()
{
    if(_init_vel_err) {
        _err_vel_prev = _err_vel;
        _init_vel_err = false;
    }

    // Proportional term
    _err_vel = _t_vel - _l_vel;

    if(_vel_vec.size() < 5) {
        _vel_vec.push_back(_err_vel);
    }
    else {

        tf::Vector3 sum;
        sum.setZero();
        for(int i=0; i<_vel_vec.size(); i++) {
            sum += _vel_vec[i];
        }
        sum += _err_vel;
        sum /= ((double)_vel_vec.size()+1.0);
        _err_vel = sum;

        _vel_vec[_vel_vec_idx%5] = _err_vel;
        _vel_vec_idx++;
    }

    // Derivative term
    _diff_err_vel = (_err_vel - _err_vel_prev) / _dt;
    _err_vel_prev = _err_vel;

    // Stacking 5 previous differential values and average
    if(_diff_vel_vec.size() < 5) {
        _diff_vel_vec.push_back(_diff_err_vel);
    }
    else {

        tf::Vector3 sum;
        sum.setZero();
        for(int i=0; i<_diff_vel_vec.size(); i++) {
            sum += _diff_vel_vec[i];
        }
        sum += _diff_err_vel;
        sum /= ((double)_diff_vel_vec.size()+1.0);
        _diff_err_vel = sum;

        _diff_vel_vec[_diff_vel_vec_idx%5] = _diff_err_vel;
        _diff_vel_vec_idx++;
    }

    // Integral term
    _intg_err_vel += _err_vel;

}


void DjiInspectCtrl::update_velocity_control()
{
    
    if(fabs(_intg_err_vel.x()) > _MAX_VEL_INTG) _intg_err_vel.setX( sgn(_intg_err_vel.x()) * _MAX_VEL_INTG);
    if(fabs(_intg_err_vel.y()) > _MAX_VEL_INTG) _intg_err_vel.setY( sgn(_intg_err_vel.y()) * _MAX_VEL_INTG);

    // PD control
    _roll     =   _err_vel.y() * _P_VEL_Y - _diff_err_vel.y() * _D_VEL_Y + _intg_err_vel.y() * _I_VEL_Y + _INIT_VEL_INTG_Y;
    _pitch    = -(_err_vel.x() * _P_VEL_X - _diff_err_vel.x() * _D_VEL_X + _intg_err_vel.x() * _I_VEL_X + _INIT_VEL_INTG_X);
    _yaw_rate =   _err_yaw     * _P_ORI_Z * 180.0 / M_PI;
    _z_rate   =   _t_vel.z();

    // Just for testing. It looks like there exist saturations for roll, pitch and z rate, so add a feed forward term
    _roll += sgn(_roll) * _FW_ROLL;
    _pitch += sgn(_pitch) * _FW_PITCH;
    _z_rate += _FW_Z_RATE;

    // Bound max values
    _roll =     (fabs(_roll)     > _MAX_ROLL)     ? sgn(_roll)     * _MAX_ROLL     : _roll;
    _pitch =    (fabs(_pitch)    > _MAX_PITCH)    ? sgn(_pitch)    * _MAX_PITCH    : _pitch;
    _yaw_rate = (fabs(_yaw_rate) > _MAX_YAW_RATE) ? sgn(_yaw_rate) * _MAX_YAW_RATE : _yaw_rate;
    _z_rate =   (fabs(_z_rate)   > _MAX_Z_RATE)   ? sgn(_z_rate)   * _MAX_Z_RATE   : _z_rate;


    // Bound min values
    _roll     = (fabs(_roll)     > _MIN_ROLL  )   ? _roll     : 0.0;
    _pitch    = (fabs(_pitch)    > _MIN_PITCH )   ? _pitch    : 0.0;
    _yaw_rate = (fabs(_yaw_rate) > _MIN_YAW_RATE) ? _yaw_rate : 0.0;
    _z_rate   = (fabs(_z_rate)   > _MIN_Z_RATE)   ? _z_rate   : 0.0;

    //if(sgn(_z_rate) > 0.0) _z_rate *= 0.8;
    // Stacking PID info into messages
/*    
    _pid_msg.velocity_p.x = -_err_vel.x() * _P_VEL_X;
    _pid_msg.velocity_p.y =  _err_vel.y() * _P_VEL_Y;
    _pid_msg.velocity_p.z = 0.0;
    _pid_msg.velocity_i.x = -_intg_err_vel.x() * _I_VEL_X;
    _pid_msg.velocity_i.y =  _intg_err_vel.y() * _I_VEL_Y;
    _pid_msg.velocity_i.z = 0.0;
    _pid_msg.velocity_d.x =  _diff_err_vel.x() * _D_VEL_X;
    _pid_msg.velocity_d.y = -_diff_err_vel.y() * _D_VEL_Y;
    _pid_msg.velocity_d.z = 0.0;
    _pid_msg.roll = _roll;
    _pid_msg.pitch = _pitch;
    _pid_msg.velocity.z = _z_rate;
    _pid_msg.header.stamp = ros::Time::now();
    _pid_msg_pub.publish(_pid_msg);
*/
}


