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
    
    /*
    if (!nh.getParam("DEBUG", _DEBUG)){
        ROS_ERROR("inspect_ctrl: can not get DEBUG param");
    }
    if (!nh.getParam("USE_DJI_VEL", _USE_DJI_VEL)){
        ROS_ERROR("inspect_ctrl: can not get USE_DJI_VEL param");
    }
    */

    // Command publisher
    _dji_cmd_pub = nh.advertise<geometry_msgs::QuaternionStamped> ("/dji_inspect_ctrl/cmd", 100);
    //_pid_msg_pub = nh.advertise<dji_inspect_ctrl::pid_msgs> ("/dji_inspect_ctrl/pid_msgs", 100);
    
    // Subscribers
    _laser_odom_sub  = nh.subscribe(_laser_odom_topic_name, 50, &DjiInspectCtrl::laser_odom_cb, this);
    //_target_pose_sub = nh.subscribe("/dji_inspect_ctrl/target_pose", 50, &DjiInspectCtrl::target_cb, this);

    /*
    if(_DEBUG) {
        _target_vel_sub = nh.subscribe("/dji_inspect_ctrl/target_vel", 50, &DjiInspectCtrl::target_vel_cb, this);
    }
    if(_USE_DJI_VEL) {
        ROS_WARN("inspect_ctrl: using dji guidance velocity estimation");
        _dji_odom_sub = nh.subscribe("/dji_sdk/odometry", 50, &DjiInspectCtrl::dji_odom_cb, this);
    }
    */

    _init_pos_err = true;
    _init_vel_err = true;
    _init_time = true;

    _t_rot.setIdentity();
    _l_rot.setIdentity();
    _t_pos.setZero();
    _l_pos.setZero();
    _t_vel.setZero();
    _l_vel.setZero();

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

}

void DjiInspectCtrl::set_target(geometry_msgs::PoseStamped &target) {
    tf::quaternionMsgToTF(target.pose.orientation, _t_quat);

    _t_rot.setRotation(_t_quat);

    double y, p, r;
    _t_rot.getRPY(r,p,y);

    // Check if roll and pitch is zero;
    if (fabs(r) > 0.00001 || fabs(p) > 0.00001) {
        ROS_WARN("Target roll or pitch is not zeros.");
    }

    _t_rot.setRPY(0.0,0.0,y);
    _t_pos.setValue(target.pose.position.x, target.pose.position.y, target.pose.position.z);

    ROS_INFO_STREAM("Target set: x=" << target.pose.position.x << " / y=" << target.pose.position.y << " / z=" << target.pose.position.z << " / yaw=" << y);
}

/*
void DjiInspectCtrl::target_cb(const geometry_msgs::PoseStamped &msg) {

    tf::quaternionMsgToTF(msg.pose.orientation, _t_quat);

    _t_rot.setRotation(_t_quat);

    double y, p, r;
    _t_rot.getRPY(r,p,y);

    // Check if roll and pitch is zero;
    if (fabs(r) > 0.00001 || fabs(p) > 0.00001) {
        ROS_WARN("Target roll or pitch is not zeros.");
    }

    _t_rot.setRPY(0.0,0.0,y);
    _t_pos.setValue(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);

    ROS_INFO_ONCE("Target callback");
}
*/

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
    //if(!_USE_DJI_VEL) { 
	_l_vel.setValue(msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z);
	_l_vel = _l_rot.transpose() * _l_vel;
    //}
    ROS_INFO_STREAM_THROTTLE(1.0, "Laser localization callback");

    update_time();
    //if(!_DEBUG) 
    {
        update_position_error();
        update_position_control();
    }
    update_velocity_error();
    update_velocity_control();
    publish_cmd();
}

/*
void DjiInspectCtrl::dji_odom_cb(const nav_msgs::Odometry &msg) {
	tf::Matrix3x3 g_rot;
	tf::Quaternion g_quat;
	tf::Vector3 g_vel;

	tf::quaternionMsgToTF(msg.pose.pose.orientation, g_quat);
	g_rot.setRotation(g_quat);
	g_vel.setValue(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);

	if(_USE_DJI_VEL) {
	    _l_vel = g_rot.transpose() * g_vel;
	}
	//ROS_INFO("inspect_ctrl: velocity = %0.2f %0.2f %0.2f", _l_vel[0], _l_vel[1], _l_vel[2]);
}
*/
/*
void DjiInspectCtrl::target_vel_cb(const geometry_msgs::Point &msg)
{
    _t_vel[0] = msg.x;
    _t_vel[1] = msg.y;
    _t_vel[2] = msg.z;

    _pid_msg.velocity.x = _t_vel.x();
    _pid_msg.velocity.y = _t_vel.y();
    _pid_msg.velocity.z = _t_vel.z();
}
*/

void DjiInspectCtrl::publish_cmd() {

    // Publish command convert back to radius
    geometry_msgs::QuaternionStamped cmd;

    cmd.header.frame_id = "dji";
    cmd.header.stamp      = ros::Time::now();
    cmd.quaternion.w      = _yaw_rate;
    cmd.quaternion.x      = _roll;
    cmd.quaternion.y      = _pitch;
    cmd.quaternion.z      = _z_rate;

    _dji_cmd_pub.publish(cmd);

    ROS_INFO_ONCE("CMD publish");

}


void DjiInspectCtrl::get_cmd(double &roll, double &pitch, double &vz, double &yawrate) {
    roll    = _roll;
    pitch   = _pitch;
    vz      = _z_rate;
    yawrate =  _yaw_rate;
    ROS_INFO_STREAM_THROTTLE(1.0, "CMD: roll=" << roll << " / pitch=" << pitch << " / yawrate=" << yawrate << " / vz=" << vz);
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
    _z_rate += sgn(_z_rate) * _FW_Z_RATE;

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


