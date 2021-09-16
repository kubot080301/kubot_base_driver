#include "base_driver_config.h"
#include "data_holder.h"

#define PI 3.1415926f

BaseDriverConfig::BaseDriverConfig(ros::NodeHandle &p) : pn(p)
{
#ifdef USE_DYNAMIC_RECONFIG
	param_update_flag = false;
#endif
	set_flag = true;
}

BaseDriverConfig::~BaseDriverConfig()
{
}

void BaseDriverConfig::init(Robot_parameter *r)
{
	rp = r;

	// Comm param
	pn.param<std::string>("port", port, "/dev/ttyACM0");
	pn.param<int32_t>("baudrate", baudrate, 115200);
	pn.param<std::string>("robot_ip", robot_ip, "192.168.172.1");

	ROS_INFO("[KUBOT]robot_ip:%s", robot_ip.c_str());
	ROS_INFO("[KUBOT]port:%s baudrate:%d", port.c_str(), baudrate);

	pn.param<std::string>("base_frame", base_frame, "base_link");
	pn.param<std::string>("odom_frame", odom_frame, "odom");
	pn.param<bool>("publish_tf", publish_tf, true);

	pn.param<bool>("out_pid_debug_enable", out_pid_debug_enable, false);
	ROS_INFO("[KUBOT]out_pid_debug_enable:%d", out_pid_debug_enable);
	pn.param<bool>("mcu_battery_volatge", mcu_battery_volatge, false);
	ROS_INFO("[KUBOT]mcu_battery_voltage:%d", mcu_battery_volatge);

	// Topic name param
	pn.param<std::string>("cmd_vel_topic", cmd_vel_topic, "cmd_vel");
	pn.param<std::string>("odom_topic", odom_topic, "odom");
	pn.param<std::string>("robot_status_topic", robot_status_topic, "robot_status");
	pn.param<std::string>("sona_data_topic", sona_data_topic, "sona_data");

	pn.param<int32_t>("freq", freq, 1000);
}

void BaseDriverConfig::SetRobotParameters()
{
#ifdef USE_DYNAMIC_RECONFIG
	static bool flag = true;
	if (flag)
	{
		flag = false;
		f = boost::bind(&BaseDriverConfig::dynamic_callback, this, _1, _2);
		server.setCallback(f);
	}
#endif
}

#ifdef USE_DYNAMIC_RECONFIG
void BaseDriverConfig::dynamic_callback(kubot_bringup::kubot_driverConfig &config, uint32_t level)
{
	if (set_flag)
	{
		set_flag = false;
		config.wheel_diameter = rp->wheel_diameter;
		config.wheel_track = rp->wheel_track;
		config.do_pid_interval = rp->do_pid_interval;
		config.encoder_resolution = rp->encoder_resolution;
		config.kp = rp->kp;
		config.ki = rp->ki;
		config.kd = rp->kd;
		config.ko = rp->ko;
		config.cmd_last_time = rp->cmd_last_time;
		config.max_v_liner_x = rp->max_v_liner_x;
		config.max_v_liner_y = rp->max_v_liner_y;
		config.max_v_angular_z = rp->max_v_angular_z;
		config.imu_type = rp->imu_type;
		config.motor_ratio = rp->motor_ratio;
		config.model_type = rp->model_type;
		config.sona_distance = rp->sona_distance;
		return;
	}

	rp->wheel_diameter = config.wheel_diameter;
	rp->wheel_track = config.wheel_track;
	rp->do_pid_interval = config.do_pid_interval;
	rp->encoder_resolution = config.encoder_resolution;
	rp->kp = config.kp;
	rp->ki = config.ki;
	rp->kd = config.kd;
	rp->ko = config.ko;
	rp->cmd_last_time = config.cmd_last_time;
	rp->max_v_liner_x = config.max_v_liner_x;
	rp->max_v_liner_y = config.max_v_liner_y;
	rp->max_v_angular_z = config.max_v_angular_z;
	rp->imu_type = config.imu_type;
	rp->motor_ratio = config.motor_ratio;
	rp->model_type = config.model_type;
	rp->sona_distance = config.sona_distance;

	Data_holder::dump_params(rp);

	param_update_flag = true;
}

bool BaseDriverConfig::get_param_update_flag()
{
	bool tmp = param_update_flag;
	param_update_flag = false;

	return tmp;
}

#endif