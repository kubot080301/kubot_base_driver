#ifndef KUBOT_BASE_DRIVER_H_
#define KUBOT_BASE_DRIVER_H_

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include "base_driver_config.h"

#include <geometry_msgs/Twist.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

#include "kubot_bringup/transport.h"
#include "kubot_bringup/dataframe.h"
#include <kubot_msgs/RawImu.h>
#include <kubot_msgs/RawRobot.h>
#include <kubot_msgs/RawSona.h>

class BaseDriver
{
private:
	BaseDriver();

	static BaseDriver* instance;

	BaseDriverConfig bdg;
	boost::shared_ptr<Transport> trans;
	boost::shared_ptr<Dataframe> frame;

	ros::NodeHandle nh;
	ros::NodeHandle pn;

public:
	static BaseDriver* Instance()
	{
		if (instance == NULL)
			instance = new BaseDriver();

		return instance;
	}
	~BaseDriver();

	void work_loop();

	BaseDriverConfig& getBaseDriverConfig()
	{
		return bdg;
	}

	ros::NodeHandle* getNodeHandle()
	{
		return &nh;
	}

	ros::NodeHandle* getPrivateNodeHandle()
	{
		return &pn;
	}

// Get robot parameter
private:
	void update_param();
	void read_param();


// Set command velocity...
private:
	void init_cmd_odom();
	void update_speed();
	void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd);
	ros::Subscriber cmd_vel_sub;

	double last_cmd_vel_time;
	bool need_update_speed;

// Set pid debug...
private:
	void init_pid_debug();
	void update_pid_debug();
#define MAX_MOTOR_COUNT 4
	ros::Publisher pid_debug_pub_input[MAX_MOTOR_COUNT];
	ros::Publisher pid_debug_pub_output[MAX_MOTOR_COUNT];

	std_msgs::Int32 pid_debug_msg_input[MAX_MOTOR_COUNT];
	std_msgs::Int32 pid_debug_msg_output[MAX_MOTOR_COUNT];

// Set odom...
private:
	void update_odom();
	ros::Publisher odom_pub;
	nav_msgs::Odometry odom;
	geometry_msgs::TransformStamped odom_trans;
	tf::TransformBroadcaster odom_broadcaster;

// Get IMU data...
private:
	void init_imu();
	void update_imu();
	kubot_msgs::RawImu raw_imu_msgs;
	ros::Publisher raw_imu_pub;

// Display robot ip...
private:
	void init_robot_ip();

// Get ultrasonic data...
private:
	void init_sona_data();
	void update_sona_data();
	kubot_msgs::RawSona raw_sona_data_msgs;
	ros::Publisher raw_sona_data_pub;

// Get robot status...
private:
	void init_robot_status();
	void update_robot_status();
	kubot_msgs::RawRobot raw_robot_status_msgs;
	ros::Publisher raw_robot_status_pub;
};

#endif /* KUBOT_BASE_DRIVER_H_ */