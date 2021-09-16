#include "base_driver.h"
#include "data_holder.h"
#include "simple_dataframe_master.h"

#include <std_msgs/Float32MultiArray.h>
#include <boost/assign/list_of.hpp>
#include <string.h> 

#ifdef USE_BOOST_SERIAL_TRANSPORT
#include "serial_transport.h"
#else
#include "serial_transport2.h"
#endif

BaseDriver* BaseDriver::instance = NULL;

BaseDriver::BaseDriver() : pn("~"), bdg(pn)
{
	//init config
	bdg.init(&Data_holder::get()->parameter);

#ifdef USE_BOOST_SERIAL_TRANSPORT
	trans = boost::make_shared<Serial_transport>(bdg.port, bdg.baudrate);
#else
	trans = boost::make_shared<Serial_transport2>(bdg.port, bdg.baudrate);
#endif

	frame = boost::make_shared<Simple_dataframe>(trans.get());

	ROS_INFO("[KUBOT]base driver startup...");
	if (trans->init())
	{
		ROS_INFO("[KUBOT]connected to driver board");
	}
	else
	{
		ROS_ERROR("[KUBOT]oops!!! can't connect to driver board, please check the usb connection or baudrate!");
		return;
	}

	ros::Duration(3).sleep(); //wait for device
	ROS_INFO("[KUBOT]wait for device...");

	frame->init();

	for (int i = 0;i < 3;i++) {
		if (frame->interact(ID_GET_VERSION))
			break;
		ros::Duration(2).sleep(); //wait for device
	}

	ROS_INFO("[KUBOT]robot version:%s build time:%s", Data_holder::get()->firmware_info.version,
		Data_holder::get()->firmware_info.time);

	init_robot_ip();

	init_cmd_odom();

	init_pid_debug();

	read_param();

	init_imu();

	init_sona_data();

	init_robot_status();
}

BaseDriver::~BaseDriver()
{
	if (instance != NULL)
		delete instance;
}

void BaseDriver::init_robot_ip()
{
	strcpy(Data_holder::get()->lcd_status.robot_ip, bdg.robot_ip.c_str());
	frame->interact(ID_SET_ROBOT_IP);
}

void BaseDriver::init_cmd_odom()
{
	frame->interact(ID_INIT_ODOM);

	ROS_INFO_STREAM("[KUBOT]subscribe cmd topic on [" << bdg.cmd_vel_topic << "]");
	cmd_vel_sub = nh.subscribe(bdg.cmd_vel_topic, 1000, &BaseDriver::cmd_vel_callback, this);

	ROS_INFO_STREAM("[KUBOT]advertise odom topic on [" << bdg.odom_topic << "]");
	odom_pub = nh.advertise<nav_msgs::Odometry>(bdg.odom_topic, 50);

	//init odom_trans
	odom_trans.header.frame_id = bdg.odom_frame;
	odom_trans.child_frame_id = bdg.base_frame;

	odom_trans.transform.translation.z = 0;

	//init odom
	odom.header.frame_id = bdg.odom_frame;
	odom.pose.pose.position.z = 0.0;
	odom.child_frame_id = bdg.base_frame;
	odom.twist.twist.linear.y = 0;

	if (!bdg.publish_tf)
	{
		odom.pose.covariance = boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
			(0) (1e-3)  (0)  (0)  (0)  (0)
			(0)   (0)  (1e6) (0)  (0)  (0)
			(0)   (0)   (0) (1e6) (0)  (0)
			(0)   (0)   (0)  (0) (1e6) (0)
			(0)   (0)   (0)  (0)  (0)  (1e3);

		odom.twist.covariance = boost::assign::list_of(1e-3) (0)   (0)  (0)  (0)  (0)
			(0) (1e-3)  (0)  (0)  (0)  (0)
			(0)   (0)  (1e6) (0)  (0)  (0)
			(0)   (0)   (0) (1e6) (0)  (0)
			(0)   (0)   (0)  (0) (1e6) (0)
			(0)   (0)   (0)  (0)  (0)  (1e3);
	}

	need_update_speed = false;

	//ROS_INFO_STREAM("subscribe cmd topic on [correct_pos]");
	//correct_pos_sub = nh.subscribe("correct_pos", 1000, &BaseDriver::correct_pos_callback, this);
}

void BaseDriver::cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
{
	ROS_INFO_STREAM("cmd_vel:[" << vel_cmd.linear.x << " " << vel_cmd.linear.y << " " << vel_cmd.angular.z << "]");

	Data_holder::get()->velocity.v_liner_x = vel_cmd.linear.x * 100;
	Data_holder::get()->velocity.v_liner_y = vel_cmd.linear.y * 100;
	Data_holder::get()->velocity.v_angular_z = vel_cmd.angular.z * 100;

	need_update_speed = true;
}

void BaseDriver::init_pid_debug()
{
	if (bdg.out_pid_debug_enable)
	{
		const char* input_topic_name[MAX_MOTOR_COUNT] = { "motor1_input", "motor2_input", "motor3_input", "motor4_input" };
		const char* output_topic_name[MAX_MOTOR_COUNT] = { "motor1_output", "motor2_output", "motor3_output", "motor4_output" };
		for (size_t i = 0; i < MAX_MOTOR_COUNT; i++)
		{
			pid_debug_pub_input[i] = nh.advertise<std_msgs::Int32>(input_topic_name[i], 1000);
			pid_debug_pub_output[i] = nh.advertise<std_msgs::Int32>(output_topic_name[i], 1000);
		}
	}
}

void BaseDriver::read_param()
{
	Robot_parameter* param = &Data_holder::get()->parameter;
	memset(param, 0, sizeof(Robot_parameter));

	frame->interact(ID_GET_ROBOT_PARAMTER);

	Data_holder::dump_params(param);

	bdg.SetRobotParameters();
}

void BaseDriver::init_imu()
{
	raw_imu_pub = nh.advertise<kubot_msgs::RawImu>("raw_imu", 50);
	raw_imu_msgs.header.frame_id = "imu_link";
	raw_imu_msgs.accelerometer = true;
	raw_imu_msgs.gyroscope = true;
	raw_imu_msgs.magnetometer = true;
}

void BaseDriver::init_sona_data()
{
	raw_sona_data_pub = nh.advertise<kubot_msgs::RawSona>("raw_sona_data_msgs", 50);
	raw_robot_status_msgs.header.frame_id = "sona_data";
}

void BaseDriver::init_robot_status()
{
	raw_robot_status_pub = nh.advertise<kubot_msgs::RawRobot>("raw_robot_status_msgs", 50);
	raw_robot_status_msgs.header.frame_id = "robot_status";
}

void BaseDriver::work_loop()
{
	ros::Rate loop(bdg.freq);
	while (ros::ok()) {
		//boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
		update_param();

		update_odom();

		update_pid_debug();

		update_speed();

		if (Data_holder::get()->parameter.imu_type == IMU_TYPE_MPU6050
			|| Data_holder::get()->parameter.imu_type == IMU_TYPE_GY85
			|| Data_holder::get()->parameter.imu_type == IMU_TYPE_GY87
			|| Data_holder::get()->parameter.imu_type == IMU_TYPE_MPU9250
			|| Data_holder::get()->parameter.imu_type == IMU_TYPE_BMI160)
		{
			update_imu();
		}

		update_robot_status();

		update_sona_data();

		loop.sleep();

		ros::spinOnce();
	}
}

void BaseDriver::update_param()
{
#ifdef USE_DYNAMIC_RECONFIG
	if (bdg.get_param_update_flag())
	{
		frame->interact(ID_SET_ROBOT_PARAMTER);
		ros::Rate loop(5);
		loop.sleep();
	}
#endif
}

void BaseDriver::update_odom()
{
	frame->interact(ID_GET_ODOM);

	ros::Time current_time = ros::Time::now();

	float x = Data_holder::get()->odom.x * 0.01;
	float y = Data_holder::get()->odom.y * 0.01;
	float th = Data_holder::get()->odom.yaw * 0.01;

	float vxy = Data_holder::get()->odom.v_liner_x * 0.01;
	float vth = Data_holder::get()->odom.v_angular_z * 0.01;

	//ROS_INFO("odom: x=%.2f y=%.2f th=%.2f vxy=%.2f vth=%.2f", x, y ,th, vxy,vth);

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	//publish_tf
	if (bdg.publish_tf)
	{
		odom_trans.header.stamp = current_time;
		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.rotation = odom_quat;
		odom_broadcaster.sendTransform(odom_trans);
	}

	//publish the message  
	odom.header.stamp = current_time;
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.orientation = odom_quat;
	odom.twist.twist.linear.x = vxy;
	odom.twist.twist.angular.z = vth;

	odom_pub.publish(odom);
}

void BaseDriver::update_speed()
{
	if (need_update_speed)
	{
		ROS_INFO_STREAM("update_speed");
		need_update_speed = !(frame->interact(ID_SET_VELOCITY));
	}
}

void BaseDriver::update_pid_debug()
{
	if (bdg.out_pid_debug_enable)
	{
		frame->interact(ID_GET_PID_DATA);

		for (size_t i = 0; i < MAX_MOTOR_COUNT; i++)
		{
			pid_debug_msg_input[i].data = Data_holder::get()->pid_data.input[i];
			pid_debug_msg_output[i].data = Data_holder::get()->pid_data.output[i];

			pid_debug_pub_input[i].publish(pid_debug_msg_input[i]);
			pid_debug_pub_output[i].publish(pid_debug_msg_output[i]);
		}
	}
}

void BaseDriver::update_imu()
{
	frame->interact(ID_GET_IMU_DATA);
	raw_imu_msgs.header.stamp = ros::Time::now();

	raw_imu_msgs.raw_linear_acceleration.x = Data_holder::get()->imu_data[0];
	raw_imu_msgs.raw_linear_acceleration.y = Data_holder::get()->imu_data[1];
	raw_imu_msgs.raw_linear_acceleration.z = Data_holder::get()->imu_data[2];
	raw_imu_msgs.raw_angular_velocity.x = Data_holder::get()->imu_data[3];
	raw_imu_msgs.raw_angular_velocity.y = Data_holder::get()->imu_data[4];
	raw_imu_msgs.raw_angular_velocity.z = Data_holder::get()->imu_data[5];
	raw_imu_msgs.raw_magnetic_field.x = Data_holder::get()->imu_data[6];
	raw_imu_msgs.raw_magnetic_field.y = Data_holder::get()->imu_data[7];
	raw_imu_msgs.raw_magnetic_field.z = Data_holder::get()->imu_data[8];

	raw_imu_pub.publish(raw_imu_msgs);
}

void BaseDriver::update_sona_data() 
{
	frame->interact(ID_GET_SONA_DATA);
	raw_sona_data_msgs.header.stamp = ros::Time::now();

	raw_sona_data_msgs.sona1_dis = Data_holder::get()->sona_data[0];
	raw_sona_data_msgs.sona2_dis = Data_holder::get()->sona_data[1];
	raw_sona_data_msgs.sona3_dis = Data_holder::get()->sona_data[2];
	raw_sona_data_msgs.sona4_dis = Data_holder::get()->sona_data[3];
	raw_sona_data_msgs.sona5_dis = Data_holder::get()->sona_data[4];
	raw_sona_data_msgs.sona6_dis = Data_holder::get()->sona_data[5];
	raw_sona_data_msgs.sona7_dis = Data_holder::get()->sona_data[6];
	raw_sona_data_msgs.sona8_dis = Data_holder::get()->sona_data[7];

	raw_sona_data_pub.publish(raw_sona_data_msgs);
}

int UPDATE_ROBOT_STATUS_INTERVAL = 1;
void BaseDriver::update_robot_status()
{
	static int last_millis = 0;
	if (ros::Time::now().toSec() - last_millis > UPDATE_ROBOT_STATUS_INTERVAL) {
		frame->interact(ID_GET_ROBOT_STATUS);
		raw_robot_status_msgs.header.stamp = ros::Time::now();

		raw_robot_status_msgs.bumper_status = Data_holder::get()->robot_status.bumper_status;

		if (bdg.mcu_battery_volatge)
		{
			raw_robot_status_msgs.mcu_voltage = Data_holder::get()->robot_status.mcu_voltage;
		}

		raw_robot_status_pub.publish(raw_robot_status_msgs);
		last_millis = ros::Time::now().toSec();
	}
}