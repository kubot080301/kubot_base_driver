#include "kubot_bringup/data_holder.h"

#if _DATAHOLDER_IN_DEVICES
#include "board.h"

void Data_holder::load_parameter()
{
	Board::get()->get_config((unsigned char*)&parameter, sizeof(parameter));
}

void Data_holder::save_parameter()
{
	Board::get()->set_config((unsigned char*)&parameter, sizeof(parameter));
}
#else
#include <ros/ros.h>
#endif

void Data_holder::dump_params(struct Robot_parameter* params)
{
#if _DATAHOLDER_IN_DEVICES
#else
	std::string model_name = "UNKNOWN";
	if (params->model_type == MODEL_TYPE_2WD_DIFF) {
		model_name = "2WD_DIFF";
	}
	else if (params->model_type == MODEL_TYPE_4WD_DIFF) {
		model_name = "4WD_DIFF";
	}
	else if (params->model_type == MODEL_TYPE_3WD_OMNI) {
		model_name = "3WD_OMNI";
	}
	else if (params->model_type == MODEL_TYPE_4WD_OMNI) {
		model_name = "4WD_OMNI";
	}
	else if (params->model_type == MODEL_TYPE_4WD_MECANUM) {
		model_name = "4WD_MECANUM";
	}

	std::string imu_name = "UNKNOWN";
	if (params->imu_type == IMU_TYPE_MPU6050) {
		imu_name = "MPU6050";
	}
	else if (params->imu_type == IMU_TYPE_GY85) {
		imu_name = "GY85";
	}
	else if (params->imu_type == IMU_TYPE_GY87) {
		imu_name = "GY87";
	}
	else if (params->imu_type == IMU_TYPE_MPU9250) {
		imu_name = "MPU6250";
	}
	else if (params->imu_type == IMU_TYPE_BMI160) {
		imu_name = "BMI160";
	}

	ROS_INFO("[KUBOT]RobotParameters:\n \
                        \t\t Model : %s(%d)\n \
                        \t\t Wheel_diameter :  %d mm\n \
                        \t\t Wheel_track : %d mm\n \
                        \t\t Encoder_resolution : 4*%d\n \
                        \t\t Motor_ratio : 1:%d\n\
                        \t\t Pid interval : %d ms\n\
                        \t\t Kp : %d/%d Ki : %d/%d Kd : %d/%d\n\
                        \t\t Cmd hold time : %d ms\n\
                        \t\t Velocity limit : x : %0.2f m/s y : %0.2f m/s a : %0.2f rad/s\n\
                        \t\t IMU: %s(%d)\n \
                        \t\t Sona_dis: %d mm\n \
                         "
		, model_name.c_str(), params->model_type
		, params->wheel_diameter
		, params->wheel_track
		, params->encoder_resolution / 4
		, params->motor_ratio
		, params->do_pid_interval
		, params->kp, params->ko, params->ki, params->ko, params->kd, params->ko
		, params->cmd_last_time
		, params->max_v_liner_x / 100., params->max_v_liner_y / 100., params->max_v_angular_z / 100.
		, imu_name.c_str(), params->imu_type
		, params->sona_distance
	);
#endif
}
