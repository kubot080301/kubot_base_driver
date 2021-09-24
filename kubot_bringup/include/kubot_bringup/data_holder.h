#ifndef KUBOT_DATA_HOLDER_H_
#define KUBOT_DATA_HOLDER_H_

#include <string.h>

#pragma pack(1)

using namespace std;

typedef int int32;
typedef short int16;
typedef unsigned short unint16;

enum IMU_TYPE
{
   IMU_TYPE_MPU6050 = 65,
   IMU_TYPE_GY85 = 69,
   IMU_TYPE_GY87 = 71,
   IMU_TYPE_MPU9250 = 92,
   IMU_TYPE_BMI160 = 160,
   IMU_TYPE_DISABLE = 255,
};

enum MODEL_TYPE
{
   MODEL_TYPE_2WD_DIFF = 1,
   MODEL_TYPE_4WD_DIFF = 2,
   MODEL_TYPE_3WD_OMNI = 11,
   MODEL_TYPE_4WD_OMNI = 12,
   MODEL_TYPE_4WD_MECANUM = 21,
};

struct Robot_firmware
{
   char version[16];   // Driver board version.
   char time[16];   // Build time.
};

struct Robot_parameter
{
   union
   {
      char buff[64];   // Total length is 64 bits.
      struct
      {
         unsigned short wheel_diameter;	// Wheel diameter (mm).
         unsigned short wheel_track;	// Wheel track (mm).
         unsigned short encoder_resolution;	// Encoder resolution.
         unsigned char do_pid_interval;	// PID interval (ms).
         unsigned short kp;
         unsigned short ki;
         unsigned short kd;
         unsigned short ko;
         unsigned short cmd_last_time;   // After this time, cmd_vel will automatically stop moving.
         unsigned short max_v_liner_x;   // Maximum linear velocity in X direction (cm/s). 
         unsigned short max_v_liner_y;   // Maximum linear velocity in Y direction (cm/s) (DIFF model is 0). 
         unsigned short max_v_angular_z;   // Maximum rotation velocity (0.01 rad/s).
         unsigned char imu_type;
         unsigned short motor_ratio;	// Motor reduction ratio.
         unsigned char model_type;   // Motion model type.
         unsigned short sona_distance;   // Ultrasonic induction deceleration distance.
      };
   };
};

struct Robot_velocity
{
   short v_liner_x;  // Linear velocity in X direction. (cm/s)
   short v_liner_y;   // Linear velocity in Y direction(cm/s) (DIFF model is 0). 
   short v_angular_z;   // Angular velocity (0.01 rad/s).
};

struct Robot_odom   // Odometer calculated by Driver Board.
{
   short v_liner_x;	//	Forward is > 0, Back is < 0, cm/s.
   short v_liner_y;	// Left is > 0, Right is < 0, cm/s.
   short v_angular_z;	// Left is > 0, Right is < 0, cm/s.
   int32 x;   // Odometer X coordinate (cm).
   int32 y;   // Odometer Y coordinate (cm).
   short yaw;   // Odometer Yaw coordinate (0.01 rad).
};

struct Robot_pid_data
{
   int32 input[4];	// The input response of each motor.
   int32 output[4];	// The output response of each motor.
};

struct Robot_imu
{
   union
   {
      float imu_data[9];
      struct
      {
         float ax;   // X linear acceleration (m/s^2).
         float ay;   // Y linear acceleration (m/s^2).
         float az;   // Z linear acceleration (m/s^2).
         float gx;   // X angular rate (rad/s^2).
         float gy;   // Y angular rate (rad/s^2).
         float gz;   // Z angular rate (rad/s^2).
         float mx;   // X gauss magnetic (mGA).
         float my;   // Y gauss magnetic (mGA).
         float mz;   // Z gauss magnetic (mGA).
      } imu;
   };
};

struct Robot_sona
{
   union
   {
      unsigned char sona_data[8];   // The sensor returns data of each ultrasonic (mm). 
      struct
      {
         unsigned char sona1_data;
         unsigned char sona2_data;
         unsigned char sona3_data;
         unsigned char sona4_data;
         unsigned char sona5_data;
         unsigned char sona6_data;
         unsigned char sona7_data;
         unsigned char sona8_data;
      } sona;
   };
};

struct Robot_status
{
   bool bumper_status;	// Robot bumper status (true is collision!)
   float mcu_voltage; // Driver board voltage (V).
};

struct Robot_lcd_status
{
   //	char robot_name;
   char robot_ip[16];	// ROS IP. It will be displayed on the LCD panel on the robot.
};

#pragma pack(0)

// ALL DATA HOLDER 
class Data_holder
{
public:
   static Data_holder* get()
   {
      static Data_holder dh;
      return &dh;
   }

   void load_parameter();
   void save_parameter();

   static void dump_params(struct Robot_parameter* params);

private:
   Data_holder()
   {
      memset(&firmware_info, 0, sizeof(struct Robot_firmware));
      memset(&parameter, 0, sizeof(struct Robot_parameter));
      memset(&velocity, 0, sizeof(struct Robot_velocity));
      memset(&odom, 0, sizeof(struct Robot_odom));
      memset(&pid_data, 0, sizeof(struct Robot_pid_data));
      memset(&imu_data, 0, sizeof(imu_data));
      memset(&sona_data, 0, sizeof(sona_data));
      memset(&robot_status, 0, sizeof(struct Robot_status));
      memset(&lcd_status, 0, sizeof(struct Robot_lcd_status));
   }

public:
   struct Robot_firmware firmware_info;
   struct Robot_parameter parameter;
   struct Robot_velocity velocity;
   struct Robot_odom odom;
   struct Robot_pid_data pid_data;
   float imu_data[9];
   unsigned char sona_data[8];
   struct Robot_status robot_status;
   struct Robot_lcd_status lcd_status;
};

#endif /* KUBOT_DATA_HOLDER_H_ */