#include <chrono>
#include <memory>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/fcntl.h>
#include <time.h>
#include <sys/types.h>
#include <vector>
#include <iostream>
#include <dirent.h>
#include <signal.h>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>

//interface package srv include...
#include "interfaces/srv/imu_reset.hpp"


// #define SERIAL_PORT	"/dev/ttyUSB0"
#define SERIAL_SPEED	B115200
#define gravity_accelation 9.806055

typedef struct IMU_DATA
{
	double dQuaternion_x = 0.0;
	double dQuaternion_y = 0.0;
	double dQuaternion_z = 0.0;
	double dQuaternion_w = 1.0;

	double dAngular_velocity_x = 0.0;
	double dAngular_velocity_y = 0.0;
	double dAngular_velocity_z = 0.0;
	
	double dLinear_acceleration_x = 0.0;
	double dLinear_acceleration_y = 0.0;
	double dLinear_acceleration_z = 0.0;
    
	double dEuler_angle_Roll = 0.0;
	double dEuler_angle_Pitch = 0.0;
	double dEuler_angle_Yaw = 0.0;

}IMU_DATA;
IMU_DATA _pIMU_data;

int serial_fd = -1;
double time_offset_in_seconds;
double dSend_Data[10];
double m_dRoll, m_dPitch, m_dYaw;
//single_used TF
// bool m_bSingle_TF_option = true; //false;
// int sensor_hz_ = 100;
// string child_frame_ = "imu";
// string parent_frame_ = "base_link";
// string serial_port_ = "/dev/ttyUSB0";

using namespace std::chrono_literals;

std::atomic<bool> stop_requested(false);
void signal_handler(int signal) 
{
    stop_requested = true;
    printf("Signal received: %d. Preparing to shutdown...\n", signal);
}

//iahrs_driver class define
class IAHRS : public rclcpp::Node
{
public:
	IAHRS() : Node("iahrs_driver")
	{
		tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);


		//Service ListUp
		euler_angle_reset_srv_ = create_service<interfaces::srv::ImuReset>(
        	"all_data_reset", 
		std::bind(&IAHRS::Euler_angle_reset_callback, this, std::placeholders::_1, std::placeholders::_2));

		//PARAM
		this->declare_parameter("m_bSingle_TF_option", rclcpp::PARAMETER_BOOL);
		this->declare_parameter("sensor_hz", 20);
		this->declare_parameter("child_frame", "imu");
		this->declare_parameter("parent_frame", "base_link");
		this->declare_parameter("serial_port", "/dev/ttyUSB0");
		this->declare_parameter("qos_reliability", "best_effort");
		this->declare_parameter("is_compensation_enable", false);
		
		//Get Param
		// m_bSingle_TF_option = 
		// sensor_hz_ = 
		// child_frame_ = 
		// parent_frame_ = 
		// serial_port_ = 
		// qos_reliability_ =
		this->get_parameter("m_bSingle_TF_option", m_bSingle_TF_option);
		this->get_parameter("sensor_hz", sensor_hz_);
		this->get_parameter("child_frame", child_frame_);
		this->get_parameter("parent_frame", parent_frame_);
		this->get_parameter("serial_port", serial_port_);
		this->get_parameter("qos_reliability", qos_reliability_);
		this->get_parameter("is_compensation_enable", is_compensation_enable_);

		printf("[DEBUG] sensor_hz: %d\n", sensor_hz_);

		rmw_qos_profile_t profile = rmw_qos_profile_sensor_data;
		profile.reliability = (qos_reliability_ == "best_effort") ? 
			RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT : 
			RMW_QOS_POLICY_RELIABILITY_RELIABLE;

		rclcpp::QoS imu_qos(rclcpp::QoSInitialization::from_rmw(profile));
		imu_data_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", imu_qos);
		variance_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("imu/variance", 10);
	}

	////value//////////////////////////////////////////////////////////////////////////////
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
	geometry_msgs::msg::TransformStamped transformStamped;
	sensor_msgs::msg::Imu imu_data_msg;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr variance_pub;
	////function//////////////////////////////////////////////////////////////////////////////
	int serial_open ()
	{
		printf ("Try to open serial: %s\n", serial_port_.c_str()); 

		// serial_fd = open(SERIAL_PORT, O_RDWR|O_NOCTTY);
		serial_fd = open(serial_port_.c_str(), O_RDWR|O_NOCTTY);
		if (serial_fd < 0) 
		{
			printf ("Error unable to open %s\n", serial_port_.c_str());
			return -1;
		}
		printf ("%s open success\n", serial_port_.c_str());

		struct termios tio;
		tcgetattr(serial_fd, &tio);
		cfmakeraw(&tio);
		tio.c_cflag = CS8|CLOCAL|CREAD;
		tio.c_iflag &= ~(IXON | IXOFF);
		cfsetspeed(&tio, SERIAL_SPEED);
		tio.c_cc[VTIME] = 0;
		tio.c_cc[VMIN] = 0;

		int err = tcsetattr(serial_fd, TCSAFLUSH, &tio);
		if (err != 0) 
		{
			printf ("Error tcsetattr() function return error\n");
			close(serial_fd);
			serial_fd = -1;
			return -1;
		}
		return 0;
	}

	static unsigned long GetTickCount() 
	{
		struct timespec ts;
	
		clock_gettime (CLOCK_MONOTONIC, &ts);

		return ts.tv_sec*1000 + ts.tv_nsec/1000000;
	}

	int SendRecv(const char* command, double* returned_data, int data_length)
	{
		#define COMM_RECV_TIMEOUT	30	

		char temp_buff[256];
		read (serial_fd, temp_buff, 256);

		int command_len = strlen(command);
		int n = write(serial_fd, command, command_len);

		if (n < 0) return -1;

		const int buff_size = 1024;
		int  recv_len = 0;
		char recv_buff[buff_size + 1];

		unsigned long time_start = GetTickCount();

		while (recv_len < buff_size) 
		{
			int n = read (serial_fd, recv_buff + recv_len, buff_size - recv_len);
			if (n < 0) 
			{
				return -1;
			}
			else if (n == 0) 
			{
				usleep(1000);
			}
			else if (n > 0) 
			{
				recv_len += n;

				if (recv_buff[recv_len - 1] == '\r' || recv_buff[recv_len - 1] == '\n') 
				{
					break;
				}
			}

			unsigned long time_current = GetTickCount();
			unsigned long time_delta = time_current - time_start;

			if (time_delta >= COMM_RECV_TIMEOUT) break;
		}
		recv_buff[recv_len] = '\0';

		if (recv_len > 0) 
		{
			if (recv_buff[0] == '!') 
			{
				return -1;
			}
		}

		if (strncmp(command, recv_buff, command_len - 1) == 0) 
		{
			if (recv_buff[command_len - 1] == '=') {
				int data_count = 0;

				char* p = &recv_buff[command_len];
				char* pp = NULL;

				for (int i = 0; i < data_length; i++) 
				{
					if (p[0] == '0' && p[1] == 'x') 
					{
						returned_data[i] = strtol(p+2, &pp, 16);
						data_count++;
					}
					else 
					{
						returned_data[i] = strtod(p, &pp);
						data_count++;
					}

					if (*pp == ',') 
					{
						p = pp + 1;
					}
					else 
					{
						break;
					}
				}
				return data_count;
			}
		}
		return 0;
	}
public:
	//DECLARE PARAM
	int sensor_hz_;
	std::string child_frame_;
	std::string parent_frame_;
	std::string serial_port_;
	bool m_bSingle_TF_option;
	std::string qos_reliability_;	
	bool is_grabity_enable_;
	bool is_compensation_enable_;
private:

	rclcpp::Service<interfaces::srv::ImuReset>::SharedPtr euler_angle_reset_srv_;

	bool Euler_angle_reset_callback(
		const std::shared_ptr<interfaces::srv::ImuReset::Request> request, 
		const std::shared_ptr<interfaces::srv::ImuReset::Response> response)
	{
		bool bResult = false;
		double dSend_Data[10];
		SendRecv("ra\n", dSend_Data, 10);
		bResult = true;
		response->result = bResult;
		return true;
	}
};


int main (int argc, char** argv)
{
	rclcpp::init(argc, argv);
	std::signal(SIGINT, signal_handler);
	// Create a function for when messages are to be sent.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	auto node = std::make_shared<IAHRS>();

	std::vector<std::array<double, 3>> accel_sample;
	std::vector<std::array<double, 3>> gyro_sample;
	std::array<double, 3> euler_bias = {0.0, 0.0, 0.0};
	std::vector<std::array<double, 3>> euler_sample;

	rclcpp::Time compensation_start_time;
	bool is_compensation_finished = false;

	std::array<double, 3> accel_bias = {0.0, 0.0, 0.0};
	std::array<double, 3> gyro_bias = {0.0 ,0.0 ,0.0};

	std::array<double, 3> compensated_accel = {0.0 ,0.0 ,0.0};
	std::array<double, 3> compensated_gyro = {0.0 ,0.0 ,0.0};

	std::array<double, 3> accel_variance = { 0.0, 0.0, 0.0};
	std::array<double, 3> gyro_variance = { 0.0, 0.0, 0.0};

	// These values do not need to be converted
	node->imu_data_msg.linear_acceleration_covariance[0] = 0.0064;
	node->imu_data_msg.linear_acceleration_covariance[4] = 0.0063;
	node->imu_data_msg.linear_acceleration_covariance[8] = 0.0064;
	node->imu_data_msg.angular_velocity_covariance[0] = 0.032*(M_PI/180.0);
	node->imu_data_msg.angular_velocity_covariance[4] = 0.028*(M_PI/180.0);
	node->imu_data_msg.angular_velocity_covariance[8] = 0.006*(M_PI/180.0);
	node->imu_data_msg.orientation_covariance[0] = 0.013*(M_PI/180.0);
	node->imu_data_msg.orientation_covariance[4] = 0.011*(M_PI/180.0);
	node->imu_data_msg.orientation_covariance[8] = 0.006*(M_PI/180.0);

	rclcpp::WallRate loop_rate(node->sensor_hz_);
	node->serial_open();
	node->SendRecv("za\n", dSend_Data, 10);	// Euler Angle -> '0.0' Reset
	usleep(10000);
	printf("                       | Z axis \n");
	printf("                       | \n");
	printf("                       |   / X axis \n");
	printf("                   ____|__/____ \n");
	printf("      Y axis     / *   | /    /| \n");
	printf("      _________ /______|/    // \n");
	printf("               /___________ // \n");
	printf("              |____iahrs___|/ \n");

	while(rclcpp::ok() && !stop_requested)
		{
		rclcpp::spin_some(node);
		if(node->is_compensation_enable_ && !is_compensation_finished)
		{
			if(accel_sample.empty())
			{
				compensation_start_time = node->now();
			}
			rclcpp::Duration elapsed = node->now() - compensation_start_time;
			if (elapsed.seconds() < 10.0)
			{
				double data[10];
				int a_cnt = node->SendRecv("a\n", data, 10);
				if(a_cnt>=3)
				{
					accel_sample.push_back({data[0], data[1], data[2]});
				}
				int g_cnt = node->SendRecv("g\n", data, 10);
				if(g_cnt >= 3)
				{
					gyro_sample.push_back({data[0], data[1], data[2]});
				}
				int e_cnt = node->SendRecv("e\n", data, 10);
				if (e_cnt >= 3) {
					euler_sample.push_back({data[0], data[1], data[2]});
				}
			}
			else
			{
				tf2::Quaternion q;
				q.setRPY(_pIMU_data.dEuler_angle_Roll, _pIMU_data.dEuler_angle_Pitch, _pIMU_data.dEuler_angle_Yaw);
				tf2::Matrix3x3 rot(q);
				tf2::Vector3 g_world(0,0,gravity_accelation);
				tf2::Vector3 g_imu = rot.inverse() *g_world;
				std::array<double, 3> accel_sum = {0.0, 0.0, 0.0};
				std::array<double, 3> gyro_sum = {0.0, 0.0, 0.0};
				std::array<double, 3> average_accel = {0.0, 0.0, 0.0};
				std::array<double, 3> average_gyro = {0.0, 0.0, 0.0};
				
				for(const auto& a : accel_sample)
				{
					accel_sum[0] += a[0];
					accel_sum[1] += a[1];
					accel_sum[2] += a[2];
				}
				for(const auto& g : gyro_sample)
				{
					gyro_sum[0] += g[0];
					gyro_sum[1] += g[1];
					gyro_sum[2] += g[2];
				}
				size_t accel_size = accel_sample.size();
				size_t gyro_size = gyro_sample.size();
				for(int i = 0; i < 3; i++)
				{
					average_accel[i] = accel_sum[i]/accel_size;
					average_gyro[i] = gyro_sum[i]/gyro_size;
				}

				for(const auto& a : accel_sample)
				{
					accel_variance[0] += std::pow(a[0] - average_accel[0],2);
					accel_variance[1] += std::pow(a[1] - average_accel[1],2);
					accel_variance[2] += std::pow(a[2] - average_accel[2],2);
				}

				for(const auto& g : gyro_sample)
				{
					gyro_variance[0] += std::pow(g[0] - average_gyro[0],2);
					gyro_variance[1] += std::pow(g[1] - average_gyro[1],2);
					gyro_variance[2] += std::pow(g[2] - average_gyro[2],2);
				}

				for(int i = 0; i < 3; i++)
				{
					accel_variance[i] /= accel_size;
					gyro_variance[i] /= gyro_size;
					accel_bias[i] = average_accel[i] - g_imu[i];
					gyro_bias[i] = average_gyro[i];
				}
				// Euler angle bias compensation
				std::array<double, 3> euler_sum = {0.0, 0.0, 0.0};
				for (const auto& e : euler_sample) {
					euler_sum[0] += e[0];
					euler_sum[1] += e[1];
					euler_sum[2] += e[2];
				}
				for (int i = 0; i < 3; ++i) {
					euler_bias[i] = (euler_sum[i] / euler_sample.size()) * (M_PI / 180.0);
				}
				is_compensation_finished = true;
			}
		}
		
		if (serial_fd >= 0) 
		{
			const int max_data = 10;
			double data[max_data];
			int no_data = 0;
			no_data = node->SendRecv("g\n", data, max_data);	// Read angular_velocity _ wx, wy, wz 
			if (no_data >= 3) 
			{
				if(node->is_compensation_enable_)
				{
					if(is_compensation_finished)
					{
						node->imu_data_msg.angular_velocity.x = _pIMU_data.dAngular_velocity_x = (data[0] - gyro_bias[0]) * (M_PI/180.0);
				    node->imu_data_msg.angular_velocity.y = _pIMU_data.dAngular_velocity_y = (data[1] - gyro_bias[1]) * (M_PI/180.0);
				    node->imu_data_msg.angular_velocity.z = _pIMU_data.dAngular_velocity_z = (data[2] - gyro_bias[2]) * (M_PI/180.0);
					}
				}
				else	// angular_velocity
				{
				    node->imu_data_msg.angular_velocity.x = _pIMU_data.dAngular_velocity_x = data[0] * (M_PI/180.0);
				    node->imu_data_msg.angular_velocity.y = _pIMU_data.dAngular_velocity_y = data[1] * (M_PI/180.0);
				    node->imu_data_msg.angular_velocity.z = _pIMU_data.dAngular_velocity_z = data[2] * (M_PI/180.0);
				}
			}
			no_data = node->SendRecv("a\n", data, max_data);	// Read linear_acceleration 	unit: m/s^2
			if (no_data >= 3) 
			{
				//// linear_acceleration   g to m/s^2
				if (node->is_compensation_enable_)
				{
					if(is_compensation_finished)
					{
						node->imu_data_msg.linear_acceleration.x = _pIMU_data.dLinear_acceleration_x = data[0] - accel_bias[0];
				    node->imu_data_msg.linear_acceleration.y = _pIMU_data.dLinear_acceleration_y = data[1] - accel_bias[1];
				    node->imu_data_msg.linear_acceleration.z = _pIMU_data.dLinear_acceleration_z = data[2] - accel_bias[2];
					}
				}
				else
				{
				    node->imu_data_msg.linear_acceleration.x = _pIMU_data.dLinear_acceleration_x = data[0];
				    node->imu_data_msg.linear_acceleration.y = _pIMU_data.dLinear_acceleration_y = data[1];
				    node->imu_data_msg.linear_acceleration.z = _pIMU_data.dLinear_acceleration_z = data[2];
				}
			}
			no_data = node->SendRecv("e\n", data, max_data);	// Read Euler angle
			if (no_data >= 3)
			{
				double roll  = data[0]*(M_PI/180.0);
				double pitch = data[1]*(M_PI/180.0);
				double yaw   = data[2]*(M_PI/180.0);
				
				if (node->is_compensation_enable_ && is_compensation_finished) {
					roll  = roll -euler_bias[0];
					pitch = pitch - euler_bias[1];
					yaw   = yaw - euler_bias[2];
				}

				_pIMU_data.dEuler_angle_Roll  = roll;
				_pIMU_data.dEuler_angle_Pitch = pitch;
				_pIMU_data.dEuler_angle_Yaw   = yaw;
			}

			tf2::Quaternion q;
			q.setRPY(_pIMU_data.dEuler_angle_Roll , _pIMU_data.dEuler_angle_Pitch, _pIMU_data.dEuler_angle_Yaw);
			// orientation
			node->imu_data_msg.orientation.x = q.x();
			node->imu_data_msg.orientation.y = q.y();
			node->imu_data_msg.orientation.z = q.z();
			node->imu_data_msg.orientation.w = q.w();
			node->imu_data_msg.header.stamp = node->now();
			node->imu_data_msg.header.frame_id = node->child_frame_;

			// publish the IMU data
			node->imu_data_pub->publish(node->imu_data_msg);
			// 보정 완료 시 variance 발행
			if (is_compensation_finished) 
			{
					std_msgs::msg::Float64MultiArray variance_msg;
					variance_msg.data.insert(variance_msg.data.end(), accel_variance.begin(), accel_variance.end());
					variance_msg.data.insert(variance_msg.data.end(), gyro_variance.begin(), gyro_variance.end());
					node->variance_pub->publish(variance_msg);
			}

			if(node->m_bSingle_TF_option)
			{
				// Update the timestamp of the transform
				node->transformStamped.header.stamp = node->now();
				node->transformStamped.header.frame_id = node->parent_frame_;   // Parent frame ID
				node->transformStamped.child_frame_id = node->child_frame_;       // IMU frame ID
				// Set the transformation translation (position)
				node->transformStamped.transform.translation.x = 0.0;
				node->transformStamped.transform.translation.y = 0.0;
				node->transformStamped.transform.translation.z = 0.2;
				node->transformStamped.transform.rotation.x = q.x();
				node->transformStamped.transform.rotation.y = q.y();
				node->transformStamped.transform.rotation.z = q.z();
				node->transformStamped.transform.rotation.w = q.w();
				// Publish the transform
				node->tf_broadcaster->sendTransform(node->transformStamped);
			}
		}
        loop_rate.sleep();
    }

	close (serial_fd);
	rclcpp::shutdown();

    return 0;
}
