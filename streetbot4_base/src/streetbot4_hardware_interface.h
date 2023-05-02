
#ifndef STREETBOT4_HARDWARE_INTERFACE_HPP_
#define STREETBOT4_HARDWARE_INTERFACE_HPP_

#include <boost/assign/list_of.hpp>
#include <sstream>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/console.h>
#include <ros/ros.h>

class StreetBot4HardwareInterface : public hardware_interface::RobotHW {
public:
	StreetBot4HardwareInterface(ros::NodeHandle node, ros::NodeHandle private_node, double target_max_wheel_angular_speed);

	void updateJointsFromHardware(const ros::Duration& period);
	void writeCommandsToHardware();

private:
	ros::NodeHandle _node;
	ros::NodeHandle _private_node;

	hardware_interface::JointStateInterface _joint_state_interface;
	hardware_interface::VelocityJointInterface _velocity_joint_interface;

	ros::Subscriber _front_left_wheel_angle_sub;
	ros::Subscriber _front_right_wheel_angle_sub;
	ros::Subscriber _rear_left_wheel_angle_sub;
	ros::Subscriber _rear_right_wheel_angle_sub;

	ros::Publisher _front_left_wheel_vel_pub;
	ros::Publisher _front_right_wheel_vel_pub;
	ros::Publisher _rear_left_wheel_vel_pub;
	ros::Publisher _rear_right_wheel_vel_pub;

	struct Joint {
		double position;
		double position_offset;
		double velocity;
		double effort;
		double velocity_command;

		Joint()
			: position(0)
			, velocity(0)
			, effort(0)
			, velocity_command(0) { }
	} _joints[6];

	double _front_left_wheel_angle;
	double _front_right_wheel_angle;
	double _rear_left_wheel_angle;
	double _rear_right_wheel_angle;
	double _max_wheel_angular_speed;

	void registerControlInterfaces();
	void frontLeftWheelAngleCallback(const std_msgs::Float64& msg);
	void frontRightWheelAngleCallback(const std_msgs::Float64& msg);
	void rearLeftWheelAngleCallback(const std_msgs::Float64& msg);
	void rearRightWheelAngleCallback(const std_msgs::Float64& msg);
	void limitDifferentialSpeed(double& diff_speed_left_side, double& diff_speed_right_side);
};

StreetBot4HardwareInterface::StreetBot4HardwareInterface(ros::NodeHandle node, ros::NodeHandle private_node, double target_max_wheel_angular_speed)
	: _node(node)
	, _private_node(private_node)
	, _max_wheel_angular_speed(target_max_wheel_angular_speed) {
	registerControlInterfaces();

	_front_left_wheel_vel_pub = _node.advertise<std_msgs::Float64>("/front_left_wheel/target_velocity", 1);
	_front_right_wheel_vel_pub = _node.advertise<std_msgs::Float64>("/front_right_wheel/target_velocity", 1);
	_rear_left_wheel_vel_pub = _node.advertise<std_msgs::Float64>("/rear_left_wheel/target_velocity", 1);
	_rear_right_wheel_vel_pub = _node.advertise<std_msgs::Float64>("/rear_right_wheel/target_velocity", 1);

	_front_left_wheel_angle_sub = _node.subscribe("/front_left_wheel/angle", 1, &StreetBot4HardwareInterface::frontLeftWheelAngleCallback, this);
	_front_right_wheel_angle_sub = _node.subscribe("/front_right_wheel/angle", 1, &StreetBot4HardwareInterface::frontRightWheelAngleCallback, this);
	_rear_left_wheel_angle_sub = _node.subscribe("/rear_left_wheel/angle", 1, &StreetBot4HardwareInterface::rearLeftWheelAngleCallback, this);
	_rear_right_wheel_angle_sub = _node.subscribe("/rear_right_wheel/angle", 1, &StreetBot4HardwareInterface::rearRightWheelAngleCallback, this);
}

void StreetBot4HardwareInterface::writeCommandsToHardware() {
	double diff_angle_speed_front_left = _joints[0].velocity_command;
	double diff_angle_speed_front_right = _joints[1].velocity_command;
	double diff_angle_speed_rear_left = _joints[4].velocity_command;
	double diff_angle_speed_rear_right = _joints[5].velocity_command;

	limitDifferentialSpeed(diff_angle_speed_front_left, diff_angle_speed_front_right);
	limitDifferentialSpeed(diff_angle_speed_rear_left, diff_angle_speed_rear_right);

	std_msgs::Float64 front_left_wheel_vel_msg;
	std_msgs::Float64 front_right_wheel_vel_msg;
	std_msgs::Float64 rear_left_wheel_vel_msg;
	std_msgs::Float64 rear_right_wheel_vel_msg;

	front_left_wheel_vel_msg.data = diff_angle_speed_front_left;
	front_right_wheel_vel_msg.data = diff_angle_speed_front_right;
	rear_left_wheel_vel_msg.data = diff_angle_speed_rear_left;
	rear_right_wheel_vel_msg.data = diff_angle_speed_rear_right;

	_front_left_wheel_vel_pub.publish(front_left_wheel_vel_msg);
	_front_right_wheel_vel_pub.publish(front_right_wheel_vel_msg);
	_rear_left_wheel_vel_pub.publish(rear_left_wheel_vel_msg);
	_rear_right_wheel_vel_pub.publish(rear_right_wheel_vel_msg);
}

void StreetBot4HardwareInterface::updateJointsFromHardware(const ros::Duration& period) {
	double delta_front_left_wheel = _front_left_wheel_angle - _joints[0].position - _joints[0].position_offset;
	double delta_front_right_wheel = _front_right_wheel_angle - _joints[1].position - _joints[1].position_offset;
	double delta_rear_left_wheel = _rear_left_wheel_angle - _joints[4].position - _joints[4].position_offset;
	double delta_rear_right_wheel = _rear_right_wheel_angle - _joints[5].position - _joints[5].position_offset;

	if (std::abs(delta_front_left_wheel) < 1) {
		_joints[0].position += delta_front_left_wheel;
		_joints[0].velocity = delta_front_left_wheel / period.toSec();
	} else {
		_joints[0].position_offset += delta_front_left_wheel;
	}

	if (std::abs(delta_front_right_wheel) < 1) {
		_joints[1].position += delta_front_right_wheel;
		_joints[1].velocity = delta_front_right_wheel / period.toSec();
	} else {
		_joints[1].position_offset += delta_front_right_wheel;
	}

	if (std::abs(delta_rear_left_wheel) < 1) {
		_joints[4].position += delta_rear_left_wheel;
		_joints[4].velocity = delta_rear_left_wheel / period.toSec();
	} else {
		_joints[4].position_offset += delta_rear_left_wheel;
	}

	if (std::abs(delta_rear_right_wheel) < 1) {
		_joints[5].position += delta_rear_right_wheel;
		_joints[5].velocity = delta_rear_right_wheel / period.toSec();
	} else {
		_joints[5].position_offset += delta_rear_right_wheel;
	}

}

void StreetBot4HardwareInterface::registerControlInterfaces() {
	ros::V_string joint_names = boost::assign::list_of("base_to_front_left_wheel")("base_to_front_right_wheel")
		("base_to_rear_left_wheel")("base_to_rear_right_wheel");

	for (unsigned int i = 0; i < joint_names.size(); i++) {
		hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &_joints[i].position, &_joints[i].velocity, &_joints[i].effort);
		_joint_state_interface.registerHandle(joint_state_handle);

		hardware_interface::JointHandle joint_handle(joint_state_handle, &_joints[i].velocity_command);
		_velocity_joint_interface.registerHandle(joint_handle);
	}
	registerInterface(&_joint_state_interface);
	registerInterface(&_velocity_joint_interface);
}

void StreetBot4HardwareInterface::frontLeftWheelAngleCallback(const std_msgs::Float64& msg) {
	_front_left_wheel_angle = msg.data;
}

void StreetBot4HardwareInterface::frontRightWheelAngleCallback(const std_msgs::Float64& msg) {
	_front_right_wheel_angle = msg.data;
}

void StreetBot4HardwareInterface::rearLeftWheelAngleCallback(const std_msgs::Float64& msg) {
	_rear_left_wheel_angle = msg.data;
}

void StreetBot4HardwareInterface::rearRightWheelAngleCallback(const std_msgs::Float64& msg) {
	_rear_right_wheel_angle = msg.data;
}

void StreetBot4HardwareInterface::limitDifferentialSpeed(double& diff_speed_left_side, double& diff_speed_right_side) {
	double large_speed = std::max(std::abs(diff_speed_left_side), std::abs(diff_speed_right_side));
	if (large_speed > _max_wheel_angular_speed) {
		diff_speed_left_side *= _max_wheel_angular_speed / large_speed;
		diff_speed_right_side *= _max_wheel_angular_speed / large_speed;
	}
}

#endif // STREETBOT4_HARDWARE_INTERFACE_HPP_
