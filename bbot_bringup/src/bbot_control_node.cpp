#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "controller_manager/controller_manager.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

// Control table addresses for Wheels (XM430-W210)
#define WHEEL_ADDR_TORQUE_ENABLE    64
#define WHEEL_ADDR_OPERATION_MODE   11
#define WHEEL_ADDR_GOAL_VELOCITY    104
#define WHEEL_ADDR_PRESENT_POSITION 132
#define WHEEL_ADDR_PRESENT_VELOCITY 128
#define WHEEL_ADDR_DRIVE_MODE 		10

// Protocol version
#define PROTOCOL_VERSION      		2.0             	// Default Protocol version of DYNAMIXEL X series.

// Default setting
#define RIGHT_WHEEL_ID              12               	// RIGHT_WHEEL
#define LEFT_WHEEL_ID               22               	// LEFT_WHEEL
#define BAUDRATE              		1000000          	// 1Mbps
#define DEVICE_NAME           		"/dev/ttyUSB0" 		// Port for communication with dynamixels

dynamixel::PortHandler * portHandler;		//Manage communication with dynamixels via USB
dynamixel::PacketHandler * packetHandler;	//Access dynamixels EEPROM and RAM for reading and writing

/////////////////////////////////////////////////////////////////////
//	BBOT HARDWARE INTERFACE
/////////////////////////////////////////////////////////////////////

class BbotHardware : public hardware_interface::RobotHW{
private: //Variables
	//Dynamixels Setup vars
	uint8_t joint_IDs[2] = {12, 22};
	uint8_t dxl_error = 0;
	int dxl_comm_result = COMM_TX_FAIL;
	
	//BbotHardware vars
	enum Joints{RIGHT_WHEEL, LEFT_WHEEL};
	hardware_interface::JointStateInterface joint_state_interface_;
	hardware_interface::VelocityJointInterface velocity_joint_interface_;
	double cmd[2];
	double pos[2];
	double vel[2];
	double eff[2];

public: //Methods
	BbotHardware(const ros::NodeHandle &nh) { 
		std::string joint_names[2] = {"right_bottom_leg2wheel","left_bottom_leg2wheel"};
//		uint8_t joint_IDs[2] = {11, 12};
		for(size_t i=0; i< 2; i++){ //Define the hardware interfaces for each joint of the robot
			cmd[i] = 0;
			pos[i] = 0;
			vel[i] = 0;
			eff[i] = 0;
			hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
			joint_state_interface_.registerHandle(joint_state_handle);

			hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd[i]);
			velocity_joint_interface_.registerHandle(joint_handle);
		}
		//Register hardware interfaces of the robot
		registerInterface(&joint_state_interface_);
		registerInterface(&velocity_joint_interface_);

		portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
		packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

		if (!portHandler->openPort()) {ROS_ERROR("Failed to open the port!");}
		if (!portHandler->setBaudRate(BAUDRATE)) {ROS_ERROR("Failed to set the baudrate!");}

		//Wheels setup
		for (size_t i=0;i<2;i++){
			dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_TORQUE_ENABLE, 0, &dxl_error); //TurnOFF Torque in order to acces the EEPROM area
			if (dxl_comm_result != COMM_SUCCESS) {ROS_ERROR("Failed to disable torque for Dynamixel ID %d", joint_IDs[i]);}
			dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_OPERATION_MODE, 1, &dxl_error); // TurnON Velocity control Mode
			if (dxl_comm_result != COMM_SUCCESS){ROS_ERROR("Failed to set Velocity control Mode for Dynamixel ID %d", joint_IDs[i]);}
			if (i == RIGHT_WHEEL){dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_DRIVE_MODE, 1, &dxl_error);} // Set Right wheel to reverse mode
			dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_TORQUE_ENABLE, 1, &dxl_error); //TurnON Torque
			if (dxl_comm_result != COMM_SUCCESS) {ROS_ERROR("Failed to enable torque for Dynamixel ID %d", joint_IDs[i]);}
		}

	}

	void write(const ros::Time &time){
		static double dxl_vel;
		// Write wheels angular velocity
		for(size_t i=0; i< 2; i++){
			dxl_vel = cmd[i]/8.9*385; //Convert angular vel to raw data for dynamixels
			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_GOAL_VELOCITY, dxl_vel, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS) {ROS_ERROR("Failed to write velocity for Dynamixel ID %d", joint_IDs[i]);}
		}

		//Debug
		// ROS_INFO("SETTING:\n\tRIGHT_WHEEL: %f\n\tLEFT_WHEEL: %f", cmd[RIGHT_WHEEL],cmd[LEFT_WHEEL]);
	}

	void read(const ros::Time &time){
		static uint32_t dxl_pos, dxl_vel;
		for(size_t i=0; i< 2; i++){
			dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_PRESENT_VELOCITY, (uint32_t *)&dxl_vel, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS) {ROS_ERROR("Failed to read velocity for Dynamixel ID %d", joint_IDs[i]);}
			dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_PRESENT_POSITION, (uint32_t *)&dxl_pos, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS) {ROS_ERROR("Failed to read position for Dynamixel ID %d", joint_IDs[i]);}
			vel[i] = (double) dxl_vel;
			pos[i] = (double) dxl_pos;
		}

		//Debug
		// ROS_INFO("READING:\n\tRIGHT_WHEEL_VEL: %f\n\tLEFT_WHEEL_VEL: %f", vel[RIGHT_WHEEL],vel[LEFT_WHEEL]);
	 	// ROS_INFO("READING:\n\tRIGHT_WHEEL_POS: %f\n\tLEFT_WHEEL_POS: %f", pos[RIGHT_WHEEL],pos[LEFT_WHEEL]);
  	}
};

/////////////////////////////////////////////////////////////////////
//	MAIN CONTROL LOOP
/////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
	ros::init(argc, argv, "bbot_control");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh;
	BbotHardware bbot_hardware(nh);
	controller_manager::ControllerManager cm(&bbot_hardware, nh);

	ros::Duration period(0.01); // 100Hz
	ros::Time last_time = ros::Time::now();

	while (ros::ok()) {
		ros::Time time = ros::Time::now();

		bbot_hardware.read(time);
		cm.update(ros::Time::now(), time - last_time);
		bbot_hardware.write(time);

		period.sleep();
		last_time = time;
	}

	return 0;
}