#include <ros/ros.h>
#include <cmath>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "controller_manager/controller_manager.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

// Control table addresses for Wheels (XM430-W210)
#define WHEEL_ADDR_TORQUE_ENABLE    64
#define WHEEL_ADDR_OPERATION_MODE   11 //CURRENT CONTROL
#define WHEEL_ADDR_GOAL_CURRENT    102
// #define WHEEL_ADDR_PRESENT_POSITION 132
#define WHEEL_ADDR_PRESENT_VELOCITY 128
#define WHEEL_ADDR_DRIVE_MODE 		10

// Control table addresses for Legs (MX-106)
#define LEG_ADDR_TORQUE_ENABLE   	64
#define LEG_ADDR_OPERATION_MODE   	11
#define LEG_ADDR_PRESENT_POSITION 	132
#define LEG_ADDR_GOAL_POSITION 		116
#define LEG_ADDR_DRIVE_MODE 		10
#define LEG_ADDR_POSITION_KP 		84
#define LEG_ADDR_POSITION_KI 		82
#define LEG_ADDR_POSITION_KD 		80

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
	std::string joint_names[6] = {"right_bottom_leg2wheel","left_bottom_leg2wheel", "base2right_top_leg", "right_top_leg2bottom_leg", "base2left_top_leg", "left_top_leg2bottom_leg"};
	uint8_t joint_IDs[6] = {12, 22, 10, 11, 20, 21}; //R_WHELL, L_WHEEL, R_TOP_LEG, R_BOTTOM_LEG, L_TOP_LEG, L_BOTTOM_LEG
	uint8_t dxl_error = 0;
	int dxl_comm_result = COMM_TX_FAIL;
	
	//BbotHardware vars
	enum Joints{RIGHT_WHEEL, LEFT_WHEEL};
	hardware_interface::JointStateInterface joint_state_interface_;
	hardware_interface::EffortJointInterface effort_joint_interface_;
	hardware_interface::PositionJointInterface position_joint_interface_;
	double cmd[6];
	double pos[6];
	double vel[6];
	double eff[6];

public: //Methods
	BbotHardware(const ros::NodeHandle &nh) { 
		//Define the hardware interfaces for the wheel joints
		for(size_t i=0; i< 2; i++){
			cmd[i] = 0;
			pos[i] = 0;
			vel[i] = 0;
			eff[i] = 0;
			hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
			joint_state_interface_.registerHandle(joint_state_handle);

			hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd[i]);
			effort_joint_interface_.registerHandle(joint_handle);
		}
		//Define the hardware interfaces for the leg joints
		for(size_t i=2; i < 6; i++){
			if(i%2==0) 	cmd[i] = 0.6545; //0.53; // Initial pose for a top leg
			else 		cmd[i] = 1.665; //1.38; // Initial pose for a bottom leg	
			pos[i] = 0;
			vel[i] = 0;
			eff[i] = 0;
			hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
			joint_state_interface_.registerHandle(joint_state_handle);

			hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd[i]);
			position_joint_interface_.registerHandle(joint_handle);
		}
		//Register hardware interfaces of the robot
		registerInterface(&joint_state_interface_);
		registerInterface(&effort_joint_interface_);
		registerInterface(&position_joint_interface_);

		//Dynamixel setup
		portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
		packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

		if (!portHandler->openPort()) {ROS_ERROR("Failed to open the port!");}
		if (!portHandler->setBaudRate(BAUDRATE)) {ROS_ERROR("Failed to set the baudrate!");}

		//Wheels setup
		for (size_t i=0;i<2;i++){
			dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_TORQUE_ENABLE, 0, &dxl_error); //TurnOFF Torque in order to acces the EEPROM area
			if (dxl_comm_result != COMM_SUCCESS) {ROS_ERROR("Failed to disable torque for Dynamixel ID %d", joint_IDs[i]);}
			
			dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_OPERATION_MODE, 0, &dxl_error); // TurnON Current control Mode
			if (dxl_comm_result != COMM_SUCCESS){ROS_ERROR("Failed to set Current control Mode for Dynamixel ID %d", joint_IDs[i]);}
			
			if (i == RIGHT_WHEEL){dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_DRIVE_MODE, 1, &dxl_error);} // Set Right wheel to reverse mode
			
			dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_TORQUE_ENABLE, 1, &dxl_error); //TurnON Torque
			if (dxl_comm_result != COMM_SUCCESS) {ROS_ERROR("Failed to enable torque for Dynamixel ID %d", joint_IDs[i]);}
		}
		//Legs setup
		for (size_t i=2;i<6;i++){
			dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, joint_IDs[i], LEG_ADDR_TORQUE_ENABLE, 0, &dxl_error); //TurnOFF Torque in order to acces the EEPROM area
			if (dxl_comm_result != COMM_SUCCESS) {ROS_ERROR("Failed to disable torque for Dynamixel ID %d", joint_IDs[i]);}
			
			dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, joint_IDs[i], LEG_ADDR_OPERATION_MODE, 3, &dxl_error); // TurnON Velocity control Mode
			if (dxl_comm_result != COMM_SUCCESS){ROS_ERROR("Failed to set Position control Mode for Dynamixel ID %d", joint_IDs[i]);}
			
			dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, joint_IDs[i], LEG_ADDR_POSITION_KP, 300, &dxl_error); // Set position Kp
			if (dxl_comm_result != COMM_SUCCESS){ROS_ERROR("Failed to set Pos KP for Dynamixel ID %d", joint_IDs[i]);}
			
			dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, joint_IDs[i], LEG_ADDR_POSITION_KI, 100, &dxl_error); // Set position Ki
			if (dxl_comm_result != COMM_SUCCESS){ROS_ERROR("Failed to set Pos KI for Dynamixel ID %d", joint_IDs[i]);}
			
			dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, joint_IDs[i], LEG_ADDR_TORQUE_ENABLE, 1, &dxl_error); //TurnON Torque
			if (dxl_comm_result != COMM_SUCCESS) {ROS_ERROR("Failed to enable torque for Dynamixel ID %d", joint_IDs[i]);}
		}
	}

	void write(const ros::Time &time){
		int16_t current, position;

		// Write wheels current
		for(size_t i=0; i< 2; i++){
			current = (int16_t) (1193*0/3.21); //TODO () Convert torque to raw data for dynamixels current
			dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_GOAL_CURRENT, current, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS) {ROS_ERROR("Failed to write current for Dynamixel ID %d", joint_IDs[i]);}
		}

		// Write legs angular position
		for(size_t i=2; i< 6; i++){
			if(joint_IDs[i]%2==0)
				position = (int32_t) (2048+cmd[i]/1.17*762); //Convert angular pos to raw data for TOP dynamixels
			else
				position = (int32_t) (4095-cmd[i]/1.57*1024); //Convert angular pos to raw data for BOTTOM dynamixels
			
			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, joint_IDs[i], LEG_ADDR_GOAL_POSITION, position, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS) {ROS_ERROR("Failed to write position for Dynamixel ID %d", joint_IDs[i]);}
		}
	}

	void read(const ros::Time &time){
		int32_t dxl_pos, dxl_vel;
		// Read wheels states
		for(size_t i=0; i< 2; i++){
			dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_PRESENT_VELOCITY, (uint32_t *)&dxl_vel, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS) {ROS_ERROR("Failed to read velocity for Dynamixel ID %d", joint_IDs[i]);}
			// dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_PRESENT_POSITION, (uint32_t *)&dxl_pos, &dxl_error);
			// if (dxl_comm_result != COMM_SUCCESS) {ROS_ERROR("Failed to read position for Dynamixel ID %d", joint_IDs[i]);}
			vel[i] = (double) (dxl_vel); 	//Convert raw data to N.m
			// pos[i] = (double) dxl_pos*0.00153398;		//Convert raw data to rad
		}
		// Read leg states
		for(size_t i=2; i< 6; i++){
			dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, joint_IDs[i], LEG_ADDR_PRESENT_POSITION, (uint32_t *)&dxl_pos, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS) {ROS_ERROR("Failed to read position for Dynamixel ID %d", joint_IDs[i]);}
			if(joint_IDs[i]%2==0) 
				pos[i] = (double) ((dxl_pos-2048)*1.17/762); 	//Convert angular pos to raw data for TOP dynamixels
			else
				pos[i] = (double) (-1*(4095-dxl_pos)*1.57/1024); 	//Convert angular pos to raw data for BOTTOM dynamixels

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

	ros::Duration period(0.04); // 25Hz
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