#include <ros/ros.h>
#include <cmath>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "controller_manager/controller_manager.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <dynamic_reconfigure/server.h>
#include <bbot_bringup/LQR_VARConfig.h>

// Control table addresses for Wheels (XM430-W210)
#define WHEEL_ADDR_TORQUE_ENABLE    64
#define WHEEL_ADDR_OPERATION_MODE   11   // PWM CONTROL MODE
#define WHEEL_ADDR_GOAL_PWM    		100
#define WHEEL_ADDR_PRESENT_VELOCITY 128
#define WHEEL_ADDR_DRIVE_MODE 		10
#define WHEEL_ADDR_CURRENT_LIMIT 	38

// Control table addresses for Legs (MX-106)
#define LEG_ADDR_TORQUE_ENABLE   	64
#define LEG_ADDR_OPERATION_MODE   	11
#define LEG_ADDR_PRESENT_POSITION 	132
#define LEG_ADDR_GOAL_POSITION 		116
#define LEG_ADDR_POSITION_KP 		84
#define LEG_ADDR_POSITION_KI 		82

// Protocol version
#define PROTOCOL_VERSION      		2.0             	// Default Protocol version of DYNAMIXEL X series.

// Default setting

#define BAUDRATE              		4500000          	// 4.5Mbps
#define DEVICE_NAME           		"/dev/ttyUSB0" 		// Port for communication with dynamixels

dynamixel::PortHandler * portHandler;					//Manage communication with dynamixels via USB
dynamixel::PacketHandler * packetHandler;				//Access dynamixels EEPROM and RAM for reading and writing

double lqr_reconfigure = 0.0;

/////////////////////////////////////////////////////////////////////
//	BBOT HARDWARE INTERFACE
/////////////////////////////////////////////////////////////////////

class BbotHardware : public hardware_interface::RobotHW{
private: //Variables
	//Dynamixels Setup vars
	std::string joint_names[6] = {"right_bottom_leg2wheel","left_bottom_leg2wheel", "base2right_top_leg", "right_top_leg2bottom_leg", "base2left_top_leg", "left_top_leg2bottom_leg"};
	uint8_t joint_IDs[6] = {12, 22, 10, 11, 20, 21}; //R_WHELL, L_WHEEL, R_TOP_LEG, R_BOTTOM_LEG, L_TOP_LEG, L_BOTTOM_LEG
	uint8_t dxl_error = 0;
	int dxl_commW_result = COMM_TX_FAIL;
	int dxl_commR_result = COMM_TX_FAIL;
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
	bool initLegs = false;

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
			if(i%2==0) 	cmd[i] = 0.64; // Initial pose for a top leg
			else 		cmd[i] = 1.65; // Initial pose for a bottom leg	
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
			
			dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_OPERATION_MODE, 16, &dxl_error); // TurnON Current control Mode
			if (dxl_comm_result != COMM_SUCCESS){ROS_ERROR("Failed to set Current control Mode for Dynamixel ID %d", joint_IDs[i]);}

			dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_CURRENT_LIMIT, 1193, &dxl_error); // TurnON Current control Mode
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
		int16_t pwm, position;

		// Write wheels pwm
		for(size_t i=0; i< 2; i++){
			pwm = (int16_t) std::round(lqr_reconfigure*cmd[i]);
			// ROS_INFO("lqr_reconfigure: %f cmd:  %f value: %f int: %d", lqr_reconfigure, cmd[i], lqr_reconfigure*cmd[i], pwm);
			
			dxl_commW_result = packetHandler->write2ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_GOAL_PWM, pwm, &dxl_error);
			if (dxl_commW_result != COMM_SUCCESS) {ROS_ERROR("Failed to write pwm for Dynamixel ID %d", joint_IDs[i]);}
		}

		if(!initLegs){
			for(size_t i=2; i< 6; i++){
				if(joint_IDs[i]%2==0)
					position = (int32_t) (2048+cmd[i]/1.18*762);  //Convert angular pos to raw data for TOP dynamixels
				else
					position = (int32_t) (4095-cmd[i]/1.67*1095); //Convert angular pos to raw data for BOTTOM dynamixels
				
				dxl_commW_result = packetHandler->write4ByteTxRx(portHandler, joint_IDs[i], LEG_ADDR_GOAL_POSITION, position, &dxl_error);
				if (dxl_commW_result != COMM_SUCCESS) {ROS_ERROR("Failed to write position for Dynamixel ID %d", joint_IDs[i]);}
			}
			initLegs = true;
		}
	}

	void read(const ros::Time &time){
		int32_t dxl_pos, dxl_vel;

		// Read wheels states
		for(int i=0; i< 2; i++){
			dxl_commR_result = packetHandler->read4ByteTxRx(portHandler, joint_IDs[i], WHEEL_ADDR_PRESENT_VELOCITY, (uint32_t *)&dxl_vel, &dxl_error);
			if (dxl_commR_result != COMM_SUCCESS) {ROS_ERROR("Failed to read position for Dynamixel ID %d", joint_IDs[i]);}
			vel[i] = (double) (dxl_vel)*0.229*(3.14159/30); 	//Convert raw data to rad/s
			// ROS_INFO("W%d = %f", i, vel[i]);
			// pos[i] = (double) dxl_pos*0.00153398;			//Convert raw data to rad
		}
		
		//TODO Read leg states
		// for(size_t i=2; i< 6; i++){
		// 	dxl_commR_result = packetHandler->read4ByteTxRx(portHandler, joint_IDs[i], LEG_ADDR_PRESENT_POSITION, (uint32_t *)&dxl_pos, &dxl_error);
		// 	if (dxl_commR_result != COMM_SUCCESS) {ROS_ERROR("Failed to read position for Dynamixel ID %d", joint_IDs[i]);}
		// 	if(joint_IDs[i]%2==0) 
		// 		pos[i] = (double) ((dxl_pos-2048)*1.17/762); 	//Convert angular pos to raw data for TOP dynamixels
		// 	else
		// 		pos[i] = (double) (-1*(4095-dxl_pos)*1.57/1024); 	//Convert angular pos to raw data for BOTTOM dynamixels
		// }

		//Debug
		// ROS_INFO("READING:\n\tRIGHT_WHEEL_VEL: %f\n\tLEFT_WHEEL_VEL: %f", vel[RIGHT_WHEEL],vel[LEFT_WHEEL]);
	 	// ROS_INFO("READING:\n\tRIGHT_WHEEL_POS: %f\n\tLEFT_WHEEL_POS: %f", pos[RIGHT_WHEEL],pos[LEFT_WHEEL]);
  	}

};

/////////////////////////////////////////////////////////////////////
//	DYNAMIC RECONFIGURE SETUP
/////////////////////////////////////////////////////////////////////
void callback(const bbot_bringup::LQR_VARConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %f", config.lqr_variable);
	lqr_reconfigure = config.lqr_variable;
}

/////////////////////////////////////////////////////////////////////
//	MAIN CONTROL LOOP
/////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
	ros::init(argc, argv, "bbot_control");
	uint8_t dxl_error = 0;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh;
	BbotHardware bbot_hardware(nh);
	controller_manager::ControllerManager cm(&bbot_hardware, nh);

	ros::Time last_time = ros::Time::now();

	dynamic_reconfigure::Server<bbot_bringup::LQR_VARConfig> server;
	dynamic_reconfigure::Server<bbot_bringup::LQR_VARConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);
	ros::Time time2;

	ros::Time tim;
	ros::Duration dur;

	while (ros::ok()) {
		ros::Time time = ros::Time::now();
		bbot_hardware.read(time);
		
		tim = ros::Time::now();
		cm.update(ros::Time::now(), time - last_time);

		while(ros::Time::now().toSec() - time2.toSec() < 0.0125){
			continue;
		}

		// ROS_INFO("TimesSleep: %f", ros::Time::now().toSec() - time2.toSec());
		time2 = ros::Time::now();
		bbot_hardware.write(time);

		last_time = time;
	}
	packetHandler->write1ByteTxRx(portHandler, 12, WHEEL_ADDR_TORQUE_ENABLE, 0, &dxl_error); //TurnOFF Torque in order to acces the EEPROM area
	packetHandler->write1ByteTxRx(portHandler, 22, WHEEL_ADDR_TORQUE_ENABLE, 0, &dxl_error); //TurnOFF Torque in order to acces the EEPROM area

	return 0;
}