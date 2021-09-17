#include "ros/ros.h"
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Twist.h"

namespace lqr_controller{

class LQRController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  {
    // get joint name from the parameter server
    std::string left_wheel, right_wheel;
    if (!n.getParam("left_wheel", left_wheel)){
      ROS_ERROR("Could not find left_wheel joint name");
      return false;
    }
    if (!n.getParam("right_wheel", right_wheel)){
      ROS_ERROR("Could not find right_wheel joint name");
      return false;
    }

    // get the joint object to use in the realtime loop
    left_wheel_joint_ = hw->getHandle(left_wheel);  // throws on failure
    right_wheel_joint_ = hw->getHandle(right_wheel);  // throws on failure

    // double Kp[2][4] = {-1.72736044378686, -0.830134477480645, -0.122691884068007, -4.31056054310816;
    //                   -1.72736044378686, -0.830134477480645, 0.122691884068007, -4.31056054310816};

    // double Ki[2][2] = { 0.0213273361882235, 0.0209403216014921;
    //                     0.0213273361882235, -0.0209403216014921};    

    imu_msgs_ = n.subscribe<sensor_msgs::Imu>("/bbot/imu", 1, &LQRController::imu_msgsCallback, this);

    cmd_vel_ = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &LQRController::cmd_velCallback, this);                    
    
    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    double left_wheel_vel = left_wheel_joint_.getVelocity();
    double right_wheel_vel = right_wheel_joint_.getVelocity();

    double robot_x_velocity = 0.05/2*(left_wheel_vel + right_wheel_vel); // r/2(thetaL + thetaR)
    double robot_yaw_velocity = 0.05/0.1431*(left_wheel_vel - right_wheel_vel); // r/d(thetaL - thetaR)

    x_vel_int_error += x_ref - robot_x_velocity;   // get integral of the error
    yaw_vel_int_error += yaw_ref - robot_yaw_velocity; // get integral of the error

    double states[6] = {robot_x_velocity, pitch_vel, robot_yaw_velocity, pitch_angle, x_vel_int_error, yaw_vel_int_error};
    // double errors[2] = {x_vel_int_error, yaw_vel_int_error};

    // Perform matrix multiplication
    double system_inputs[2] = {0.0, 0.0};
    for(int i=0;i<2;i++){
      for(int j=0;j<6;j++){
        system_inputs[i] += K[i][j]*states[j];
      }
    }

    // double integral_part[2] = {0.0, 0.0};
    // for(int i=0;i<2;i++){
    //   for(int j=0;j<2;j++){
    //     integral_part[i] += Kp[i][j]*errors[j];
    //   }
    // }

    left_wheel_joint_.setCommand(system_inputs[0]);   //send commands
    right_wheel_joint_.setCommand(system_inputs[1]);  //send commands
  }

  // Save IMU info
  void imu_msgsCallback(const sensor_msgs::ImuConstPtr& data){
    tf::Quaternion quat_angle;

    tf::quaternionMsgToTF(data->orientation, quat_angle);

    tf::Matrix3x3(quat_angle).getRPY(roll_angle, pitch_angle, yaw_angle);
    roll_vel = data->angular_velocity.x;
    pitch_vel = data->angular_velocity.y;
    yaw_vel = data->angular_velocity.z;
  }

  // Save user commands info
  void cmd_velCallback(const geometry_msgs::TwistConstPtr& data){
    x_ref = data->linear.x;
    yaw_ref = data->angular.z;
  }

  void starting(const ros::Time& time) { }
  void stopping(const ros::Time& time) { }

  private:
    hardware_interface::JointHandle left_wheel_joint_;
    hardware_interface::JointHandle right_wheel_joint_;

    double roll_angle = 0.0;
    double pitch_angle = 0.0;
    double yaw_angle = 0.0;

    double roll_vel = 0.0;
    double pitch_vel = 0.0;
    double yaw_vel = 0.0;

    double x_vel_int_error = 0.0;
    double yaw_vel_int_error = 0.0;

    double x_ref = 0.0;
    double yaw_ref = 0.0;

    ros::Subscriber imu_msgs_;
    ros::Subscriber cmd_vel_;

  // Get K matrix
    double K[2][6] = {{-1.72736044378686, -0.830134477480645, -0.122691884068007, -4.31056054310816, 0.0213273361882235, 0.0209403216014921},
                      {-1.72736044378686, -0.830134477480645, 0.122691884068007, -4.31056054310816, 0.0213273361882235, -0.0209403216014921}};
};
PLUGINLIB_EXPORT_CLASS(lqr_controller::LQRController, controller_interface::ControllerBase);
// PLUGINLIB_DECLARE_CLASS(LQRController::LQRController, controller_interface::ControllerBase);
}//namespace