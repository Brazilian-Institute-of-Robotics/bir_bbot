#include "ros/ros.h"
#include <Eigen/Dense>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"

namespace lqr_controller{

class LQRController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
  // public:
  //   LQRController():Klqr(2,6), current_states(6,1), system_inputs(2,1){}
  // Eigen::Matrix<double, 2, 6> Klqr;
  // Eigen::Matrix<double, 6, 1> current_states;
  // Eigen::Matrix<double, 2, 1> system_inputs;

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  {
    // GET parameters from Param Server
    std::string left_wheel, right_wheel;
    if (!n.getParam("left_wheel", left_wheel)){
      ROS_ERROR("Could not find left_wheel joint name");
      return false;
    }
    if (!n.getParam("right_wheel", right_wheel)){
      ROS_ERROR("Could not find right_wheel joint name");
      return false;
    }
    if (!n.getParam("wheel_separation", wheel_separation)){
      ROS_ERROR("Could not find wheel_separation");
      return false;
    }
    if (!n.getParam("wheel_radius", wheel_radius)){
      ROS_ERROR("Could not find wheel_radius");
      return false;
    }
    
    Klqr.row(0) << -0.961008304510787, -0.476939988582031, -0.370035227287944, -2.25496217761648;
    Klqr.row(1) << -0.961008304535923, -0.476939988585445,  0.370035227287944, -2.25496217765837;

    // if (!n.getParam("K_l1", K_matrix[0])){
    //   ROS_ERROR("Could not find first line of the K gain matrix");
    //   return false;
    // }
    // if (!n.getParam("K_l2", K_matrix[1])){
    //   ROS_ERROR("Could not find second line of the K gain matrix");
    //   return false;
    // }
    if (!n.getParam("control_period", control_period)){
      ROS_ERROR("Could not find control_period");
      return false;
    }
    if (!n.getParam("balance_angle_offset", balance_angle_offset)){
      ROS_ERROR("Could not find balance_angle_offset");
      return false;
    }
    if (!n.getParam("imu_topic", imu_topic)){
      ROS_ERROR("Could not find imu_topic");
      return false;
    }

    // get the joint object to use in the realtime loop
    left_wheel_joint_ = hw->getHandle(left_wheel);  // throws on failure
    right_wheel_joint_ = hw->getHandle(right_wheel);  // throws on failure 

    // set ROS interface topics
    imu_msgs_ = n.subscribe<sensor_msgs::Imu>(imu_topic, 1, &LQRController::imu_msgsCallback, this);
    cmd_vel_ = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &LQRController::cmd_velCallback, this);                    
    
    // set debug topics
    bbot_states_pub_ = n.advertise<std_msgs::Float64MultiArray>("robot_states",1);
    bbot_inputs_pub_ = n.advertise<std_msgs::Float64MultiArray>("robot_inputs",1);

    // get current time
    last_pub_time = ros::Time::now().toSec();
    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period) //Simulation period = 0.001 and Time is incremented by 0.001
  {
    if(time.toSec() - last_pub_time >= control_period){ //Keep commands at 25 Hz 
      double left_wheel_vel = left_wheel_joint_.getVelocity(); // rad/s
      double right_wheel_vel = right_wheel_joint_.getVelocity(); // rad/s

      double robot_x_velocity = wheel_radius/2*(left_wheel_vel + right_wheel_vel); // r/2(thetaL + thetaR)
      double robot_yaw_velocity = wheel_radius/wheel_separation*(right_wheel_vel - left_wheel_vel); // r/d(thetaL - thetaR)
      double windup_limit = 30.0;
      double input_limit = 1.0;

      x_vel_int_error += x_ref - robot_x_velocity;   // get integral of the error
      yaw_vel_int_error += yaw_ref - yaw_vel; // get integral of the error

      //Apply anti wind-up
      if(x_vel_int_error > windup_limit)
        x_vel_int_error = windup_limit;
      else if(x_vel_int_error < -windup_limit)
        x_vel_int_error = -windup_limit;

      if(yaw_vel_int_error > windup_limit)
        yaw_vel_int_error = windup_limit;
      else if(yaw_vel_int_error < -windup_limit)
        yaw_vel_int_error = -windup_limit;

      current_states.col(0) << robot_x_velocity, pitch_vel, yaw_vel, -pitch_angle - balance_angle_offset;// x_vel_int_error, yaw_vel_int_error;

      std_msgs::Float64MultiArray states_msg, inputs_msg;
      std::vector<double> states = {robot_x_velocity, pitch_vel, yaw_vel, -pitch_angle - balance_angle_offset, x_vel_int_error, yaw_vel_int_error};
      states_msg.data = states;
      bbot_states_pub_.publish(states_msg);

      // Perform matrix multiplication
      system_inputs << -Klqr*current_states;

      // std::vector<double> system_inputs = {0.0, 0.0};
      // for(int i=0;i<2;i++){
      //   for(int j=0;j<6;j++){ //TODO WITHOUT INTEGRAL ACTION
      //     system_inputs[i] += Klqr.coeff(i,j)*current_states.coeff(j,0);
      //   }
      // }

      // Apply Saturation to the inputs
      // for(int i=0;i<2;i++){
      //   if(system_inputs[i] > input_limit)
      //     system_inputs[i] = input_limit;
      //   else if(system_inputs[i] < -input_limit)
      //     system_inputs[i] = -input_limit;
      // }

      left_wheel_joint_.setCommand(system_inputs.coeff(0,0));
      right_wheel_joint_.setCommand(system_inputs.coeff(1,0));
      std::vector<double> pub_inputs = {system_inputs.coeff(0,0), system_inputs.coeff(1,0)};
      inputs_msg.data = pub_inputs;
      bbot_inputs_pub_.publish(inputs_msg);
      // ROS_INFO("Pub %f", time.toSec() - last_pub_time);
      last_pub_time = time.toSec();
    }
  }

  // Save IMU info
  void imu_msgsCallback(const sensor_msgs::ImuConstPtr& data){
    tf::Quaternion quat_angle;

    tf::quaternionMsgToTF(data->orientation, quat_angle);

    // Pitch and roll are inverted due to simulation issues
    tf::Matrix3x3(quat_angle).getRPY(roll_angle,pitch_angle, yaw_angle);
    roll_vel = data->angular_velocity.x;
    pitch_vel = -1*data->angular_velocity.y; //todo
    yaw_vel = data->angular_velocity.z;
  }

  // Save user commands info
  void cmd_velCallback(const geometry_msgs::TwistConstPtr& data){
    x_ref = data->linear.x;
    yaw_ref = data->angular.z;
  }

  void starting(const ros::Time& time) { }

  void stopping(const ros::Time& time) { 
      x_vel_int_error = 0.0;
      yaw_vel_int_error = 0.0;
      left_wheel_joint_.setCommand(0.0);
      right_wheel_joint_.setCommand(0.0);

  }

  private:
    hardware_interface::JointHandle left_wheel_joint_;
    hardware_interface::JointHandle right_wheel_joint_;

    double last_pub_time;
    double wheel_separation;
    double wheel_radius;
    double control_period;
    double balance_angle_offset;
    std::string imu_topic;

    Eigen::Matrix<double, 2, 4> Klqr;
    Eigen::Matrix<double, 4, 1> current_states;
    Eigen::Matrix<double, 2, 1> system_inputs;
    // std::vector<std::vector<double>> K_matrix = std::vector<std::vector<double>>(2,std::vector<double>(6, 0.0));

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

    double pitch_filtered = 0.0;

    ros::Subscriber imu_msgs_;
    ros::Subscriber cmd_vel_;
    ros::Publisher bbot_states_pub_;
    ros::Publisher bbot_inputs_pub_;

    // Get K matrix Pose C               
    // double K[2][6] = {{-0.351342726830498, -0.136414901229862, -0.0590995944246939, -1.08273414375471, 0.00177016409511773,  0.0186040640917086},
    //                   {-0.351342726830253, -0.136414901229804,  0.0590995944246846, -1.08273414375414, 0.00177016409511191, -0.0186040640917081}};
};
PLUGINLIB_EXPORT_CLASS(lqr_controller::LQRController, controller_interface::ControllerBase);
}//namespace