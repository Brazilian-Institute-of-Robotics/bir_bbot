#include "ros/ros.h"
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
    if (!n.getParam("K_l1", K_matrix[0])){
      ROS_ERROR("Could not find first line of the K gain matrix");
      return false;
    }
    if (!n.getParam("K_l2", K_matrix[1])){
      ROS_ERROR("Could not find second line of the K gain matrix");
      return false;
    }
    if (!n.getParam("control_period", control_period)){
      ROS_ERROR("Could not find control_period");
      return false;
    }

    // get the joint object to use in the realtime loop
    left_wheel_joint_ = hw->getHandle(left_wheel);  // throws on failure
    right_wheel_joint_ = hw->getHandle(right_wheel);  // throws on failure 

    // set ROS interface topics
    imu_msgs_ = n.subscribe<sensor_msgs::Imu>("/bbot/imu", 1, &LQRController::imu_msgsCallback, this);
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
      double robot_yaw_velocity = wheel_radius/wheel_separation*(left_wheel_vel - right_wheel_vel); // r/d(thetaL - thetaR)

      x_vel_int_error += x_ref - robot_x_velocity;   // get integral of the error
      yaw_vel_int_error += yaw_ref - robot_yaw_velocity; // get integral of the error

      std::vector<double> states = {robot_x_velocity, pitch_vel, yaw_vel, pitch_angle - 0.16, x_vel_int_error, -yaw_vel_int_error};

      std_msgs::Float64MultiArray states_msg, inputs_msg;
      states_msg.data = states;
      bbot_states_pub_.publish(states_msg);

      // Perform matrix multiplication
      std::vector<double> system_inputs = {0.0, 0.0};
      for(int i=0;i<2;i++){
        for(int j=0;j<6;j++){
          system_inputs[i] += K_matrix[i][j]*states[j];
        }
      }

      //Apply Saturation to the inputs
      // for(int i=0;i<2;i++){
      //   if(system_inputs[i] > 2)
      //     system_inputs[i] = 0.5;
      //   else if(system_inputs[i] < -0.5)
      //     system_inputs[i] = -0.5;
      // }
      
      system_inputs[0] *= -1;
      system_inputs[1] *= -1;
      inputs_msg.data = system_inputs;

      left_wheel_joint_.setCommand(system_inputs[0]);
      right_wheel_joint_.setCommand(system_inputs[1]);
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
    tf::Matrix3x3(quat_angle).getRPY(pitch_angle,roll_angle, yaw_angle);
    pitch_vel = data->angular_velocity.x;
    roll_vel = data->angular_velocity.y;
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
  }

  private:
    hardware_interface::JointHandle left_wheel_joint_;
    hardware_interface::JointHandle right_wheel_joint_;

    double last_pub_time;
    double wheel_separation;
    double wheel_radius;
    double control_period;
    std::vector<std::vector<double>> K_matrix = std::vector<std::vector<double>>(2,std::vector<double>(6, 0.0));

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