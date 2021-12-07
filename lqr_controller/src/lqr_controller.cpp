#include "ros/ros.h"
#include <Eigen/Dense>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"
#include <vector>
#include <deque>

namespace lqr_controller{

class LQRController : public controller_interface::Controller<hardware_interface::EffortJointInterface>{

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n){
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
    if (!n.getParam("balance_angle_offset", balance_angle_offset)){
      ROS_ERROR("Could not find balance_angle_offset");
      return false;
    }
    if (!n.getParam("imu_topic", imu_topic)){
      ROS_ERROR("Could not find imu_topic");
      return false;
    }

    // Filter
    for(int i=0; i< filter_size; i++){
      Lpast_inputs.push_front(0.0);
    }
    for(int i=0; i< filter_size; i++){
      Rpast_inputs.push_front(0.0);
    }

    // Set LQR Matrices
    Klqr.row(0) <<  K_matrix[0][0],  K_matrix[0][1],  K_matrix[0][2],  K_matrix[0][3],  K_matrix[0][4],  K_matrix[0][5], K_matrix[0][6],  K_matrix[0][7];
    Klqr.row(1) << K_matrix[1][0],  K_matrix[1][1],  K_matrix[1][2],  K_matrix[1][3],  K_matrix[1][4],  K_matrix[1][5], K_matrix[1][6],  K_matrix[1][7];
    
    Kp = Klqr.leftCols(6);
    Ki = Klqr.rightCols(2);
    
    system_inputs.col(0) << 0.0, 0.0;
    system_inputs_filtered.col(0) << 0.0, 0.0;
    system_P_inputs = system_inputs;
    system_I_inputs = system_inputs;

    //Set KALMAN FILTER Matrices
    Amodelo.row(0) << 9.16290516e-01,  3.75189470e-03,  0.00000000e+00, -6.76620370e-02,  2.07746011e-06,  2.07746011e-06;
    Amodelo.row(1) << 6.51498644e-01,  9.74185042e-01,  0.00000000e+00, 1.06890914e+00, -1.61685676e-05, -1.61685676e-05;
    Amodelo.row(2) << 0.00000000e+00,  0.00000000e+00,  9.48756646e-01, 0.00000000e+00, -1.77740313e-05,  1.77740313e-05;
    Amodelo.row(3) << 4.15071057e-03,  1.23208015e-02,  0.00000000e+00, 1.00675997e+00, -1.03010260e-07, -1.03010260e-07;
    Amodelo.row(4) << 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 0.00000000e+00,  8.44400000e-01,  0.00000000e+00;
    Amodelo.row(5) << 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 0.00000000e+00,  0.00000000e+00,  8.44400000e-01;

    Bmodelo.col(0) << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    Bmodelo.col(1) << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    Cmodelo.row(0) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Cmodelo.row(1) << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    Cmodelo.row(2) << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    Cmodelo.row(3) << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;

    estimated_states.col(0) << 0.0, 0.0, 0.0, 0.17, 0.0, 0.0;
    final_states.col(0) << 0.0, 0.0, 0.0, 0.17, 0.0, 0.0, 0.0, 0.0;

    Kfgain.row(0) <<      0.49117022186851,   0.00375288430151178,  5.75583289487367e-21,  -0.0553164886343522;
    Kfgain.row(1) <<     0.341579996864384,     0.974178476465656, -3.56113661413267e-20,    0.770255999843656;
    Kfgain.row(3) << -6.77887436684277e-22,  1.02673453612981e-21,     0.515319204083695,  7.0834524263889e-22;
    Kfgain.row(3) <<  -0.00467372852741737,    0.0123228877384466,  3.10770302107752e-22,    0.729644361591903;
    Kfgain.row(4) <<   2.61797996435082e-6, -2.73720031788039e-10,  -2.22738014446146e-5, -1.05654235493194e-8;
    Kfgain.row(5) <<   2.61797996427498e-6, -2.73720044282855e-10,    2.2273801446606e-5, -1.05654235395279e-8;

    KfA = Amodelo - Kfgain*Cmodelo;
    KfB.leftCols(2) = Bmodelo;
    KfB.rightCols(4) = Kfgain;
    kalman_inputs.col(0) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // get the joint object to use in the realtime loop
    left_wheel_joint_ = hw->getHandle(left_wheel);    // throws on failure
    right_wheel_joint_ = hw->getHandle(right_wheel);  // throws on failure 

    // set ROS interface topics
    imu_msgs_ = n.subscribe<sensor_msgs::Imu>(imu_topic, 1, &LQRController::imu_msgsCallback, this);
    cmd_vel_ = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &LQRController::cmd_velCallback, this);                    
    
    // set debug topics
    bbot_states_pub_ = n.advertise<std_msgs::Float64MultiArray>("robot_states",1);
    bbot_est_states_pub_ = n.advertise<std_msgs::Float64MultiArray>("est_robot_states",1);
    bbot_inputs_pub_ = n.advertise<std_msgs::Float64MultiArray>("robot_inputs",1);

    // get current time
    last_pub_time = ros::Time::now().toSec();
    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period){
    if(time.toSec() - last_pub_time >= control_period){           //Keep commands at 80 Hz 
      double left_wheel_vel = left_wheel_joint_.getVelocity();    // rad/s
      double right_wheel_vel = right_wheel_joint_.getVelocity();  // rad/s

      double robot_x_velocity = wheel_radius/2*(left_wheel_vel + right_wheel_vel);                  // r/2(thetaL + thetaR)
      double robot_yaw_velocity = wheel_radius/wheel_separation*(right_wheel_vel - left_wheel_vel); // r/d(thetaL - thetaR)
      double x_windup_limit = 100.0;
      double yaw_windup_limit = 5000.0;
      double input_limit = 885;

      x_vel_int_error += x_ref - robot_x_velocity;   // get integral of the error
      yaw_vel_int_error += yaw_ref - yaw_vel;        // get integral of the error

      // Apply anti wind-up
      if(x_vel_int_error > x_windup_limit)
        x_vel_int_error = x_windup_limit;
      else if(x_vel_int_error < -x_windup_limit)
        x_vel_int_error = -x_windup_limit;

      if(yaw_vel_int_error > yaw_windup_limit)
        yaw_vel_int_error = yaw_windup_limit;
      else if(yaw_vel_int_error < -yaw_windup_limit)
        yaw_vel_int_error = -yaw_windup_limit;

      // Current readings
      current_states.col(0) << robot_x_velocity, pitch_vel_filtered, yaw_vel, -pitch_angle - balance_angle_offset; // x_vel_int_error, yaw_vel_int_error;

      //!KALMAN FILTER Adaptative K
      // estimated_states << Amodelo*estimated_states + Bmodelo*system_inputs;
      // Ymodelo << Cmodelo*estimated_states;
      // P << Amodelo*P*Amodelo.transpose() + Vd;
      // auxiliar << Cmodelo*P*Cmodelo.transpose() + Vn;
      // auxiliar << auxiliar.inverse();
      // KF << P*Cmodelo.transpose()*auxiliar;
      // estimated_states << estimated_states + KF*(current_states - Ymodelo);
      // P << (eye - KF*Cmodelo)*P;

      //!KALMAN FILTER Fixed K
      kalman_inputs.topRows(2) = system_inputs; // system_inputs_filtered;
      kalman_inputs.bottomRows(4) = current_states;
      estimated_states = KfA*estimated_states + KfB*kalman_inputs;
      final_states.topRows(6) = estimated_states;
      final_states(6,0) = x_vel_int_error;
      final_states(7,0) = yaw_vel_int_error;

      // Perform LQR calculation
      system_P_inputs = -Kp*estimated_states;
      system_I_inputs = -Ki*final_states.bottomRows(2);
      system_inputs = system_P_inputs + system_I_inputs;

      // Control FILTER
      Lpast_inputs.push_front(system_inputs(0,0));
      Lpast_inputs.pop_back();
      Rpast_inputs.push_front(system_inputs(1,0));
      Rpast_inputs.pop_back();
      double filtered_valueL = 0.0;
      double filtered_valueR = 0.0;
      for(int i=0;i< filter_size;i++){
        filtered_valueL += Lpast_inputs[i];
      }
      for(int i=0;i< filter_size;i++){
        filtered_valueR += Rpast_inputs[i];
      }
      system_inputs_filtered.col(0) << filtered_valueL/filter_size, filtered_valueR/filter_size;
      // system_inputs_filtered(0,0) = 0.1*system_inputs(0,0) + 0.9*system_inputs_filtered(0,0); 
      // system_inputs_filtered(1,0) = 0.1*system_inputs(1,0) + 0.9*system_inputs_filtered(1,0); 

      // Pub states
      std_msgs::Float64MultiArray states_msg, inputs_msg, est_states_msg;
      std::vector<double> states = {robot_x_velocity, pitch_vel_filtered, yaw_vel, -pitch_angle - balance_angle_offset, x_vel_int_error, yaw_vel_int_error};
      std::vector<double> est_states = {estimated_states.coeff(0,0),estimated_states.coeff(1,0),estimated_states.coeff(2,0),estimated_states.coeff(3,0)};
      
      states_msg.data = states;
      est_states_msg.data = est_states;
      
      bbot_est_states_pub_.publish(est_states_msg);
      bbot_states_pub_.publish(states_msg);

      // std::vector<double> system_inputs = {0.0, 0.0};
      // for(int i=0;i<2;i++){
      //   for(int j=0;j<6;j++){ //TODO WITHOUT INTEGRAL ACTION
      //     system_inputs[i] += Klqr.coeff(i,j)*current_states.coeff(j,0);
      //   }
      // }

      // Apply Saturation to the inputs
      for(int i=0;i<2;i++){
        if(system_inputs_filtered(i,0) > input_limit)
          system_inputs_filtered(i,0) = input_limit;
        else if(system_inputs_filtered(i,0) < -input_limit)
          system_inputs_filtered(i,0) = -input_limit;
      }

      left_wheel_joint_.setCommand(system_inputs_filtered.coeff(0,0));
      right_wheel_joint_.setCommand(system_inputs_filtered.coeff(1,0));
      std::vector<double> pub_inputs = {system_inputs_filtered.coeff(0,0), system_inputs_filtered.coeff(1,0)};
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
    pitch_vel_filtered = 0.4*pitch_vel + 0.6*pitch_vel_filtered;
    yaw_vel = data->angular_velocity.z;

    // pitch_angle_filtered = 0.4*pitch_angle + 0.6*pitch_angle_filtered;
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
    // LQR Matrices
    Eigen::Matrix<double, 2, 8> Klqr;
    Eigen::Matrix<double, 2, 6> Kp;
    Eigen::Matrix<double, 2, 2> Ki;
    Eigen::Matrix<double, 4, 1> current_states;
    Eigen::Matrix<double, 2, 1> system_inputs;
    Eigen::Matrix<double, 2, 1> system_inputs_filtered;
    Eigen::Matrix<double, 2, 1> system_P_inputs;
    Eigen::Matrix<double, 2, 1> system_I_inputs;

    // Kalman Filter matrices
    Eigen::Matrix<double, 6, 6> Amodelo;
    Eigen::Matrix<double, 6, 2> Bmodelo;
    Eigen::Matrix<double, 4, 6> Cmodelo;
    Eigen::Matrix<double, 6, 1> estimated_states;
    Eigen::Matrix<double, 8, 1> final_states;

    Eigen::Matrix<double, 6, 4> Kfgain;
    Eigen::Matrix<double, 6, 6> KfA;
    Eigen::Matrix<double, 6, 6> KfB;
    Eigen::Matrix<double, 6, 1> kalman_inputs;

    std::vector<std::vector<double>> K_matrix = std::vector<std::vector<double>>(2,std::vector<double>(8, 0.0));

    double roll_angle = 0.0;
    double pitch_angle = 0.0;
    double pitch_angle_filtered = 0.0;
    double yaw_angle = 0.0;

    double roll_vel = 0.0;
    double pitch_vel = 0.0;
    double pitch_vel_filtered = 0.0;
    double yaw_vel = 0.0;

    double x_vel_int_error = 0.0;
    double yaw_vel_int_error = 0.0;

    double x_ref = 0.0;
    double yaw_ref = 0.0;

    double pitch_filtered = 0.0;

    // Filters
    int filter_size = 32;
    std::deque<double> Rpast_inputs;
    std::deque<double> Lpast_inputs;

    ros::Subscriber imu_msgs_;
    ros::Subscriber cmd_vel_;
    ros::Publisher bbot_states_pub_;
    ros::Publisher bbot_est_states_pub_;
    ros::Publisher bbot_inputs_pub_;
};

PLUGINLIB_EXPORT_CLASS(lqr_controller::LQRController, controller_interface::ControllerBase);
}