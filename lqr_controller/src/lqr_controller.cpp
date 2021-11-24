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
    // Set LQR Matrices
    Klqr.row(0) <<  -662.224947506569, -181.891378224247, -43.3637526093121, -1460.60630987986,  0.257625835152467, 0.0609893238064036;
    Klqr.row(1) <<  -662.224947506569, -181.891378224247,   43.363752609312, -1460.60630987986, 0.0609893238064036,  0.257625835152467;
    system_inputs.col(0) << 0.0, 0.0;

    //Set KALMAN FILTER Matrices
    Amodelo.row(0) << 9.16290516e-01,  3.75189470e-03,  0.00000000e+00, -6.76620370e-02,  5.53989362e-05,  5.53989362e-05;
    Amodelo.row(1) << 6.51498644e-01,  9.74185042e-01,  0.00000000e+00, 1.06890914e+00, -4.31161802e-04, -4.31161802e-04;
    Amodelo.row(2) << 0.00000000e+00,  0.00000000e+00,  9.48756646e-01, 0.00000000e+00, -4.73974169e-04,  4.73974169e-04;
    Amodelo.row(3) << 4.15071057e-03,  1.23208015e-02,  0.00000000e+00, 1.00675997e+00, -2.74694026e-06, -2.74694026e-06;
    Amodelo.row(4) << 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 0.00000000e+00,  8.44400000e-01,  0.00000000e+00;
    Amodelo.row(5) << 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 0.00000000e+00,  0.00000000e+00,  8.44400000e-01;

    Bmodelo.col(0) << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    Bmodelo.col(1) << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    Cmodelo.row(0) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Cmodelo.row(1) << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    Cmodelo.row(2) << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    Cmodelo.row(3) << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;

    // P.row(0) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // P.row(1) << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    // P.row(2) << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    // P.row(3) << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
    // P.row(4) << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    // P.row(5) << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    // P << 1e-9*P;

    // Vd.row(0) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // Vd.row(1) << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    // Vd.row(2) << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    // Vd.row(3) << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
    // Vd.row(4) << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    // Vd.row(5) << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    // Vd << 1e-9*Vd;

    // Vn.row(0) << 1.0, 0.0, 0.0, 0.0;
    // Vn.row(1) << 0.0, 1.0, 0.0, 0.0;
    // Vn.row(2) << 0.0, 0.0, 1.0, 0.0;
    // Vn.row(3) << 0.0, 0.0, 0.0, 1.0;
    // Vn << 1e-9*Vn;

    estimated_states.col(0) << 0.0, 0.0, 0.0, 0.17, 0.0, 0.0;
    // Eigen::Matrix<double, 6, 1> test; 
    // test.col(0) << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

    // std::cout << estimated_states + test;

    // eye.row(0) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // eye.row(1) << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    // eye.row(2) << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    // eye.row(3) << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
    // eye.row(4) << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    // eye.row(5) << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    // std::cout << P << std::endl;
    // std::cout << Vd << std::endl;
    // std::cout << Vn << std::endl;
    // KF2 ------------------

    Kfgain.row(0) <<  0.491170238667083,   0.00375288429980882,  4.25680378109589e-19,  -0.0553164886924014;
    Kfgain.row(1) <<  0.34157993108342,     0.974178476472573, -3.99323095071199e-19,     0.77025600011628;
    Kfgain.row(2) <<  3.06222262767131e-19, -3.55327872143084e-21,     0.515320444840736, 9.23744384792761e-20;
    Kfgain.row(3) <<  -0.00467372896955414,    0.0123228877384941,  6.98500260561216e-20,    0.729644361593931;
    Kfgain.row(4) <<  6.98127941878135e-5,  -7.29920047589147e-9, -0.000593964960553881, -2.81744532843314e-7;
    Kfgain.row(5) <<  6.98127941877348e-5,  -7.29920048221736e-9,  0.000593964960555286, -2.81744532801115e-7;

    KfA = Amodelo - Kfgain*Cmodelo;
    // KfB << Amodelo; // Just for initializing the values
    KfB.leftCols(2) = Bmodelo;
    KfB.rightCols(4) = Kfgain;
    kalman_inputs.col(0) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // std::cout << kalman_inputs << std::endl << std::endl;
    // std::cout << KfA << std::endl << std::endl;
    // std::cout << KfB << std::endl << std::endl;


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
    bbot_est_states_pub_ = n.advertise<std_msgs::Float64MultiArray>("est_robot_states",1);
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
      double input_limit = 885;

      x_vel_int_error += x_ref - robot_x_velocity;   // get integral of the error
      yaw_vel_int_error += yaw_ref - yaw_vel; // get integral of the error

      //Apply anti wind-up
      // if(x_vel_int_error > windup_limit)
      //   x_vel_int_error = windup_limit;
      // else if(x_vel_int_error < -windup_limit)
      //   x_vel_int_error = -windup_limit;

      // if(yaw_vel_int_error > windup_limit)
      //   yaw_vel_int_error = windup_limit;
      // else if(yaw_vel_int_error < -windup_limit)
      //   yaw_vel_int_error = -windup_limit;

      // Current readings
      current_states.col(0) << robot_x_velocity, pitch_vel, yaw_vel, -pitch_angle - balance_angle_offset;// x_vel_int_error, yaw_vel_int_error;


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
      kalman_inputs.topRows(2) = system_inputs;
      kalman_inputs.bottomRows(4) = current_states;
      estimated_states = KfA*estimated_states + KfB*kalman_inputs;

      // Perform LQR calculation
      system_inputs << -Klqr*estimated_states;

      // Pub states
      std_msgs::Float64MultiArray states_msg, inputs_msg, est_states_msg;
      std::vector<double> states = {robot_x_velocity, pitch_vel, yaw_vel, -pitch_angle - balance_angle_offset, x_vel_int_error, yaw_vel_int_error};
      std::vector<double> est_states = {estimated_states.coeff(0,0),estimated_states.coeff(1,0),estimated_states.coeff(2,0),estimated_states.coeff(3,0)};
      
      states_msg.data = states;
      est_states_msg.data = est_states;
      // for (auto i: est_states)
      //   std::cout << i << ' ';
      // std::cout << std::endl;
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
        if(system_inputs(i,0) > input_limit)
          system_inputs(i,0) = input_limit;
        else if(system_inputs(i,0) < -input_limit)
          system_inputs(i,0) = -input_limit;
      }

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
    // LQR Matrices
    Eigen::Matrix<double, 2, 6> Klqr;
    Eigen::Matrix<double, 4, 1> current_states;
    Eigen::Matrix<double, 2, 1> system_inputs;

    // Kalman Filter matrices
    Eigen::Matrix<double, 6, 6> Amodelo;
    Eigen::Matrix<double, 6, 2> Bmodelo;
    Eigen::Matrix<double, 4, 6> Cmodelo;
    // Eigen::Matrix<double, 10, 4> KF;
    // Eigen::Matrix<double, 6, 6> P;
    // Eigen::Matrix<double, 6, 6> Vd;
    // Eigen::Matrix<double, 4, 4> Vn;
    Eigen::Matrix<double, 6, 1> estimated_states;
    // Eigen::Matrix<double, 4, 1> Ymodelo;
    // Eigen::Matrix<double, 4, 4> auxiliar;
    // Eigen::Matrix<double, 6, 6> eye;

    Eigen::Matrix<double, 6, 4> Kfgain;
    Eigen::Matrix<double, 6, 6> KfA;
    Eigen::Matrix<double, 6, 6> KfB;
    Eigen::Matrix<double, 6, 1> kalman_inputs;

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
    ros::Publisher bbot_est_states_pub_;
    ros::Publisher bbot_inputs_pub_;

    // Get K matrix Pose C               
    // double K[2][6] = {{-0.351342726830498, -0.136414901229862, -0.0590995944246939, -1.08273414375471, 0.00177016409511773,  0.0186040640917086},
    //                   {-0.351342726830253, -0.136414901229804,  0.0590995944246846, -1.08273414375414, 0.00177016409511191, -0.0186040640917081}};
};
PLUGINLIB_EXPORT_CLASS(lqr_controller::LQRController, controller_interface::ControllerBase);
}//namespace