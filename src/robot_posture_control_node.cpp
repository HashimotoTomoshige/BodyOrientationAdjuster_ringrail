#include "include_files.h"
#include "compute_motor_angle.h"
#include "compute_kinematics.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include "geometry_msgs/Twist.h"

class RobotPostureControl : public robot_posture_control::ComputeKinematics
{
protected:
  ros::NodeHandle nh_;
  ros::Publisher motor_position_pub_;
  ros::Publisher robot_posture_pub_;
  ros::Publisher joint_angle_pub_;
  ros::Publisher chest_angle_pub_;
  ros::Subscriber gui_sub_;
  std_msgs::Int32MultiArray motor_position_;
  std_msgs::Float32 chest_angle_msg_;
  geometry_msgs::Twist robot_posture_;
  geometry_msgs::Twist joint_angle_;

public:
  RobotPostureControl();
  void guiCallback(const geometry_msgs::Twist& msg);
  void publishRobotPosture();
  void publishJointAngle(const Eigen::VectorXd& q);
  void publishMotorPosition(const Eigen::VectorXd& q);
  void publishChestAngle();
};

RobotPostureControl::RobotPostureControl(): robot_posture_control::ComputeKinematics()
{
  motor_position_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("motor_position", 1);
  robot_posture_pub_ = nh_.advertise<geometry_msgs::Twist>("robot_posture", 1);
  joint_angle_pub_ = nh_.advertise<geometry_msgs::Twist>("joint_angle", 1);
  chest_angle_pub_ = nh_.advertise<std_msgs::Float32>("chest_angle", 1);
  gui_sub_ = nh_.subscribe("robot_pose", 1, &RobotPostureControl::guiCallback, this);
  motor_position_.data.resize(4);
}

void RobotPostureControl::guiCallback(const geometry_msgs::Twist& msg)
{
  gui_goal_(0) = msg.angular.z;
  gui_goal_(1) = msg.angular.y;
  write_flag_ = true;
}

void RobotPostureControl::publishJointAngle(const Eigen::VectorXd& q)
{
  joint_angle_.angular.z = q(0);
  joint_angle_.angular.y = q(1);
  joint_angle_.angular.x = q(2);
  joint_angle_pub_.publish(joint_angle_);
}

void RobotPostureControl::publishRobotPosture()
{
  robot_posture_.angular.z = r_(0);
  robot_posture_.angular.y = r_(1);
  robot_posture_.angular.x = r_(2);
  robot_posture_pub_.publish(robot_posture_);
}

void RobotPostureControl::publishMotorPosition(const Eigen::VectorXd& q)
{
  motor_position_.data[0] = convertWaistAngleToRotationNumber(q(0));
  motor_position_.data[1] = convertBendingAngleToRotationNumber(q(1));
  motor_position_.data[2] = convertChestAngleToRotationNumber(q(2));
  motor_position_.data[3] = convertBaseAngleToRotationNumber(q(0));
  motor_position_pub_.publish(motor_position_);
  write_flag_ = false;
}

void RobotPostureControl::publishChestAngle()
{
  chest_angle_msg_.data = chest_angle_;
  chest_angle_pub_.publish(chest_angle_msg_);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_attitude_control_pubsub_node");
  RobotPostureControl robot1;
  ros::Rate loop_rate(10);
  Eigen::VectorXd q = Eigen::VectorXd::Constant(3,0);
  Eigen::VectorXd r = Eigen::VectorXd::Constant(3,0);

  const int mode = 2; //0がGUIのプログラムから目標姿勢入力, 1がターミナルから目標姿勢入力
  
  while(ros::ok())
  {
    if(mode == 0)
    {
      if(robot1.write_flag_ ==true)
      {
        q = robot1.computePosture(robot1.inputGoalFromGui());
        robot1.publishMotorPosition(q);
        robot1.publishChestAngle();
      }
      robot1.publishJointAngle(q);
    }
    else if(mode == 1)
    {
      q = robot1.computePosture(robot1.inputGoal());
      robot1.publishMotorPosition(q);
      robot1.publishJointAngle(q);
      robot1.publishChestAngle();
    }
    else if(mode == 2)
    {
      double rall, pitch;
      std::cin >> rall;
      std::cin >> pitch;
      r = robot1.ConvertMotionAxisIntoPosture(rall*M_PI/180, pitch*M_PI/180);
      q = robot1.computePosture(r);
      std::cout << q << std::endl;
      robot1.publishMotorPosition(q);
      robot1.publishJointAngle(q);
      robot1.publishChestAngle();
      // q << 0.5, 0.523597, 0.5;
      // r = robot1.getPosture(q);
      // std::cout << r << std::endl;
      
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}

/*各関節角からモーターの回転数を計算*/
    //ComputeMotorAngle::convertWaistAngleToRotationNumber();
    //ComputeMotorAngle::convertChestAngleToRotationNumber();
    //ComputeMotorAngle::convertBendingAngleToRotationNumber();
