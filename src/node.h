#ifndef ECN_MANIP_NODE_H
#define ECN_MANIP_NODE_H

#include <visp/vpColVector.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace ecn
{

struct Config
{
  double lambda = 1;
  int mode = 0;
  double switch_time = 2;
  void updateFrom(const std::vector<std::string> &names,
                  const std::vector<double> &values)
  {
    size_t i(0);
    for(const auto &name: names)
    {
      if(name == "lambda")
        lambda = values[i];
      else if(name == "mode")
        mode = round(values[i]);
      else if(name == "switch_time")
        switch_time = values[i];
      i++;
    }
  }
};

class Node
{
public:
  static void init(int argc, char ** argv);
  Node(double _rate = 100);
  std::string robotName() const {return robot_name;}
  uint initRobot(std::vector<double> &q_max, std::vector<double> &q_min,
                 std::vector<double> &v_max);

  void setJointPosition(const vpColVector &_q);
  void setJointVelocity(const vpColVector &_velocity, double scale);
  vpColVector jointPosition() const;
  void setDesiredPose(const vpPoseVector &_pose);

  bool ok();
  double time();
  int cycleLength() const;

  void sendTransform(const vpHomogeneousMatrix &M, const std::string &frame);
  void printGroundTruth();

  // ROS-agnostic
  Config config;
  vpColVector twist;
  uint dofs = 0;
  std::vector<double> q;
  vpColVector desired_twist = vpColVector(6);
  std::vector<std::string> names;
  std::string robot_name;


private:
  void initInterface();

  ros::NodeHandle nh;
  ros::Rate rate;
  ros::Publisher cmd_pub, desired_pose_pub;
  ros::Subscriber position_sub, twist_sub, config_sub;
  tf2_ros::TransformBroadcaster br;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tl;
  sensor_msgs::JointState joint_cmd;

  std_msgs::Float32MultiArray desired_pose;

  void onReadPosition(const sensor_msgs::JointState::ConstPtr& _msg);
  void onReadTwist(const geometry_msgs::Twist &_msg)
  {
    desired_twist[0] = _msg.linear.x;
    desired_twist[1] = _msg.linear.y;
    desired_twist[2] = _msg.linear.z;
    desired_twist[3] = _msg.angular.x;
    desired_twist[4] = _msg.angular.y;
    desired_twist[5] = _msg.angular.z;
  }
  void onReadConfig(const sensor_msgs::JointState::ConstPtr& _msg)
  {
    config.updateFrom(_msg->name, _msg->position);
  }

};

}

#endif // NODE_H
