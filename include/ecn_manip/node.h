#ifndef ECN_MANIP_NODE_H
#define ECN_MANIP_NODE_H

#include <visp/vpColVector.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rate.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

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

class Node : public rclcpp::Node
{
public:
  static void init(int argc, char ** argv);
  Node(double _rate = 100);
  Node(rclcpp::NodeOptions) : Node() {}
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

  std::shared_ptr<rclcpp::Node> nh;
  rclcpp::WallRate rate;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr desired_pose_pub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr config_sub;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tl;
  tf2_ros::TransformBroadcaster br;

  sensor_msgs::msg::JointState joint_cmd;
  std_msgs::msg::Float32MultiArray desired_pose;
};

}

#endif // NODE_H
