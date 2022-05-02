#include <ecn_manip/node.h>
#include <urdf/model.h>
#include <std_msgs/msg/string.hpp>

namespace ecn
{


void Node::init(int argc, char **argv)
{
  rclcpp::init(argc, argv);
}

Node::Node(double _rate)
  : rclcpp::Node("control"), nh(this), rate(_rate)
  , tfBuffer(get_clock()), tl(tfBuffer), br(nh)
{
}

std::unique_ptr<urdf::Model> Node::robot() const
{
  // get robot_description from
  const auto rsp{std::make_shared<rclcpp::Node>("rsp_reader")};
  const auto rsp_param{std::make_shared<rclcpp::SyncParametersClient>
                      (rsp, "/robot_state_publisher")};
  std::cout << "Reading robot description... " << std::flush;
  while(true)
  {
    rclcpp::spin_some(rsp);
    rsp_param->wait_for_service();
    const auto urdf_xml{rsp_param->get_parameter<std::string>("robot_description")};
    if(urdf_xml.empty())
      continue;



    auto model{std::make_unique<urdf::Model>()};
    model->initString(urdf_xml);
    std::cout << " found " << model->getName() << std::endl;
    return model;
  }
  return {};
}

void Node::initInterface(const std::vector<std::string> &joints)
{
  dofs = joints.size();
  joint_cmd.name = joints;
  joint_cmd.position.resize(dofs);
  joint_cmd.velocity.resize(dofs);
  q.resize(dofs);
  desired_pose.data.resize(6);
  initInterface();
}

double Node::time()
{
  return get_clock()->now().seconds();
}

int Node::cycleLength() const
{
  return config.switch_time / std::chrono::duration_cast<std::chrono::seconds>(rate.period()).count();
}

bool Node::ok()
{
  rclcpp::spin_some(nh);
  rate.sleep();
  return rclcpp::ok();
}

void Node::initInterface()
{
  // initialise ROS topics: publisher for command, subscriber for position measurement
  cmd_pub = nh->create_publisher<sensor_msgs::msg::JointState>("/main_control/command", 10);
  desired_pose_pub = nh->create_publisher<std_msgs::msg::Float32MultiArray>("desired_pose", 10);
  desired_pose.data.resize(6);

  position_sub = create_subscription<sensor_msgs::msg::JointState>
                 ("/joint_states", 10, [this](sensor_msgs::msg::JointState::UniquePtr msg)
  {
                 for(unsigned int i=0;i<dofs;++i)
      q[i] = msg->position[i];});

twist_sub = create_subscription<geometry_msgs::msg::Twist>
            ("gui/twist_manual", 10, [this](geometry_msgs::msg::Twist::UniquePtr msg)
{
            desired_twist[0] = msg->linear.x;
desired_twist[1] = msg->linear.y;
desired_twist[2] = msg->linear.z;
desired_twist[3] = msg->angular.x;
desired_twist[4] = msg->angular.y;
desired_twist[5] = msg->angular.z;});

config_sub = create_subscription<sensor_msgs::msg::JointState>
             ("gui/config", 10, [this](sensor_msgs::msg::JointState::UniquePtr msg)
{
             config.updateFrom(msg->name, msg->position);
             });

rclcpp::spin_some(nh);
}

void Node::sendTransform(const vpHomogeneousMatrix &M, const std::string &frame)
{
  const vpTranslationVector t(M);
  vpQuaternionVector qu; M.extract(qu);
  geometry_msgs::msg::TransformStamped transform;
  transform.transform.translation.x = t[0];
  transform.transform.translation.y = t[1];
  transform.transform.translation.z = t[2];
  transform.transform.rotation.x = qu.x();
  transform.transform.rotation.y = qu.y();
  transform.transform.rotation.z = qu.z();
  transform.transform.rotation.w = qu.w();
  transform.header.stamp = get_clock()->now();
  transform.header.frame_id = "base_link";
  transform.child_frame_id  = frame;

  br.sendTransform(transform);
}

void Node::setJointPosition(const vpColVector &_q)
{
  joint_cmd.header.stamp = get_clock()->now();
  if(joint_cmd.velocity.size())
    joint_cmd.velocity.clear();
  for(unsigned int i=0;i<dofs;++i)
    joint_cmd.position[i] = _q[i];
  cmd_pub->publish(joint_cmd);
}

void Node::setJointVelocity(const vpColVector &_velocity, double scale)
{
  joint_cmd.header.stamp = get_clock()->now();

  if(joint_cmd.velocity.size() != dofs)
    joint_cmd.velocity.resize(dofs);

  for(uint i=0;i<dofs;++i)
    joint_cmd.velocity[i] = _velocity[i] * scale;

  cmd_pub->publish(joint_cmd);
}

void Node::setDesiredPose(const vpPoseVector &_pose)
{
  for(unsigned int i = 0; i < 6; ++i)
    desired_pose.data[i] = static_cast<float>(_pose[i]);
  desired_pose_pub->publish(desired_pose);
}


void Node::printGroundTruth()
{
  // check with simulation
  if(tfBuffer.canTransform("base_link", "tool0", tf2::TimePointZero,
                           tf2::durationFromSec(1)))
  {
    auto transform = tfBuffer.lookupTransform("base_link", "tool0", tf2::TimePointZero);
    vpTranslationVector t(transform.transform.translation.x,
                          transform.transform.translation.y,
                          transform.transform.translation.z);
    vpRxyzVector rot(vpThetaUVector(
                       vpQuaternionVector(transform.transform.rotation.x,
                                          transform.transform.rotation.y,
                                          transform.transform.rotation.z,
                                          transform.transform.rotation.w)));
    std::cout << "Ground truth: t = " << t.t() <<
                 " / RPY = " << rot.t() << std::endl << std::endl;
  }
}
}
