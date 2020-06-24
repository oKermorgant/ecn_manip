#include <node.h>
#include <urdf/model.h>

namespace ecn
{
void Node::init(int argc, char **argv)
{
  ros::init(argc, argv, "main_control");
}

Node::Node(double _rate) : rate(_rate), tl(tfBuffer)
{
  urdf::Model model;
  model.initParam("/robot_description");
  robot_name = model.getName();
}

uint Node::initRobot(std::vector<double> &q_max, std::vector<double> &q_min,
                    std::vector<double> &v_max)
{
  urdf::Model model;
  model.initParam("/robot_description");
  // find tree from base_link to tool0
  std::string cur_link = "tool0";

  q_max.clear();
  q_min.clear();
  v_max.clear();

  while(cur_link != "base_link")
  {
    for(const auto &joint_it : model.joints_)
    {
      urdf::JointSharedPtr joint = joint_it.second;
      if(joint->child_link_name == cur_link)
      {
        if(joint->type != urdf::Joint::FIXED)
        {
          joint_cmd.name.push_back(joint->name);
          v_max.push_back(joint->limits->velocity);
          q_max.push_back(joint->limits->upper);
          q_min.push_back(joint->limits->lower);
          dofs += 1;
        }
        cur_link = joint->parent_link_name;
      }
    }
  }
  std::reverse(joint_cmd.name.begin(), joint_cmd.name.end());
  std::reverse(q_max.begin(), q_max.end());
  std::reverse(q_min.begin(), q_min.end());
  v_max.resize(dofs);
  std::reverse(v_max.begin(), v_max.end());
  std::cout << "Found robot description: " << robot_name << " with " << dofs << " DOFs" << std::endl;

  q.resize(dofs);
  joint_cmd.position.resize(dofs);
  joint_cmd.velocity.reserve(dofs);
  desired_pose.data.resize(6);

  initInterface();

  return dofs;
}

double Node::time()
{
  return ros::Time::now().toSec();
}

int Node::cycleLength() const
{
  return config.switch_time / rate.expectedCycleTime().toSec();
}

bool Node::ok()
{
  ros::spinOnce();
  rate.sleep();
  return ros::ok();
}

void Node::initInterface()
{
  // initialise ROS topics: publisher for command, subscriber for position measurement
  cmd_pub = nh.advertise<sensor_msgs::JointState>("/main_control/command", 1000);
  desired_pose_pub = nh.advertise<std_msgs::Float32MultiArray>("desired_pose", 10);
  desired_pose.data.resize(6);

  position_sub = nh.subscribe("/joint_states", 1000, &Node::onReadPosition, this);
  twist_sub = nh.subscribe("gui/twist_manual", 100, &Node::onReadTwist, this);
  config_sub = nh.subscribe("gui/config", 100, &Node::onReadConfig, this);
  ros::spinOnce();
}

void Node::sendTransform(const vpHomogeneousMatrix &M, const std::string &frame)
{
  const vpTranslationVector t(M);
  vpQuaternionVector qu; M.extract(qu);
  geometry_msgs::TransformStamped transform;
  transform.transform.translation.x = t[0];
  transform.transform.translation.y = t[1];
  transform.transform.translation.z = t[2];
  transform.transform.rotation.x = qu.x();
  transform.transform.rotation.y = qu.y();
  transform.transform.rotation.z = qu.z();
  transform.transform.rotation.w = qu.w();
  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = "base_link";
  transform.child_frame_id  = frame;

  br.sendTransform(transform);
}

// read joint_state topic and write articular position
void Node::onReadPosition(const sensor_msgs::JointState::ConstPtr& _msg)
{
  // parse message to joint position, check for joint names
  for(unsigned int i=0;i<dofs;++i)
    q[i] = _msg->position[i];
}

void Node::setJointPosition(const vpColVector &_q)
{
  joint_cmd.header.stamp = ros::Time::now();
  if(joint_cmd.velocity.size())
    joint_cmd.velocity.clear();
  for(unsigned int i=0;i<dofs;++i)
    joint_cmd.position[i] = _q[i];
  cmd_pub.publish(joint_cmd);
}

void Node::setJointVelocity(const vpColVector &_velocity, double scale)
{
  joint_cmd.header.stamp = ros::Time::now();

  if(joint_cmd.velocity.size() != dofs)
    joint_cmd.velocity.resize(dofs);

  for(uint i=0;i<dofs;++i)
    joint_cmd.velocity[i] = _velocity[i] * scale;

  cmd_pub.publish(joint_cmd);
}

void Node::setDesiredPose(const vpPoseVector &_pose)
{
  for(unsigned int i = 0; i < 6; ++i)
    desired_pose.data[i] = static_cast<float>(_pose[i]);
  desired_pose_pub.publish(desired_pose);
}

void Node::printGroundTruth()
{
  // check with simulation
  if(tfBuffer.canTransform("base_link", "tool0", ros::Time(0)))
  {
    auto transform = tfBuffer.lookupTransform("base_link", "tool0", ros::Time(0));
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

