
#include <robot_base.h>
#include <urdf/model.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpRxyzVector.h>
#include <algorithm>

using std::string;
using std::cout;
using std::endl;
using ecn::Robot;

namespace {
bool inAngleLimits(double &qi, double q_min, double q_max)
{
  if(qi >= q_min && qi <= q_max)
    return true;
  qi += 2*M_PI;
  if(qi >= q_min && qi <= q_max)
    return true;
  qi -= 4*M_PI;
  if(qi >= q_min && qi <= q_max)
    return true;
  return false;
}
}

Robot::Robot(const urdf::Model &model, double rate)
{    
  nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle);
  this->rate = std::unique_ptr<ros::Rate>(new ros::Rate(rate));
  br = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster);
  tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer);
  tl = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(*tfBuffer.get()));
  t_gt = 0;

  // init joints

  std::vector<double> v_max_vec;
  q_max.clear();
  q_min.clear();
  dofs = 0;
  joint_cmd.name.clear();

  // find tree from base_link to tool0
  std::string cur_link = "tool0";
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
          v_max_vec.push_back(joint->limits->velocity);
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
  v_max_.resize(dofs);
  std::reverse(v_max_vec.begin(), v_max_vec.end());
  for(unsigned int i = 0; i < dofs; ++i)
    v_max_[i] = v_max_vec[i];
  cout << "Found robot description: " << model.getName() << " with " << dofs << " DOFs" << endl;

  q_.resize(dofs);
  joint_cmd.position.resize(dofs);
  joint_cmd.velocity.reserve(dofs);

  // initialise ROS topics: publisher for command, subscriber for position measurement
  cmd_pub = nh->advertise<sensor_msgs::JointState>("/main_control/command", 1000);
  position_sub = nh->subscribe("/joint_states", 1000, &Robot::onReadPosition, this);
  desired_pose_pub = nh->advertise<std_msgs::Float32MultiArray>("desired_pose", 10);
  desired_pose.data.resize(6);
  config.switch_time = 2;
  config.mode = 0;

  twist_sub = nh->subscribe("gui/twist_manual", 100, &Robot::onReadTwist, this);
  config_sub = nh->subscribe("gui/config", 100, &Robot::onReadConfig, this);
  ros::spinOnce();
}

void Robot::checkPose(const vpHomogeneousMatrix &M)
{
  displayFrame(M);
  if(ros::Time::now().toSec() - t_gt > 1)
  {
    t_gt = ros::Time::now().toSec();
    // passed transform
    vpRxyzVector rot(M.getRotationMatrix());
    std::cout << "Computed:     t = " << M.getTranslationVector().t() <<
                 " / RPY = " << rot.t() << std::endl;

    // check with simulation
    if(tfBuffer->canTransform("base_link", "tool0", ros::Time(0)))
    {
      auto transform = tfBuffer->lookupTransform("base_link", "tool0", ros::Time(0));
      vpTranslationVector t(transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z);
      rot.buildFrom(vpThetaUVector(
                      vpQuaternionVector(transform.transform.rotation.x,
                                         transform.transform.rotation.y,
                                         transform.transform.rotation.z,
                                         transform.transform.rotation.w)));
      std::cout << "Ground truth: t = " << t.t() <<
                   " / RPY = " << rot.t() << std::endl << std::endl;
    }
  }
}

vpHomogeneousMatrix Robot::intermediaryPose(vpHomogeneousMatrix M1, vpHomogeneousMatrix M2, double a)
{
  if(a <= 0)
    return M1;

  else if(a >= 1)
    return M2;

  /*vpPoseVector p1(M1), p2(M2), p;
    for(int i = 0; i < 6; ++i)
        p[i] = (1-a)*p1[i] + a*p2[i];
    return vpHomogeneousMatrix(p);*/

  vpHomogeneousMatrix M;
  for(unsigned int i = 0; i < 3; ++i)
    M[i][3] = (1-a)*M1[i][3] + a*M2[i][3];
  M1[0][3] = M1[1][3] = M1[2][3] = 0;
  M2[0][3] = M2[1][3] = M2[2][3] = 0;
  // t-part
  // for(unsigned int i = 0; i < 3; ++i)
  //    M[i][3] = (1-a)*M1[i][3] + a*M2[i][3];

  const vpColVector v = vpExponentialMap::inverse(M1.inverse() * M2);
  M.insert(M1.getRotationMatrix() * vpExponentialMap::direct(v, a).getRotationMatrix());
  return M;
}

// read joint_state topic and write articular position
void Robot::onReadPosition(const sensor_msgs::JointState::ConstPtr& _msg)
{
  // parse message to joint position, check for joint names
  for(unsigned int i=0;i<dofs;++i)
    q_[i] = _msg->position[i];
}


// set articular position
void Robot::setJointPosition(const vpColVector &_position)
{
  if(_position.getRows() != dofs)
    std::cout << "Robot::setPosition: bad dimension" << std::endl;
  else
  {
    joint_cmd.header.stamp = ros::Time::now();
    if(joint_cmd.velocity.size())
      joint_cmd.velocity.clear();
    for(unsigned int i=0;i<dofs;++i)
      joint_cmd.position[i] = _position[i];

    cmd_pub.publish(joint_cmd);
  }
}

// set articular velocity
void Robot::setJointVelocity(const vpColVector &_velocity)
{
  if(_velocity.getRows() != dofs)
    std::cout << "Robot::setVelocity: bad dimension" << std::endl;
  else
  {
    joint_cmd.header.stamp = ros::Time::now();

    // if v_max_, scales velocity (educational goal)
    unsigned int i;
    double scale = 1.;
    if(v_max_.size()!=0)
    {
      for(i=0;i<dofs;++i)
        scale = std::max(scale, std::abs(_velocity[i])/v_max_[i]);
      if(scale > 1)
      {
        scale = 1./scale;
        std::cout << "scaling v: " << scale << std::endl;
      }
    }

    if(joint_cmd.velocity.size() != dofs)
      joint_cmd.velocity.resize(dofs);

    for(i=0;i<dofs;++i)
      joint_cmd.velocity[i] = _velocity[i] * scale;

    cmd_pub.publish(joint_cmd);
  }
}

// loop function
bool Robot::ok()
{
  if(config.mode > 1)
  {
    updateDesiredPose();
    // publish current desired pose
    vpPoseVector p(Md());
    for(unsigned int i = 0; i < 6; ++i)
      desired_pose.data[i] = p[i];
    desired_pose_pub.publish(desired_pose);
  }
  ros::spinOnce();
  rate->sleep();
  return ros::ok();
}

vpMatrix Robot::fJe(const vpColVector &q) const
{
  vpMatrix V(6,6);
  for(unsigned int i = 0; i < 6; ++i)
    V[i][i] = 1;
  vpSubMatrix Vs(V, 0, 3, 3, 3);
  Vs = -(fMw(q).getRotationMatrix() * wMe.getTranslationVector()).skew();

  return V * fJw(q);
}


// inverse geometry methods
void Robot::addCandidate(std::vector<double> q_candidate) const
{
  if(q_candidate.size() != dofs)
  {
    std::cout << "WARNING in InverseGeometry: adding a candidate with wrong dofs"
              << std::endl;
  }
  else
    q_candidates.push_back(q_candidate);
}


vpColVector Robot::bestCandidate(const vpColVector &q0) const
{
  // returns position from q_candidates closest to q0
  vpColVector q(q0);
  if(q_candidates.size())
  {
    double best;
    int best_idx = -1;
    int k = 0;
    for(auto &qsol: q_candidates)
    {
      bool in_bounds = true;
      for(unsigned int i = 0; i < qsol.size(); ++i)
      {
        if(!inAngleLimits(qsol[i], q_min[i], q_max[i]))
          in_bounds = false;
      }
      if(in_bounds)
      {
        double d = 0;
        for(int i = 0; i < qsol.size(); ++i)
          d += fabs(q0[i] - qsol[i]);
        if(best_idx == -1 || d < best)
        {
          best = d;
          best_idx = k;
        }
      }
      else
      {
        std::cout << "Solution " << k << "/" << q_candidates.size() << " out of bounds" << std::endl;
      }
      k++;
    }

    if(best_idx != -1)
      q = q_candidates[best_idx];

    q_candidates.clear();
  }
  return q;
}
