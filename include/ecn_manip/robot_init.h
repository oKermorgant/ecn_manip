#ifndef ECNROBOTINIT_H
#define ECNROBOTINIT_H

#include <ecn_manip/robot_base.h>
#include <ecn_manip/robot_kr16.h>
#include <ecn_manip/robot_turret.h>
#include <ecn_manip/robot_ur10.h>
#include <stdexcept>
#include <visp/vpSubMatrix.h>
#include <visp/vpSubColVector.h>


vpColVector operator-(const vpPoseVector &p1, const vpPoseVector &p2)
{
  vpColVector d(6);
  for(unsigned int i = 0; i < 6; ++i)
    d[i] = p1[i] - p2[i];
  return d;
}

namespace ecn
{

void putAt(vpMatrix &_J, const vpMatrix &_Jsub, const unsigned int r, const unsigned int c)
{
  vpSubMatrix Js(_J, r, c, _Jsub.getRows(), _Jsub.getCols());
  Js = _Jsub;
}

// put a vector inside another
void putAt(vpColVector &_e, const vpColVector &_esub, const unsigned int r)
{
  vpSubColVector es(_e, r, _esub.getRows());
  es = _esub;
}

std::unique_ptr<ecn::Robot> initRobot(int argc, char ** argv, double rate = 100)
{
  Node::init(argc, argv);
  auto node{std::make_unique<Node>(rate)};
  const std::string name{node->robotName()};

  if(name == "turret")
    return std::make_unique<RobotTurret>(node);
  else if(name == "ur10")
    return std::make_unique<RobotUR10>(node);
  else if(name == "kuka_kr16")
    return std::make_unique<RobotKr16>(node);

  std::cout << "Robot with name " << name << " is not modeled" << std::endl;
  throw std::invalid_argument(name);
}
}

#endif
