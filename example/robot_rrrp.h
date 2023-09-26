#ifndef ROBOT_KR16_H
#define ROBOT_KR16_H

#include <ecn_manip/robot_base.h>

namespace ecn
{

class RobotRRRP: public ecn::Robot
{
public:
  RobotRRRP() : ecn::Robot()
  {
    // for the example robot, dimensions and joint limits are not read through ROS but hard coded
    dofs = 4;
    q_min = {-M_PI, -M_PI, -M_PI, -0.1};
    q_max = {M_PI, M_PI, M_PI, 0.1};

    this->init_wMe();
  }

  void init_wMe() override;
  vpHomogeneousMatrix fMw(const vpColVector &q) const override;
  vpColVector inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const override;
  vpMatrix fJw(const vpColVector &q) const override;
};

}

#endif // ROBOT_KR16_H
