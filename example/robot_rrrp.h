#ifndef ROBOT_KR16_H
#define ROBOT_KR16_H

#include <robot_base.h>

namespace ecn
{

class RobotRRRP: public ecn::Robot
{
public:
  RobotRRRP() : ecn::Robot()
  {
    // for the example robot, dimensions and joint limits are not read through ROS
    dofs = 4;
    q_min = {-M_PI, -M_PI, -M_PI, -0.1};
    q_max = {M_PI, M_PI, M_PI, 0.1};

    init_wMe();
  }

  void init_wMe();
  vpHomogeneousMatrix fMw(const vpColVector &q) const;
  vpColVector inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const;
  vpMatrix fJw(const vpColVector &q) const;
};

}

#endif // ROBOT_KR16_H
