#ifndef ROBOT_CYL_H
#define ROBOT_CYL_H

#include <ecn_manip/robot_base.h>

namespace ecn
{

class RobotCYL: public ecn::Robot
{
public:
  RobotCYL() : ecn::Robot()
  {
    // for the example robot, dimensions and joint limits are not read through ROS but hard coded
    dofs = 6;
    q_min = {-M_PI, 0, 0, -M_PI, -M_PI, -M_PI};
    q_max = {M_PI, 0.5, 0.3, M_PI, M_PI, M_PI};

    this->init_wMe();
  }

  void init_wMe() override;
  vpHomogeneousMatrix fMw(const vpColVector &q) const override;
  vpColVector inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const override;
  vpMatrix fJw(const vpColVector &q) const override;
};

}

#endif // ROBOT_CYL_H
