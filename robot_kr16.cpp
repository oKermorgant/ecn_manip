#include <ecn_manip/robot_kr16.h>
#include <ecn_manip/trig_solvers.h>

// Model of Kuka KR16 robot

// Any end-effector to wrist constant transform
void ecn::RobotKr16::init_wMe()
{

}

// Direct Geometry fixed to wrist frame
vpHomogeneousMatrix ecn::RobotKr16::fMw(const vpColVector &q) const
{
  vpHomogeneousMatrix M;


  return M;
}


// Inverse Geometry
vpColVector ecn::RobotKr16::inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const
{
  // desired wrist position
  const auto [tx,ty,tz] = explodeTranslation(Md);

  // first solve position for (q1,q2,q3)



  // then (inside the last for block) build R36 and solve it for (q4,q5,q6)



  return bestCandidate(q0);
}


vpMatrix ecn::RobotKr16::fJw(const vpColVector &q) const
{
  vpMatrix J(6, dofs);



  return J;
}
