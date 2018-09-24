#include <robot_kr16.h>
#include <trig_solvers.h>

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
    // desired wrist pose
    vpHomogeneousMatrix fMw = Md * wMe.inverse();




    return bestCandidate(q0);
}


vpMatrix ecn::RobotKr16::fJw(const vpColVector &q) const
{
    vpMatrix J(6, dofs);



    return J;
}
