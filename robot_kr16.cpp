#include <robot_kr16.h>

// Model of Kuka KR16 robot

// Any end-effector to wrist constant transform
void ecn::RobotKr16::init_wMe()
{

}

// Direct Kinematics
vpHomogeneousMatrix ecn::RobotKr16::fMw(const vpColVector &q) const
{
    vpHomogeneousMatrix M;


    return M;
}


// Inverse Kinematics
vpColVector ecn::RobotKr16::inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const
{
    vpColVector q(dofs);


    return q;
}


vpMatrix ecn::RobotKr16::fJw(const vpColVector &q) const
{
    vpMatrix J(6, dofs);



    return J;
}
