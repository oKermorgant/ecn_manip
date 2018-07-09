#include <robot_kr3.h>

// Model of Kuka KR3 robot

// Any end-effector to wrist constant transform
void ecn::RobotKr3::initConstantTransforms()
{

}

// Direct Kinematics
vpHomogeneousMatrix ecn::RobotKr3::fMw(const vpColVector &q) const
{
    vpHomogeneousMatrix M;


    return M;
}


// Inverse Kinematics
vpColVector ecn::RobotKr3::computeIK(const vpHomogeneousMatrix &Md, const vpColVector &q0) const
{
    vpColVector q(dofs);


    return q;
}


vpMatrix ecn::RobotKr3::fJw(const vpColVector &q) const
{
    vpMatrix J(6, dofs);



    return J;
}
