#include <robot_ur10.h>

// Model of UR-10 robot

// Any constant transform at base of end-effector
void ecn::RobotUR10::initConstantTransforms()
{

}

// Direct Kinematics
vpHomogeneousMatrix ecn::RobotUR10::fMw(const vpColVector &q) const
{
    vpHomogeneousMatrix M;


    return M;
}


// Inverse Kinematics
vpColVector ecn::RobotUR10::computeIK(const vpHomogeneousMatrix &Md, const vpColVector &q0) const
{
    vpColVector q(dofs);


    return q;
}


vpMatrix ecn::RobotUR10::fJw(const vpColVector &q) const
{
    vpMatrix J(6, dofs);



    return J;
}
