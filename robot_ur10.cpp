#include <robot_ur10.h>
#include <trig_solvers.h>

// Model of UR-10 robot

// Any constant transform at base of end-effector
void ecn::RobotUR10::init_wMe()
{

}

// Direct Geometry
vpHomogeneousMatrix ecn::RobotUR10::fMw(const vpColVector &q) const
{
    vpHomogeneousMatrix M;


    return M;
}


// Inverse Geometry is already given for this robot


// Wrist Jacobian
vpMatrix ecn::RobotUR10::fJw(const vpColVector &q) const
{
    vpMatrix J(6, dofs);



    return J;
}
