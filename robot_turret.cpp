#include <ecn_manip/robot_turret.h>
#include <ecn_manip/trig_solvers.h>

// Model of Turret robot

// Any constant transform at base or end-effector
void ecn::RobotTurret::init_wMe()
{

}

// Direct Geometry
vpHomogeneousMatrix ecn::RobotTurret::fMw(const vpColVector &q) const
{
    vpHomogeneousMatrix M;

    return M;
}


// Inverse Geometry
vpColVector ecn::RobotTurret::inverseGeometry(const vpHomogeneousMatrix &fMe_des, const vpColVector &q0) const
{
    // build corresponding oMw and explode into 12 elements
    const auto [xx,xy,xz,yx,yy,yz,zx,zy,zz,tx,ty,tz] = explodeMatrix(fMe_des);

    // TODO add candidates


    return bestCandidate(q0);
}

// Wrist Jacobian
vpMatrix ecn::RobotTurret::fJw(const vpColVector &q) const
{
    vpMatrix J(6, dofs);



    return J;
}
