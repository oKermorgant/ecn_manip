#include <visp/vpHomogeneousMatrix.h>
#include <robot_init.h>

// generic call to models
// DO NOT MODIFY

vpHomogeneousMatrix computeDK(const vpColVector &q, const ecn::Robot &robot)
{
    if (robot.name() == "turret")
        return turret::computeDK(q);
    else if(robot.name() == "kuka_kr3r540")
        return kr3::computeDK(q);
    return ur10::computeDK(q);
}

vpColVector computeIK(const vpColVector &q0, const vpHomogeneousMatrix &Md, const ecn::Robot &robot)
{
    if (robot.name() == "turret")
        return turret::computeIK(q0, Md, robot);
    else if(robot.name() == "kuka_kr3r540")
        return kr3::computeIK(q0, Md, robot);

    return ur10::computeIK(q0, Md, robot);
}


// calcul  Jacobien
vpMatrix computeJacobian(const vpColVector &q, const ecn::Robot &robot)
{
    if (robot.name() == "turret")
        return turret::computeJacobian(q);
    else if(robot.name() == "kuka_kr3r540")
        return ur10::computeJacobian(q);

    return ur10::computeJacobian(q);
}


