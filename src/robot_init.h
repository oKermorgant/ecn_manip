#ifndef ECNROBOTINIT_H
#define ECNROBOTINIT_H

#include <robot_base.h>
#include <robot_kr16.h>
#include <robot_turret.h>
#include <robot_ur10.h>
#include <urdf/model.h>
#include <stdexcept>

vpColVector operator-(const vpPoseVector &p1, const vpPoseVector &p2)
{
    vpColVector d(6);
    for(unsigned int i = 0; i < 6; ++i)
        d[i] = p1[i] - p2[i];
    return d;
}

namespace ecn
{


std::unique_ptr<ecn::Robot> initRobot(int argc, char ** argv, double rate = 100)
{
    ros::init(argc, argv, "main_control");
    // parse URDF to get robot data (name, DOF, joint limits, etc.)
    urdf::Model model;
    model.initParam("/robot_description");
    const std::string name = model.getName();

    if(name == "turret")
        return std::unique_ptr<Robot>(new ecn::RobotTurret(model, rate));
    else if(name == "ur10")
        return std::unique_ptr<Robot>(new ecn::RobotUR10(model, rate));
    else if(name == "kuka_kr16")
        return std::unique_ptr<Robot>(new ecn::RobotKr16(model, rate));


    std::cout << "Robot with name " << name << " is not modeled" << std::endl;
    throw std::invalid_argument(name);
}

}


#endif
