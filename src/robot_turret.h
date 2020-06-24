#ifndef ROBOT_TURRET_H
#define ROBOT_TURRET_H

#include <robot_base.h>

namespace ecn
{

class RobotTurret : public ecn::Robot
{
public:
    RobotTurret(std::unique_ptr<Node> &_node) : ecn::Robot(_node)
    {
        vpPoseVector p;
        p[0] = .065;        p[1] = -0.02;        p[2] = .61;
        p[3] = 0.08263749827;
        p[4] = 0.5495697433;
        p[5] = -0.2908102243;
        M1_.buildFrom(p);
        p[0] = -0.046;
        p[1] = -0.046;
        p[2] = 0.6;
        p[3] = 0.6751199893;
        p[4] = 0.2796438558;
        p[5] = -2.275845114;
        M2_.buildFrom(p);
        init_wMe();
    }
    void init_wMe();
    vpHomogeneousMatrix fMw(const vpColVector &q) const;
    vpColVector inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const;
    vpMatrix fJw(const vpColVector &q) const;
};
}

#endif // ROBOT_TURRET_H
