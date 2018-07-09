#ifndef ROBOT_UR10_H
#define ROBOT_UR10_H

#include <robot_base.h>

namespace ecn
{

class RobotUR10 : public ecn::Robot
{
public:
    RobotUR10(const urdf::Model &model, double rate = 100) : ecn::Robot(model, rate)
    {
        vpPoseVector p;
        p[0] = 0.3004522869;
        p[1] = -0.5945692882;
        p[2] = 1.048935232;
        p[3] = -1.219712878;
        p[4] = .82020013;
        p[5] = -1.195306449;
        M1_.buildFrom(p);

        p[0] = 0.3004522869;
        p[1] = -0.5945692882;
        p[2] = 1.048935232;
        p[3] = -0.02018874914;
        p[4] = -0.4552241391;
        p[5] = 0.1691234224;
        M2_.buildFrom(p);
        initConstantTransforms();
    }
    void initConstantTransforms();
    vpHomogeneousMatrix fMw(const vpColVector &q) const;
    vpColVector inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const;
    vpMatrix fJw(const vpColVector &q) const;
};

}

#endif // ROBOT_UR10_H
