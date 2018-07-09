#ifndef ROBOT_KR16_H
#define ROBOT_KR16_H

#include <robot_base.h>

namespace ecn
{

class RobotKr16: public ecn::Robot
{
public:
    RobotKr16(const urdf::Model &model, double rate = 100) : ecn::Robot(model, rate)
    {
        vpPoseVector p;

        p[0] = 2.26116489;
        p[1] =   0.3939218829;
        p[2] =  0.7428915174 ;
        p[3] = -0.6677717793;
        p[4] =   2.524004629;
        p[5] =   1.167453601;
        M1_.buildFrom(p);

        p[0] = 1.402406755;
        p[1] =   1.362092363;
        p[2] =   1.677578319;
        p[3] =   1.914559837;
        p[4] =   0.3815079999;
        p[5] =   1.153593361;
        M2_.buildFrom(p);
        initConstantTransforms();
    }
    void initConstantTransforms();
    vpHomogeneousMatrix fMw(const vpColVector &q) const;
    vpColVector computeIK(const vpHomogeneousMatrix &Md, const vpColVector &q0) const;
    vpMatrix fJw(const vpColVector &q) const;
};
}

#endif // ROBOT_KR16_H
