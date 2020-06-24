#ifndef ROBOT_KR16_H
#define ROBOT_KR16_H

#include <robot_base.h>

namespace ecn
{

class RobotKr16: public ecn::Robot
{
public:
    RobotKr16(std::unique_ptr<Node> &_node) : ecn::Robot(_node)
    {
        vpPoseVector p;

        p[0] = 0.812110097;
        p[1] = -1.187465983;
        p[2] = 0.4915271277;
        p[3] = 0.9423584377;
        p[4] = -2.852157604;
        p[5] =  0.1075927581;
        M1_.buildFrom(p);

        p[0] = 1.282877001;
        p[1] = 0.3344962596;
        p[2] = 1.415167601;
        p[3] = -1.142592072;
        p[4] = 1.424640835;
        p[5] =  1.004968206;
        M2_.buildFrom(p);
        init_wMe();
    }
    void init_wMe();
    vpHomogeneousMatrix fMw(const vpColVector &q) const;
    vpColVector inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const;
    vpMatrix fJw(const vpColVector &q) const;
};
}

#endif // ROBOT_KR16_H
