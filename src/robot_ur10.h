#ifndef ROBOT_UR10_H
#define ROBOT_UR10_H

#include <robot_base.h>
#include <ur_kin/ur_kin.h>

namespace ecn
{

class RobotUR10 : public ecn::Robot
{
public:
    RobotUR10(std::unique_ptr<Node> &_node) : ecn::Robot(_node)
    {
        vpPoseVector p;

     /*   p[0] = 0.3004522869;
        p[1] = -0.5945692882;
        p[2] = .848935232;
        p[3] = -1.219712878;
        p[4] = .82020013;
        p[5] = -1.195306449;*/
        p[0] = 1.035090748;
        p[1] = -0.3528816786;
        p[2] = 0.6013436785;
        p[3] = -0.9453002187;
        p[4] = 2.633358931;
        p[5] = 0.2983002012;
        M1_.buildFrom(p);

        /*p[0] = 0.3004522869;
        p[1] = -0.5945692882;
        p[2] = .848935232;
        p[3] = -0.02018874914;
        p[4] = -0.4552241391;
        p[5] = 0.1691234224;*/
        p[0] = 1.035090748;
        p[1] = -0.3528816786;
        p[2] = 0.6013436785;
        p[3] = 1.723449781;
        p[4] = -2.273080608;
        p[5] = 0.8914900738;
        M2_.buildFrom(p);
        init_wMe();
    }
    void init_wMe();
    vpHomogeneousMatrix fMw(const vpColVector &q) const;
    vpColVector inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const
    {
        vpMatrix q_sol(8,6);
        vpHomogeneousMatrix eMw;
        eMw[0][0] = eMw[1][1] = eMw[2][2] = 0;
        eMw[1][2] = eMw[0][1] = -1;
        eMw[2][0] = 1;
        eMw[2][3] = -0.1;

        vpHomogeneousMatrix M = Md*eMw;
        int sols = ur_kinematics::inverse(M.data, q_sol.data);
        for(int i = 0; i < sols; ++i)
        {
            for(int j = 0; j <6; ++j)
                if(q_sol[i][j] > M_PI)
                    q_sol[i][j] -= 2*M_PI;
        }

        // get nearest position from q0
        int best = 0;
        double best_dist = (q_sol.getRow(0).t() - q0).frobeniusNorm();
        for(int i = 1; i < sols; ++i)
        {
            const double d = (q_sol.getRow(i).t() - q0).frobeniusNorm();
            if(d < best_dist)
            {
                best = i;
                best_dist = d;
            }
        }
        return q_sol.getRow(best).t();
    }

    vpMatrix fJw(const vpColVector &q) const;
};

}

#endif // ROBOT_UR10_H
