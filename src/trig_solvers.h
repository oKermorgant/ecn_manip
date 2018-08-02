#ifndef ECNTRIG_SOLVERS_H
#define ECNTRIG_SOLVERS_H

#include <vector>
#include <math.h>
namespace ecn
{

struct JointSolution
{
    double qi, qj;
};

typedef std::vector<JointSolution> vectord;

//double solveType1(double x, double y) {return y/x;}

inline bool isNull(double v)    {return fabs(v) < 1e-6;}
inline double sqr(double v)     {return v*v;}

bool inAngleLimits(double &qi, double q_min, double q_max);
bool inAngleLimits(double &qi, double si, double ci, double q_min, double q_max);

std::vector<double> solveType2(double x, double y, double z, double q_min, double q_max);

std::vector<double> solveType3(double x1, double y1, double z1,
                               double x2, double y2, double z2,
                               double q_min, double q_max);

vectord solveType4(double x1, double y1, double x2, double y2,
                   double qi_min, double qi_max,
                   double qj_min, double qj_max);

vectord solveType5(double x1, double y1, double z1,
                   double x2, double y2, double z2,
                   double qi_min, double qi_max,
                   double qj_min, double qj_max);

vectord solveType6(double x, double y,
                   double z1, double z2, double w,
                   double qi_min, double qi_max,
                   double qj_min, double qj_max);

vectord solveType7(double x, double y, double z1, double z2, double w1, double w2,
                   double qi_min, double qi_max,
                   double qj_min, double qj_max);

vectord solveType8(double x, double y, double z1, double z2,
                   double qi_min, double qi_max,
                   double qj_min, double qj_max);

}

#endif // ECNTRIG_SOLVERS_H
