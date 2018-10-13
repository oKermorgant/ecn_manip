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

std::vector<double> solveType2(double x, double y, double z);

std::vector<double> solveType3(double x1, double y1, double z1,
                               double x2, double y2, double z2);

vectord solveType4(double x1, double y1, double x2, double y2);

vectord solveType5(double x1, double y1, double z1,
                   double x2, double y2, double z2);

vectord solveType6(double x, double y,
                   double z1, double z2, double w);

vectord solveType7(double x, double y, double z1, double z2, double w1, double w2);

vectord solveType8(double x, double y, double z1, double z2);

}

#endif // ECNTRIG_SOLVERS_H
