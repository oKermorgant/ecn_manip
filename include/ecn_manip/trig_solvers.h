#ifndef ECNTRIG_SOLVERS_H
#define ECNTRIG_SOLVERS_H

#include <vector>
#include <math.h>
namespace ecn
{

struct TwoJoints
{
  double qi, qj;
};

using OneJointCandidate = std::vector<double>;
using TwoJointsCandidate = std::vector<TwoJoints>;

//double solveType1(double x, double y) {return y/x;}

inline bool isNull(double v)    {return fabs(v) < 1e-6;}
inline double sqr(double v)     {return v*v;}

/// returns cos(q) and sin(q)
inline std::pair<double,double> cos_sin(double q)
{
  return {cos(q), sin(q)};
}

/// returns either C/c or S/s depending on c and s
inline double bestDivision(double c, double s, double C, double S)
{
  if(std::abs(c) < std::abs(s)) return S/s;
  return C/s;
}

/// returns either C/cos(q) or S/sin(q) depending on cos and sin
inline double bestDivision(double q, double C, double S)
{
  const auto [c,s] = cos_sin(q); {}
  return bestDivision(c,s,C,S);
}

OneJointCandidate solveType2(double x, double y, double z=0);

OneJointCandidate solveType3(double x1, double y1, double z1,
                               double x2, double y2, double z2);

TwoJointsCandidate solveType4(double x1, double y1, double x2, double y2);

TwoJointsCandidate solveType5(double x1, double y1, double z1,
                              double x2, double y2, double z2);

TwoJointsCandidate solveType6(double x, double y,
                              double z1, double z2, double w);

TwoJointsCandidate solveType7(double x, double y, double z1, double z2, double w1, double w2);

TwoJointsCandidate solveType8(double x, double y, double z1, double z2);

}

#endif // ECNTRIG_SOLVERS_H
