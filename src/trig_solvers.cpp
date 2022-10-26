#include <ecn_manip/trig_solvers.h>
#include <math.h>
#include <algorithm>
#include <iostream>

namespace ecn
{

template <typename Solution, typename Particular, class Transform>
inline void append(std::vector<Solution> &q, const std::vector<Particular> &part,
            const Transform &tr)
{
  std::transform(part.begin(), part.end(), std::back_inserter(q), tr);
}

template <class Solver>
auto toTwoSolutions(const Solver &fct)
{
  return std::vector{fct(-1), fct(1)};
}

std::vector<double> solveType2(double x, double y, double z)
{
  if(isNull(x) && !isNull(y))
  {
    const auto t = acos(z/y);
    return {t, -t};
  }
  if(isNull(y) && !isNull(x))
  {
    const auto t = asin(z/x);
    return {t, M_PI-t};
  }
  if(isNull(z))
  {
    const auto t{atan2(-y,x)};
    return {t, t+M_PI};
  }

  // x y z non 0
  const auto d{sqr(x)+sqr(y)};
  if(d - sqr(z) < 0)
    return {};
  const auto D{sqrt(d - sqr(z))};
  return toTwoSolutions([&](int e){return atan2((x*z+e*y*D)/d, (y*z-e*x*D)/d);});
}

std::vector<double> solveType3(double x1, double y1, double z1,
                               double x2, double y2, double z2)
{

  if(isNull(y1) && isNull(x2))
    return {atan2(z1/x1, z2/y2)};

  if(isNull(x1) && isNull(y2))
    return {atan2(z2/x2, z1/y1)};

  const auto d{x1*y2-x2*y1};
  if(isNull(d)) // singular system
  {
    std::vector<double> q;
    const auto candidates{solveType2(x1, y1, z1)};
    std::copy_if(candidates.begin(), candidates.end(), std::back_inserter(q), [&](auto qi)
    {
      // check consistency with eq. 2
      return isNull(x2*sin(qi)+y2*cos(qi)-z2);
    });
    return q;
  }
  return {atan2((z1*y2-z2*y1)/d, (z2*x1-z1*x2)/d)};
}

TwoJointsCandidate solveType4(double x1, double y1, double x2, double y2)
{
  if(isNull(x1) || isNull(x2))
  {
    std::cerr << "solveType4: x1 or x2 is null, cannot solve" << std::endl;
    return {};
  }
  const auto r = sqrt(sqr(y1/x1) + sqr(y2/x2));
  return toTwoSolutions([&](int e)
  {
    return TwoJoints{atan2(y1/(x1*e*r), y2/(x2*e*r)), e*r};
  });
}

TwoJointsCandidate solveType5(double x1, double y1, double z1,
                   double x2, double y2, double z2)
{
  if(isNull(x1) || isNull(x2))
  {
    std::cerr << "solveType5: x1 or x2 is null, cannot solve" << std::endl;
    return {};
  }
  TwoJointsCandidate q;
  y1 /= x1;
  z1 /= x1;
  y2 /= x2;
  z2 /= x2;

  const auto D = (sqr(z1)+sqr(z2) - sqr(z1*y2-z2*y1));
  if(D<0)
    return q;

  // solve for r
  for(const int e: {-1, 1})
  {
    const auto r = (-(y1*z1+y2*z2) + e*sqrt(D))/(sqr(z1)+sqr(z2));
    append(q, solveType3(1, 0, y1+z1*r, 0, 1, y2+z2*r),
           [&](auto t) {return TwoJoints{t,r};});
  }
  return q;
}

TwoJointsCandidate solveType6(double x, double y,
                   double z1, double z2, double w)
{
  TwoJointsCandidate q;

  // qi from type2
  for(auto qi: solveType2(2*(z1*y + z2*x),
                            2*(z1*x - z2*y),
                            sqr(w) - sqr(x) - sqr(y) - sqr(z1) - sqr(z2)))
  {
    const auto [c,s] = cos_sin(qi);
    // qj from type 3
    append(q, solveType3(w, 0, x*c+y*s+z1, 0, w, x*s-y*c+z2),
           [&](auto qj) {return TwoJoints{qi,qj};});
  }
  return q;
}

TwoJointsCandidate solveType7(double x, double y, double z1, double z2, double w1, double w2)
{
  TwoJointsCandidate q;

  // qi from type2
  for(double qi: solveType2(2*(z1*y + z2*x),
                            2*(z1*x - z2*y),
                            sqr(w1) + sqr(w2) - sqr(x) - sqr(y) - sqr(z1) - sqr(z2)))
  {
    const auto [c,s] = cos_sin(qi);
    // qj from type 3
    append(q, solveType3(w2, w1, x*c+y*s+z1, w1, -w2, x*s-y*c+z2),
           [&](auto qj) {return TwoJoints{qi,qj};});
  }
  return q;
}

TwoJointsCandidate solveType8(double x, double y, double z1, double z2)
{
  const auto cj = (sqr(z1)+sqr(z2)-sqr(x)-sqr(y))/(2*x*y);
  const auto sj = sqrt(1-sqr(cj));
  return toTwoSolutions([&](int e)
  {
    const auto qj = atan2(e*sj, cj);
    const auto b1 = x+y*cj;
    const auto b2 = y*e*sj;
    const auto b1b2 = sqr(b1)+sqr(b2);
    return TwoJoints{atan2((b1*z2-b2*z1)/b1b2, (b1*z1+b2*z2)/b1b2),qj};
  });
}

}
