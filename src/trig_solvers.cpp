#include <trig_solvers.h>
#include <math.h>

namespace ecn
{

std::vector<double> solveType2(double x, double y, double z)
{
  if(isNull(x) && !isNull(y))
  {
    const double t = acos(z/y);
    return {t, -t};
  }
  if(isNull(y) && !isNull(x))
  {
    const double t = asin(z/x);
    return {t, M_PI-t};
  }
  if(isNull(z))
  {
    const double t = atan2(-y,x);
    return {t, t+M_PI};
  }

  // x y z non 0
  const double d = sqr(x)+sqr(y);
  double D = d - sqr(z);
  if(D < 0)
    return {};

  std::vector<double> q;
  D = sqrt(D);
  for(const int e: {-1,1})
    q.push_back(atan2((x*z+e*y*D)/d, (y*z-e*x*D)/d));
  return q;
}

std::vector<double> solveType3(double x1, double y1, double z1,
                               double x2, double y2, double z2)
{

  if(isNull(y1) && isNull(x2))
  {
    return {atan2(z1/x1, z2/y2)};
  }
  if(isNull(x1) && isNull(y2))
  {
    return {atan2(z2/x2, z1/y1)};
  }

  const double d = x1*y2-x2*y1;
  if(isNull(d)) // singular system
  {
    std::vector<double> q;
    for(double qi: solveType2(x1, y1, z1))
    {
      // check consistency with eq. 2
      if(isNull(x2*sin(qi)+y2*cos(qi)-z2))
        q.push_back(qi);
    }
    return q;
  }
  return {atan2((z1*y2-z2*y1)/d, (z2*x1-z1*x2)/d)};
}

vectord solveType4(double x1, double y1, double x2, double y2)
{
  vectord q;
  const double r = sqrt(sqr(y1/x1) + sqr(y2/x2));
  for(const int e: {-1,1})
    q.push_back({atan2(y1/(x1*e*r), y2/(x2*e*r)),
                 e*r});
  return q;
}

vectord solveType5(double x1, double y1, double z1,
                   double x2, double y2, double z2)
{
  vectord q;
  y1 /= x1;
  z1 /= x1;
  y2 /= x2;
  z2 /= x2;

  const double D = (sqr(z1)+sqr(z2) - sqr(z1*y2-z2*y1));
  if(D<0)
    return q;

  // solve for r
  for(const int e: {-1, 1})
  {
    const double r = (-(y1*z1+y2*z2) + e*sqrt(D))/(sqr(z1)+sqr(z2));
    // solve for theta
    for(double t: solveType3(1, 0, y1+z1*r,
                             0, 1, y2+z2*r))
      q.push_back({t,r});
  }
  return q;
}

vectord solveType6(double x, double y,
                   double z1, double z2, double w)
{
  vectord q;

  // qi from type2
  for(double qi: solveType2(2*(z1*y + z2*x),
                            2*(z1*x - z2*y),
                            sqr(w) - sqr(x) - sqr(y) - sqr(z1) - sqr(z2)))
  {
    const double c = cos(qi), s = sin(qi);
    // qj from type 3
    for(double qj: solveType3(w, 0, x*c+y*s+z1,
                              0, w, x*s-y*c+z2))
      q.push_back({qi,qj});
  }
  return q;
}

vectord solveType7(double x, double y, double z1, double z2, double w1, double w2)
{
  vectord q;

  // qi from type2
  for(double qi: solveType2(2*(z1*y + z2*x),
                            2*(z1*x - z2*y),
                            sqr(w1) + sqr(w2) - sqr(x) - sqr(y) - sqr(z1) - sqr(z2)))
  {
    const double c = cos(qi), s = sin(qi);
    // qj from type 3
    for(double qj: solveType3(w2, w1, x*c+y*s+z1,
                              w1, -w2, x*s-y*c+z2))
      q.push_back({qi,qj});
  }
  return q;
}

vectord solveType8(double x, double y, double z1, double z2)
{
  vectord q;

  const double cj = (sqr(z1)+sqr(z2)-sqr(x)-sqr(y))/(2*x*y);
  const double sj = sqrt(1-sqr(cj));
  for(const int e: {-1, 1})
  {
    const double qj = atan2(e*sj, cj);
    const double b1 = x+y*cj;
    const double b2 = y*e*sj;
    const double b1b2 = sqr(b1)+sqr(b2);
    q.push_back({atan2((b1*z2-b2*z1)/b1b2, (b1*z1+b2*z2)/b1b2),qj});
  }
  return q;
}

}
