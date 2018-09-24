#include <trig_solvers.h>
#include <math.h>

namespace ecn
{

bool inAngleLimits(double &qi, double q_min, double q_max)
{
    if(qi >= q_min && qi <= q_max)
        return true;
    qi += 2*M_PI;
    if(qi >= q_min && qi <= q_max)
        return true;
    qi -= 4*M_PI;
    if(qi >= q_min && qi <= q_max)
        return true;
    return false;
}

bool inAngleLimits(double &qi, double si, double ci, double q_min, double q_max)
{   // returns True and updates qi if si and ci are sin/cos and atan2 is between joint limits
    if(fabs(si) <= 1 && fabs(ci) <= 1)
    {
        qi = atan2(si, ci);
        return inAngleLimits(qi, q_min, q_max);
    }
    return false;
}

std::vector<double> solveType2(double x, double y, double z, double q_min, double q_max)
{
    std::vector<double> q;
    q.reserve(2);
    if(isNull(x) && !isNull(y))
    {
        double t = acos(z/y);
        if(inAngleLimits(t, q_min, q_max))
            q.push_back(t);
        t = -t;
        if(inAngleLimits(t, q_min, q_max))
            q.push_back(t);
        return q;
    }
    if(isNull(y) && !isNull(x))
    {
        double t = asin(z/x);
        if(inAngleLimits(t, q_min, q_max))
            q.push_back(t);
        t = M_PI-t;
        if(inAngleLimits(t, q_min, q_max))
            q.push_back(t);
        return q;
    }
    if(isNull(z))
    {
        double t = atan2(-y,x);
        if(inAngleLimits(t, q_min, q_max))
            q.push_back(t);
        t = t+M_PI;
        if(inAngleLimits(t, q_min, q_max))
            q.push_back(t);
    }
    // x y z non 0
    const double d = sqr(x)+sqr(y);
    double D = d - sqr(z);
    if(D < 0)
        return q;
    D = sqrt(D);
    double t;
    for(const int e: {-1,1})
    {
        if(inAngleLimits(t, (x*z+e*y*D)/d, (y*z-e*x*D)/d, q_min, q_max))
            q.push_back(t);
    }
    return q;
}

std::vector<double> solveType3(double x1, double y1, double z1,
                               double x2, double y2, double z2,
                               double q_min, double q_max)
{
    std::vector<double> q;
    q.reserve(2);

    if(isNull(y1) && isNull(x2))
    {
        double t;
        if(inAngleLimits(t, z1/x1, z2/y2, q_min, q_max))
            q.push_back(t);
        return q;
    }
    if(isNull(x1) && isNull(y2))
    {
        double t;
        if(inAngleLimits(t, z2/x2, z1/y1, q_min, q_max))
            q.push_back(t);
        return q;
    }

    const double d = x1*y2-x2*y1;
    if(isNull(d)) // singular system
    {
        for(double qi: solveType2(x1, y1, z1, q_min, q_max))
        {
            // check consistency with eq. 2
            if(isNull(x2*sin(qi)+y2*cos(qi)-z2))
                q.push_back(qi);
        }
        return q;
    }
    double t;
    if(inAngleLimits(t, (z1*y2-z2*y1)/d, (z2*x1-z1*x2)/d, q_min, q_max))
        q.push_back(t);
    return q;
}

vectord solveType4(double x1, double y1, double x2, double y2,
                   double qi_min, double qi_max, double qj_min, double qj_max)
{
    vectord q;
    const double r = sqrt(sqr(y1/x1) + sqr(y2/x2));
    for(const int e: {-1,1})
    {
        if(e*r <= qj_max && e*r >= qj_min)
        {
            double t;
            if(inAngleLimits(t, y1/(x1*e*r), y2/(x2*e*r), qi_min, qi_max))
                q.push_back({t,e*r});
        }
    }
    return q;
}

vectord solveType5(double x1, double y1, double z1,
                   double x2, double y2, double z2,
                   double qi_min, double qi_max,
                   double qj_min, double qj_max)
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
        if(r <= qj_max && r >= qj_min)
        {
            // solve for theta
            for(double t: solveType3(1, 0, y1+z1*r,
                                     0, 1, y2+z2*r,
                                     qi_min, qi_max))
                q.push_back({t,r});
        }
    }
    return q;
}

vectord solveType6(double x, double y,
                   double z1, double z2, double w,
                   double qi_min, double qi_max,
                   double qj_min, double qj_max)
{
    vectord q;

    // qi from type2
    for(double qi: solveType2(2*(z1*y + z2*x),
                              2*(z1*x - z2*y),
                              sqr(w) - sqr(x) - sqr(y) - sqr(z1) - sqr(z2),
                              qi_min, qi_max))
    {
        const double c = cos(qi), s = sin(qi);
        // qj from type 3
        for(double qj: solveType3(w, 0, x*c+y*s+z1,
                                  0, w, x*s-y*c+z2,
                                  qj_min, qj_max))
            q.push_back({qi,qj});
    }
    return q;
}

vectord solveType7(double x, double y, double z1, double z2, double w1, double w2,
                   double qi_min, double qi_max,
                   double qj_min, double qj_max)
{
    vectord q;

    // qi from type2
    for(double qi: solveType2(2*(z1*y + z2*x),
                              2*(z1*x - z2*y),
                              sqr(w1) + sqr(w2) - sqr(x) - sqr(y) - sqr(z1) - sqr(z2),
                              qi_min, qi_max))
    {
        const double c = cos(qi), s = sin(qi);
        // qj from type 3
        for(double qj: solveType3(w2, w1, x*c+y*s+z1,
                                  w1, -w2, x*s-y*c+z2,
                                  qj_min, qj_max))
            q.push_back({qi,qj});
    }
    return q;
}

vectord solveType8(double x, double y, double z1, double z2,
                   double qi_min, double qi_max,
                   double qj_min, double qj_max)
{
    vectord q;

    const double cj = (sqr(z1)+sqr(z2)-sqr(x)-sqr(y))/(2*x*y);
    const double sj = sqrt(1-sqr(cj));
    double qj, qi;
    for(const int e: {-1, 1})
    {
        qj = atan2(e*sj, cj);
        if(inAngleLimits(qj, qj_min, qj_max))
        {
            const double b1 = x+y*cj;
            const double b2 = y*e*sj;
            const double b1b2 = sqr(b1)+sqr(b2);
            if(inAngleLimits(qi, (b1*z2-b2*z1)/b1b2, (b1*z1+b2*z2)/b1b2, qi_min, qi_max))
                q.push_back({qi,qj});
        }
    }
    return q;
}






}
