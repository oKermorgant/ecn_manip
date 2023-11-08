#include "robot_cylinder.h"
#include <ecn_manip/trig_solvers.h>

// Model of cylinder robot

const auto r4{.5};
const auto r2{.6};

// Any end-effector to wrist constant transform
void ecn::RobotCYL::init_wMe()
{
}

// Direct Geometry fixed to wrist frame
vpHomogeneousMatrix ecn::RobotCYL::fMw(const vpColVector &q) const
{


  vpHomogeneousMatrix M;
  // Generated pose code
  const auto c1{cos(q[0])};
  const auto c4{cos(q[3])};
  const auto c5{cos(q[4])};
  const auto c6{cos(q[5])};
  const auto s1{sin(q[0])};
  const auto s4{sin(q[3])};
  const auto s5{sin(q[4])};
  const auto s6{sin(q[5])};
  M[0][0] = (s1*s5 + s4*c1*c5)*c6 + s6*c1*c4;
  M[0][1] = -(s1*s5 + s4*c1*c5)*s6 + c1*c4*c6;
  M[0][2] = -s1*c5 + s4*s5*c1;
  M[0][3] = (-q[2] - r4)*s1;
  M[1][0] = (s1*s4*c5 - s5*c1)*c6 + s1*s6*c4;
  M[1][1] = -(s1*s4*c5 - s5*c1)*s6 + s1*c4*c6;
  M[1][2] = s1*s4*s5 + c1*c5;
  M[1][3] = (q[2] + r4)*c1;
  M[2][0] = -s4*s6 + c4*c5*c6;
  M[2][1] = -s4*c6 - s6*c4*c5;
  M[2][2] = s5*c4;
  M[2][3] = q[1] + r2;
  M[3][0] = 0;
  M[3][1] = 0;
  M[3][2] = 0;
  M[3][3] = 1.;
  // End of pose code


  return M;
}


// Inverse Geometry
vpColVector ecn::RobotCYL::inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const
{
  const auto [tx,ty,tz]  = explodeTranslation(Md); {}

  const auto q2 = tz - r2;

  for(auto sign: {-1, 1})
  {
    const auto q3 = sign * sqrt(tx*tx+ty*ty) - r4;

    for(const auto q1: solveType3(-q3-r4, 0, tx, 0, q3+r4, ty))
    {
      vpRotationMatrix R03;
      const auto c1{cos(q1)};
      const auto s1{sin(q1)};
      R03[0][0] = c1;
      R03[0][1] = 0;
      R03[0][2] = -s1;
      R03[1][0] = s1;
      R03[1][1] = 0;
      R03[1][2] = c1;
      R03[2][0] = 0;
      R03[2][1] = -1.;
      R03[2][2] = 0;

      const auto [xx,xy,xz,yx,yy,yz,zx,zy,zz] = explodeWristMatrix(Md, R03); {}

      if(isNull(xz*xz + yz*yz))
      {
        // q5 = 0
        const auto q46{atan2(yy, yx)};
        for(const auto q46_t: {q46, q46+2*M_PI, q46-2*M_PI})
        {
          const auto q6 = .5*(q0[5] + q46_t - q0[3]);
          const auto q4{q46_t-q6};
          addCandidate({q1,q2,q3,q4,0,q6});
        }
      }
      else
      {
        for(const auto s5: {sqrt(xz*xz + yz*yz), -sqrt(xz*xz + yz*yz)})
        {
          for(const auto q4: solveType3(s5, 0, zx, 0, -s5, zy))
          {
            for(const auto q6: solveType3(s5, 0, yz, 0, -s5, xz))
            {
              addCandidate({q1,q2,q3,q4,atan2(s5, zz),q6});
            }
          }
        }
      }
    }
  }
  return bestCandidate(q0);
}


vpMatrix ecn::RobotCYL::fJw(const vpColVector &q) const
{

  vpMatrix J(6, dofs);
  // Generated Jacobian code
  const auto c1{cos(q[0])};
  const auto c4{cos(q[3])};
  const auto c5{cos(q[4])};
  const auto s1{sin(q[0])};
  const auto s4{sin(q[3])};
  const auto s5{sin(q[4])};
  J[0][0] = (-q[2] - r4)*c1;
  J[0][1] = 0;
  J[0][2] = -s1;
  J[0][3] = 0;
  J[0][4] = 0;
  J[0][5] = 0;
  J[1][0] = (-q[2] - r4)*s1;
  J[1][1] = 0;
  J[1][2] = c1;
  J[1][3] = 0;
  J[1][4] = 0;
  J[1][5] = 0;
  J[2][0] = 0;
  J[2][1] = 1.;
  J[2][2] = 0;
  J[2][3] = 0;
  J[2][4] = 0;
  J[2][5] = 0;
  J[3][0] = 0;
  J[3][1] = 0;
  J[3][2] = 0;
  J[3][3] = -s1;
  J[3][4] = c1*c4;
  J[3][5] = -s1*c5 + s4*s5*c1;
  J[4][0] = 0;
  J[4][1] = 0;
  J[4][2] = 0;
  J[4][3] = c1;
  J[4][4] = s1*c4;
  J[4][5] = s1*s4*s5 + c1*c5;
  J[5][0] = 1.;
  J[5][1] = 0;
  J[5][2] = 0;
  J[5][3] = 0;
  J[5][4] = -s4;
  J[5][5] = s5*c4;
  // End of Jacobian code


  return J;
}
