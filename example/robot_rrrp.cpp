#include "robot_rrrp.h"
#include <trig_solvers.h>

// Model of RRRP robot

// Any end-effector to wrist constant transform
void ecn::RobotRRRP::init_wMe()
{
  wMe[2][3] = 0.07;
}

// Direct Geometry fixed to wrist frame
vpHomogeneousMatrix ecn::RobotRRRP::fMw(const vpColVector &q) const
{
  const double r1 = 0.27;
  const double a2 = 0.2;
  const double r3 = 0.15;

  vpHomogeneousMatrix M;
  // Generated pose code
  const double c1 = cos(q[0]);
  const double c3 = cos(q[2]);
  const double c12 = cos(q[0]+q[1]);
  const double s1 = sin(q[0]);
  const double s3 = sin(q[2]);
  const double s12 = sin(q[0]+q[1]);
  M[0][0] = -s12*c3;
  M[0][1] = c12;
  M[0][2] = -s3*s12;
  M[0][3] = a2*c1 - q[3]*s3*s12 + r3*c12;
  M[1][0] = c3*c12;
  M[1][1] = s12;
  M[1][2] = s3*c12;
  M[1][3] = a2*s1 + q[3]*s3*c12 + r3*s12;
  M[2][0] = s3;
  M[2][1] = 0;
  M[2][2] = -c3;
  M[2][3] = -q[3]*c3 + r1;
  M[3][0] = 0;
  M[3][1] = 0;
  M[3][2] = 0;
  M[3][3] = 1.;
  // End of pose code

  return M;
}



//#define MANIP_2021

#ifdef MANIP_2021

// Inverse Geometry - 2021 approach with type 3
vpColVector ecn::RobotRRRP::inverseGeometry(const vpHomogeneousMatrix &fMe_des, const vpColVector &q0) const
{
  // elements of target DGM matrix (new in 2021!)
  const auto [xx,xy,xz,yx,yy,yz,zx,zy,zz,tx,ty,tz] = explodeMatrix(fMe_des);

      const double r1 = 0.27;
      const double a2 = 0.2;
      const double r3 = 0.15;

      // start by solving q3
      for(const double q3: solveType3(1, 0, xz, 0, -1, zz))
  {
    const auto c3{cos(q3)};
    const auto s3{sin(q3)};

    if(!isNull(c3))
    {
      const double q4 = -(tz - r1)/c3;

      for(const auto q12: solveType3(1, 0, yy, 0, 1, yx))
      {
        const auto c12{cos(q12)};
        const auto s12{sin(q12)};

        const auto X1{0};
        const auto Y1{a2};
        const auto Z1{tx + q4*s3*s12 - r3*c12};

        const auto X2{a2};
        const auto Y2{0};
        const auto Z2{ty - q4*s3*c12 - r3*s12};

        for(const auto q1: solveType3(X1, Y1, Z1, X2, Y2, Z2))
        {
          const auto q2 = q12 - q1;
          addCandidate({q1, q2, q3, q4});
        }
      }
    }
    else
    {
      // cos(q3) = 0 -> do something else

      const auto c12{yx};
      const auto s12{yy};

      const auto Y1{ty-r3*s12};
      const auto Z1{-s3*c12};
      const auto Y2{tx-r3*c12};
      const auto Z2{s3*s12};
      for(const auto [q1, q4]: solveType5(a2, Y1, Z1, a2, Y2, Z2))
      {
        for(const auto q12: solveType3(1,0,s12,0,1,c12))
        {
          const auto q2{q12-q1};
          addCandidate({q1, q2, q3, q4});
        }
      }
    }
  }

  const auto eMf{fMe_des.inverse()};
  for(const auto &q_cand: q_candidates)
  {
    std::cout << " - Candidate solution: ";
    for(auto &qi: q_cand)
      std::cout << qi << " ";
    std::cout << " (pose error is " << vpPoseVector(fMe(q_cand)*eMf).t().frobeniusNorm() << ")" << std::endl;
  }

  return bestCandidate(q0);
}

#else

// Inverse Geometry - 2020 approach with type 5
vpColVector ecn::RobotRRRP::inverseGeometry(const vpHomogeneousMatrix &fMe_des, const vpColVector &q0) const
{
  // reduced DGM
  const vpHomogeneousMatrix oMw = fM0.inverse() * fMe_des * wMe.inverse();
  const auto [xx,xy,xz,yx,yy,yz,zx,zy,zz,tx,ty,tz] = explodeMatrix(fMe_des);

      const double r1 = 0.27;
      const double a2 = 0.2;
      const double r3 = 0.15;

      const auto s3{oMw[2][0]};
  const auto c3{-oMw[2][2]};

  // start by solving q3
  for(double q3: solveType3(1, 0, s3, 0, 1, c3))
  {
    const auto c12{yx};
    const auto s12{yy};

    const auto Y1{ty-r3*s12};
    const auto Z1{-s3*c12};
    const auto Y2{tx-r3*c12};
    const auto Z2{s3*s12};
    for(const auto [q1, q4]: solveType5(a2, Y1, Z1, a2, Y2, Z2))
    {
      if(isNull(-q4*c3 + r1 - tz))
      {
        for(const auto q12: solveType3(1,0,s12,0,1,c12))
        {
          const auto q2{q12-q1};
          addCandidate({q1, q2, q3, q4});
        }
      }
    }
  }

  const auto eMf{fMe_des.inverse()};
  for(const auto &q_cand: q_candidates)
  {
    std::cout << " - Candidate solution: ";
    for(auto &qi: q_cand)
      std::cout << qi << " ";
    std::cout << " (pose error is " << vpPoseVector(fMe(q_cand)*eMf).t().frobeniusNorm() << ")" << std::endl;
  }



  return bestCandidate(q0);
}

#endif


vpMatrix ecn::RobotRRRP::fJw(const vpColVector &q) const
{
  const double a2 = 0.2;
  const double r3 = 0.15;

  vpMatrix J(6, dofs);

  // Generated Jacobian code
  const double c1 = cos(q[0]);
  const double c3 = cos(q[2]);
  const double c12 = cos(q[0]+q[1]);
  const double s1 = sin(q[0]);
  const double s3 = sin(q[2]);
  const double s12 = sin(q[0]+q[1]);
  J[0][0] = -a2*s1 - q[3]*s3*c12 - r3*s12;
  J[0][1] = -q[3]*s3*c12 - r3*s12;
  J[0][2] = -q[3]*s12*c3;
  J[0][3] = -s3*s12;
  J[1][0] = a2*c1 - q[3]*s3*s12 + r3*c12;
  J[1][1] = -q[3]*s3*s12 + r3*c12;
  J[1][2] = q[3]*c3*c12;
  J[1][3] = s3*c12;
  //J[2][0] = 0;
  //J[2][1] = 0;
  J[2][2] = q[3]*s3;
  J[2][3] = -c3;
  //J[3][0] = 0;
  //J[3][1] = 0;
  J[3][2] = c12;
  //J[3][3] = 0;
  //J[4][0] = 0;
  //J[4][1] = 0;
  J[4][2] = s12;
  //J[4][3] = 0;
  J[5][0] = 1.;
  J[5][1] = 1.;
  //J[5][2] = 0;
  //J[5][3] = 0;
  // End of Jacobian code

  return J;
}
