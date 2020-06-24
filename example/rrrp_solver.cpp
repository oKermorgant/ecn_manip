#include <trig_solvers.h>
#include <iostream>
#include <algorithm>
#include <visp/vpHomogeneousMatrix.h>
#include "robot_rrrp.h"

using namespace ecn;

int main()
{
  RobotRRRP robot;

  for(auto i = 0 ; i < 6; ++i)
  {
    // generate random valid joint positions
    auto q = robot.jointRand();

    // check it works for cos(q3) = 0
    if(i == 0)
      q[2] = M_PI/2;
    // check behavior if q1 out of bounds
    if(i == 1)
    {
      std::cout << "q1 out of bounds\n";
      q[0] = robot.jointMax()[0] + 1.;
    }
    // check behavior if q4 out of bounds
    if(i == 2)
    {
      std::cout << "q4 out of bounds\n";
      q[3] = robot.jointMax()[3] + 1.;
    }


    std::cout << "Joint positions: " << q.t() << std::endl;

    // compute corresponding DGM
    auto M = robot.fMe(q);

    // try to find q back from M
    auto q_solution = robot.inverseGeometry(M, robot.jointRand());
    std::cout << "Solution found : " << q_solution.t() << std::endl;
    M *= robot.fMe(q_solution).inverse();
    std::cout << "Pose error: " << vpPoseVector(M).t().frobeniusNorm() << std::endl;

    q_solution = robot.iterativeIK(robot.fMe(q), robot.jointRand());
    std::cout << "Iterative sol. found : " << q_solution.t() << std::endl;
    M *= robot.fMe(q_solution).inverse();
    std::cout << "Pose error: " << vpPoseVector(M).t().frobeniusNorm() << std::endl;
    std::cout << std::endl;
  }







}
