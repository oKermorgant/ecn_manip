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
    std::cout << std::endl;
    // generate random valid joint positions
    auto q = robot.jointRand();

    // check if it works for cos(q3) = 0
    if(i == 0)
    {
      std::cout << "q3 is pi/2\n";
      q[2] = M_PI/2;
    }
    // check if it works for cos(q3) = 0
    else if(i == 1)
    {
      std::cout << "q3 is -pi/2\n";
      q[2] = -M_PI/2;
    }
    // check behavior if q4 out of bounds
    else if(i == 2)
    {
      std::cout << "q4 out of bounds\n";
      q[3] = robot.jointMax()[3] + 1.;
    }
    else
    {
      std::cout << "valid joint position" << endl;
    }

    std::cout << "Joint positions: " << q.t() << std::endl;

    // compute corresponding DGM
    auto M = robot.fMe(q);

    q += 0.1*robot.jointRand();

    // try to find q back from M
    auto q_solution = robot.inverseGeometry(M, q);
    std::cout << "Chosen solution: " << q_solution.t() << std::endl;

    /*q_solution = robot.iterativeIK(robot.fMe(q), robot.jointRand());
    std::cout << "Iterative sol. found : " << q_solution.t() << std::endl;
    M *= robot.fMe(q_solution).inverse();
    std::cout << "  Pose error: " << vpPoseVector(M).t().frobeniusNorm() << std::endl;*/

  }


}
