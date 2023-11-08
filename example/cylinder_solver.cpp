#include <ecn_manip/trig_solvers.h>
#include <iostream>
#include <algorithm>
#include <visp/vpHomogeneousMatrix.h>
#include "robot_cylinder.h"

using namespace ecn;

int main()
{
  RobotCYL robot;


  for(auto i = 0 ; i < 6; ++i)
  {
    // generate random valid joint positions
    auto q = robot.jointRand();


    // check it works for cos(q5) = 1
    if(i == 3)
    {
      std::cout << "Special case: q5 = 0\n";
      q[4] = 0;
    }

    std::cout << "Source position: " << q.t() << std::endl;

    // compute corresponding DGM
    auto M = robot.fMe(q);

    // try to find q back from M
    auto q_solution = robot.inverseGeometry(M, q + 0.1*robot.jointRand());
    std::cout << "Chosen solution: " << q_solution.t();
    std::cout << "\n  pose error: " << vpPoseVector(M*robot.fMe(q_solution).inverse()).t().frobeniusNorm() << std::endl;

   /* q_solution = robot.iterativeIK(robot.fMe(q), robot.jointRand());
    std::cout << "\n Iterative solution : " << q_solution.t();
    std::cout << " / pose error: " << vpPoseVector(M*robot.fMe(q_solution).inverse()).t().frobeniusNorm() << std::endl;
    std::cout << std::endl;
*/
    std::cout << '\n';
  }





}
