#include <ecn_manip/trig_solvers.h>
#include <iostream>
#include <visp/vpHomogeneousMatrix.h>
#include <ecn_manip/robot_base.h>

using namespace ecn;

constexpr auto l{0.12};
class Robot5DOF: public ecn::Robot
{
public:
  Robot5DOF() : ecn::Robot()
  {
    // for the example robot, dimensions and joint limits are not read through ROS but hard coded
    dofs = 5;
    q_min = {-M_PI, -M_PI, -M_PI, -M_PI, -M_PI};
    q_max = {M_PI, M_PI, M_PI, M_PI, M_PI};

    init_wMe();
  }

  void init_wMe()
  {
    wMe[0][0] = 1.;
    wMe[0][1] = 0;
    wMe[0][2] = 0;
    wMe[0][3] = 0;
    wMe[1][0] = 0;
    wMe[1][1] = 0;
    wMe[1][2] = -1.;
    wMe[1][3] = -l;
    wMe[2][0] = 0;
    wMe[2][1] = 1.;
    wMe[2][2] = 0;
    wMe[2][3] = 0;
    wMe[3][0] = 0;
    wMe[3][1] = 0;
    wMe[3][2] = 0;
    wMe[3][3] = 1.;
  }
  vpHomogeneousMatrix fMw(const vpColVector &q) const override
  {
    vpHomogeneousMatrix M;

	const auto c1{cos(q[0])};
	const auto c2{cos(q[1])};
	const auto c4{cos(q[3])};
	const auto c5{cos(q[4])};
	const auto c23{cos(q[1]+q[2])};
	const auto s1{sin(q[0])};
	const auto s2{sin(q[1])};
	const auto s4{sin(q[3])};
	const auto s5{sin(q[4])};
	const auto s23{sin(q[1]+q[2])};
	M[0][0] = (-s1*s4 + c1*c4*c23)*c5 - s5*s23*c1;
	M[0][1] = -(-s1*s4 + c1*c4*c23)*s5 - s23*c1*c5;
	M[0][2] = -s1*c4 - s4*c1*c23;
	M[0][3] = l*(s2 + 2.0*s23)*c1;
	M[1][0] = (s1*c4*c23 + s4*c1)*c5 - s1*s5*s23;
	M[1][1] = -(s1*c4*c23 + s4*c1)*s5 - s1*s23*c5;
	M[1][2] = -s1*s4*c23 + c1*c4;
	M[1][3] = l*(s2 + 2.0*s23)*s1;
	M[2][0] = -s5*c23 - s23*c4*c5;
	M[2][1] = s5*s23*c4 - c5*c23;
	M[2][2] = s4*s23;
	M[2][3] = l*(c2 + 2.0*c23 + 1.0);
	M[3][0] = 0;
	M[3][1] = 0;
	M[3][2] = 0;
	M[3][3] = 1.;
	// End of pose code

	return M;
  }
  vpColVector inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const override
  {
    const auto [tx,ty,tz] = explodeTranslation(Md);


	const auto ac1{tx/l};
	const auto as1{ty/l};
	double s22s23 = 0.;
	std::vector<double> q1s{q0[0]};
	if(!isNull(ac1) || !isNull(as1))
	{
	  q1s = {atan2(as1, ac1), atan2(as1, ac1)+M_PI};
	  const auto [c1,s1] = cos_sin(q1s[0]);
	  s22s23 = bestDivision(c1, s1, ac1, as1);
	}
	const auto c22c23{tz/l - 1};
	for(const auto q1: q1s)
	{
	  const auto [c1,s1] = cos_sin(q1);
	  for(const auto [q2,q3]: solveType8(1, 2, c22c23, s22s23))
	  {
		const auto [c23, s23] = cos_sin(q2+q3);
		vpRotationMatrix R03;
		R03[0][0] = c1*c23;
		R03[0][1] = -s23*c1;
		R03[0][2] = -s1;
		R03[1][0] = s1*c23;
		R03[1][1] = -s1*s23;
		R03[1][2] = c1;
		R03[2][0] = -s23;
		R03[2][1] = -c23;
		R03[2][2] = 0;

		const auto [xx,xy,xz,yx,yy,yz,zx,zy,zz] = explodeWristMatrix(Md, R03);
		const auto q5{atan2(xy,yy)};
		const auto q4{atan2(-zx, zz)};
		addCandidate({q1,q2,q3,q4,q5}, Md);
	  }
	}
	return bestCandidate(q0);
  }
  vpMatrix fJw(const vpColVector &q) const override
  {
    return vpMatrix(6, 5);
  }
};



int main()
{
  Robot5DOF robot;

  const auto q1q2 = solveType8(1,2,3,0);

  for(auto i = 0 ; i < 5; ++i)
  {
    // generate random valid joint positions
    auto q = robot.jointRand();

	// check it works for sin(q2+q3) = 0
	if(i == 0)
	{
	  std::cout << "Special case: s2 = s23 = 0\n";
	  q[1] = q[2] = 0;
	}

	std::cout << "Source position: " << q.t() << std::endl;

	// compute corresponding DGM
	auto M = robot.fMe(q);

	// try to find q back from M
	auto q_solution = robot.inverseGeometry(M, robot.jointRand());
	std::cout << "Chosen solution: " << q_solution.t();
	std::cout << "\n pose error: " << vpPoseVector(M*robot.fMe(q_solution).inverse()).t().frobeniusNorm() << std::endl;

    /* q_solution = robot.iterativeIK(robot.fMe(q), robot.jointRand());
    std::cout << "\n Iterative solution : " << q_solution.t();
    std::cout << " / pose error: " << vpPoseVector(M*robot.fMe(q_solution).inverse()).t().frobeniusNorm() << std::endl;
    std::cout << std::endl;
*/
    std::cout << '\n';
  }

}
