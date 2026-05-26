#include <ecn_manip/trig_solvers.h>
#include <iostream>
#include <algorithm>
#include <visp/vpHomogeneousMatrix.h>
#include <ecn_manip/robot_base.h>

using namespace ecn;

constexpr auto l1{3};
constexpr auto l2{2};
constexpr auto l3{10};
constexpr auto l6{3};

class Robot6DOF: public ecn::Robot
{
public:
  Robot6DOF() : ecn::Robot()
  {
    // for the example robot, dimensions and joint limits are not read through ROS but hard coded
    dofs = 6;
    q_min = {-M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI};
    q_max = {M_PI, M_PI, M_PI, M_PI, M_PI, M_PI};

    init_wMe();
  }

  void init_wMe()
  {
    wMe[0][0] = 1.;
    wMe[0][1] = 0;
    wMe[0][2] = 0;
    wMe[0][3] = 0;
    wMe[1][0] = 0;
    wMe[1][1] = 1.;
    wMe[1][2] = 0;
    wMe[1][3] = 0;
    wMe[2][0] = 0;
    wMe[2][1] = 0;
    wMe[2][2] = 1.;
    wMe[2][3] = l6;
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
	const auto c6{cos(q[5])};
	const auto c23{cos(q[1]+q[2])};
	const auto s1{sin(q[0])};
	const auto s2{sin(q[1])};
	const auto s4{sin(q[3])};
	const auto s5{sin(q[4])};
	const auto s6{sin(q[5])};
	const auto s23{sin(q[1]+q[2])};
	M[0][0] = (-(s1*s4 + s23*c1*c4)*c5 - s5*c1*c23)*c6 + (-s1*c4 + s4*s23*c1)*s6;
	M[0][1] = -(-(s1*s4 + s23*c1*c4)*c5 - s5*c1*c23)*s6 + (-s1*c4 + s4*s23*c1)*c6;
	M[0][2] = -(s1*s4 + s23*c1*c4)*s5 + c1*c5*c23;
	M[0][3] = (l2*c2 + l3*c23)*c1;
	M[1][0] = ((-s1*s23*c4 + s4*c1)*c5 - s1*s5*c23)*c6 + (s1*s4*s23 + c1*c4)*s6;
	M[1][1] = -((-s1*s23*c4 + s4*c1)*c5 - s1*s5*c23)*s6 + (s1*s4*s23 + c1*c4)*c6;
	M[1][2] = (-s1*s23*c4 + s4*c1)*s5 + s1*c5*c23;
	M[1][3] = (l2*c2 + l3*c23)*s1;
	M[2][0] = (s5*s23 - c4*c5*c23)*c6 + s4*s6*c23;
	M[2][1] = -(s5*s23 - c4*c5*c23)*s6 + s4*c6*c23;
	M[2][2] = -s5*c4*c23 - s23*c5;
	M[2][3] = l1 - l2*s2 - l3*s23;
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

	for(const auto q1: {atan2(ty,tx), atan2(-ty,-tx)})
	{
	  const auto Z1{bestDivision(q1, tx, ty)};
	  const auto Z2{l1-tz};
	  const auto c1{cos(q1)};
	  const auto s1{sin(q1)};
	  for(const auto &[q2,q3]: solveType8(l2, l3, Z1, Z2))
	  {
		vpRotationMatrix R03;
		const auto c23{cos(q2+q3)};
		const auto s23{sin(q2+q3)};
		R03[0][0] = -s23*c1;
		R03[0][1] = -c1*c23;
		R03[0][2] = -s1;
		R03[1][0] = -s1*s23;
		R03[1][1] = -s1*c23;
		R03[1][2] = c1;
		R03[2][0] = -c23;
		R03[2][1] = s23;
		R03[2][2] = 0;
		const auto [xx,xy,xz,yx,yy,yz,zx,zy,zz] = explodeWristMatrix(Md, R03);
		if(isNull(xy*xy+yy*yy))
		{
		  const auto q5{0.};
		  const auto q46{atan2(xz, xx)};
		  addCandidate({q1,q2,q3,q46/2,q5,q46/2});
		}
		else
		{
		  const auto q6{atan2(-yy,xy)};
		  const auto q4{atan2(zz,zx)};
		  const auto q5{atan2(bestDivision(q4, zx, zz), -zy)};
		  addCandidate({q1,q2,q3,q4,q5,q6});
		}
	  }
	}
	return bestCandidate(q0);
  }
  vpMatrix fJw(const vpColVector &q) const override
  {
    return vpMatrix(6, 6);
  }
};

int main()
{
  Robot6DOF robot;


  for(auto i = 0 ; i < 6; ++i)
  {
    // generate random valid joint positions
    auto q = robot.jointRand();

    std::cout << "Source position: " << q.t() << std::endl;

    // compute corresponding DGM
    auto M = robot.fMe(q);

    // try to find q back from M
    auto q_solution = robot.inverseGeometry(M, robot.jointRand());
    std::cout << "Chosen solution: " << q_solution.t();
    std::cout << "\n pose error: " << vpPoseVector(M*robot.fMe(q_solution).inverse()).t().frobeniusNorm() << std::endl;

    std::cout << '\n';
  }





}
