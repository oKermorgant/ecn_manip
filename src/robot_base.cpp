#include <robot_base.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpRxyzVector.h>
#include <algorithm>
#include <visp/vpSubColVector.h>

using std::string;
using std::cout;
using std::endl;
using ecn::Robot;

namespace {
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
}

Robot::Robot(std::unique_ptr<ecn::Node> &_node)
  : node(std::move(_node))
{
  dofs = node->initRobot(q_max, q_min, v_max);
}

void Robot::checkPose(const vpHomogeneousMatrix &M)
{
  displayFrame(M);
  if(node->time() - t_gt > 1)
  {
    t_gt = node->time();
    // passed transform
    vpRxyzVector rot(M.getRotationMatrix());
    std::cout << "Computed:     t = " << M.getTranslationVector().t() <<
                 " / RPY = " << rot.t() << std::endl;

    // check with simulation
    node->printGroundTruth();
  }
}

vpHomogeneousMatrix Robot::intermediaryPose(vpHomogeneousMatrix M1, vpHomogeneousMatrix M2, double a)
{
  if(a <= 0)
    return M1;

  else if(a >= 1)
    return M2;

  const vpThetaUVector dTU(M1.getRotationMatrix().t() * M2.getRotationMatrix());
  return vpHomogeneousMatrix(
        M1.getTranslationVector()*(1-a) + M2.getTranslationVector()*a,
        M1.getRotationMatrix() * vpRotationMatrix(dTU[0]*a, dTU[1]*a, dTU[2]*a));
}


// set articular position
void Robot::setJointPosition(const vpColVector &_position)
{
  if(_position.getRows() != dofs)
  {
    std::cout << "Robot::setPosition: bad dimension" << std::endl;
    return;
  }
  node->setJointPosition(_position);
}

vpColVector Robot::iterativeIK(const vpHomogeneousMatrix &Md, vpColVector q0) const
{
  const uint max_iter(1000);
  const double min_error = 1e-4;
  uint iter(0);
  const double l = 0.15;

  const auto fMw_d = Md * wMe.inverse();

  auto M = fMw(q0);

  vpColVector v(6);
  vpSubColVector t(v, 0, 3);
  vpSubColVector tu(v, 3, 3);

  vpPoseVector p(M.inverse()*fMw_d);

  while(iter++ < max_iter &&
        (p.getTranslationVector().frobeniusNorm() > min_error ||
         std::abs(p.getThetaUVector().getTheta()) > 1e-3))
  {


    t = M.getRotationMatrix() * p.getTranslationVector();
    tu = M.getRotationMatrix() * static_cast<vpColVector>(p.getThetaUVector());
    q0 += l * fJw(q0).t() * v;
    M = fMw(q0);
    p.buildFrom(M.inverse() * fMw_d);
  }
  return q0;
}

// set articular velocity
void Robot::setJointVelocity(const vpColVector &_velocity)
{
  if(_velocity.getRows() != dofs)
  {
    std::cout << "Robot::setVelocity: bad dimension" << std::endl;
    return;
  }
  // if v_max_, scales velocity (educational goal)
  unsigned int i;
  double scale = 1.;
  if(v_max.size()!=0)
  {
    for(i=0;i<dofs;++i)
      scale = std::max(scale, std::abs(_velocity[i])/v_max[i]);
    if(scale > 1)
    {
      scale = 1./scale;
      std::cout << "scaling v: " << scale << std::endl;
    }
  }
  node->setJointVelocity(_velocity, scale);
}

// loop function
bool Robot::ok()
{
  if(node->config.mode > 1)
  {
    updateDesiredPose();
    // publish current desired pose
    vpPoseVector p(Md());
    node->setDesiredPose(p);
  }
  return node->ok();
}

vpMatrix Robot::fJe(const vpColVector &q) const
{
  vpMatrix V(6,6);
  for(unsigned int i = 0; i < 6; ++i)
    V[i][i] = 1;
  vpSubMatrix Vs(V, 0, 3, 3, 3);
  Vs = -(fMw(q).getRotationMatrix() * wMe.getTranslationVector()).skew();

  return V * fJw(q);
}


// inverse geometry methods
void Robot::addCandidate(std::vector<double> q_candidate) const
{
  if(q_candidate.size() != dofs)
  {
    std::cout << "WARNING in InverseGeometry: adding a candidate with wrong dofs"
              << std::endl;
  }
  else
    q_candidates.push_back(q_candidate);
}


vpColVector Robot::bestCandidate(const vpColVector &q0) const
{
  // returns position from q_candidates closest to q0
  vpColVector q(q0);
  if(q_candidates.size())
  {
    double best = 0.0;
    int best_idx = -1;
    int k = 0;
    for(auto &qsol: q_candidates)
    {
      bool in_bounds = true;
      for(unsigned int i = 0; i < qsol.size(); ++i)
      {
        if(!inAngleLimits(qsol[i], q_min[i], q_max[i]))
          in_bounds = false;
      }
      if(in_bounds)
      {
        double d = 0;
        for(uint i = 0; i < qsol.size(); ++i)
          d += fabs(q0[i] - qsol[i]);
        if(best_idx == -1 || d < best)
        {
          best = d;
          best_idx = k;
        }
      }
      k++;
    }

    if(best_idx != -1)
      q = q_candidates[best_idx];

    q_candidates.clear();
  }
  return q;
}
