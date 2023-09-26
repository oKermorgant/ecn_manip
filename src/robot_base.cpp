#include <ecn_manip/robot_base.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpRxyzVector.h>
#include <algorithm>
#include <visp/vpSubColVector.h>

using std::string;
using std::cout;
using std::endl;
using ecn::Robot;

namespace {
bool angleOK(double &qi, double q_min, double q_max)
{
  for(auto turn: {0., 2*M_PI, -4*M_PI})
  {
    qi += turn;
    if(qi >= q_min && qi <= q_max)
      return true;
  }
  return false;
}
}

Robot::Robot(std::shared_ptr<ecn::Node> &_node, const urdf::Model &model)
  : node(_node)
{
  // parse robot model

  std::string cur_link = "tool0";
  q_max.clear();
  q_min.clear();
  v_max.clear();
  std::vector<string> names;

  while(cur_link != "base_link")
  {
    for(const auto &[name, joint]: model.joints_)
    {
      if(joint->child_link_name == cur_link)
      {
        if(joint->type != urdf::Joint::FIXED)
        {
          q_max.insert(q_max.begin(), joint->limits->upper);
          q_min.insert(q_min.begin(), joint->limits->lower);
          v_max.insert(v_max.begin(), joint->limits->velocity);
          names.insert(names.begin(), joint->name);
        }
        cur_link = joint->parent_link_name;
      }
    }
  }
  dofs = q_max.size();
  node->initInterface(names);
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
  if(mode() == ControlMode::VELOCITY_P2P || mode() == ControlMode::VELOCITY_MANUAL)
    return;

  if(_position.getRows() != dofs)
  {
    std::cout << "Robot::setPosition: bad dimension" << std::endl;
    return;
  }
  node->setJointPosition(_position);
}

vpColVector Robot::iterativeIK(const vpHomogeneousMatrix &fMe_des, vpColVector q0) const
{
  const uint max_iter(1000);
  const double min_lin_error = 1e-4;
  const double min_ang_error = 1e-3;
  const double lambda = 0.15;

  const auto fMw_d = fMe_des * wMe.inverse();

  auto M = fMw(q0);

  vpColVector v(6);
  vpPoseVector p(M.inverse()*fMw_d);

  uint iter(0);
  while(iter++ < max_iter &&
        (p.getTranslationVector().frobeniusNorm() > min_lin_error ||
         std::abs(p.getThetaUVector().getTheta()) > min_ang_error))
  {
    const auto R{M.getRotationMatrix()};
    v.insert(0, R * p.getTranslationVector());
    v.insert(3, R * static_cast<vpColVector>(p.getThetaUVector()));
    q0 += lambda * fJw(q0).pseudoInverse() * v;
    for(uint i = 0; i < dofs; ++i)
      q0[i] = std::clamp(q0[i], q_min[i], q_max[i]);
    M = fMw(q0);
    p.buildFrom(M.inverse() * fMw_d);
  }
  return q0;
}

// set articular velocity
void Robot::setJointVelocity(const vpColVector &_velocity)
{
  if(mode() != ControlMode::VELOCITY_P2P && mode() != ControlMode::VELOCITY_MANUAL)
    return;

  if(_velocity.getRows() != dofs)
  {
    std::cout << "Robot::setVelocity: bad dimension" << std::endl;
    return;
  }
  // if v_max_, scales velocity (educational goal)
  double scale = 1.;
  if(v_max.size()!=0)
  {
    for(uint i=0;i<dofs;++i)
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

std::array<double, 12> Robot::explodeMatrix(const vpHomogeneousMatrix &fMe_des) const
{
  const auto oMw{fM0.inverse() * fMe_des * wMe.inverse()};

  std::array<double, 12> elements;
  auto elem{elements.begin()};
  for(uint col = 0; col < 4; ++col)
  {
    for(uint row = 0; row < 3; ++row)
    {
      *(elem++) = oMw[row][col];
      //elem++;
    }
  }
  return elements;
}

std::array<double, 3> Robot::explodeTranslation(const vpHomogeneousMatrix &fMe_des) const
{
  const auto oTw{(fM0.inverse() * fMe_des * wMe.inverse()).getTranslationVector()};
  return {oTw[0], oTw[1], oTw[2]};
}

std::array<double, 9> Robot::explodeWristMatrix(const vpHomogeneousMatrix &fMe_des,
                                                const vpRotationMatrix &R03) const
{
  const auto R06{fM0.getRotationMatrix().t() * fMe_des.getRotationMatrix() * wMe.getRotationMatrix().t()};
  const auto R36{R03.t() * R06};
  std::array<double, 9> elements;
  auto elem{elements.begin()};
  for(uint col = 0; col < 3; ++col)
  {
    for(uint row = 0; row < 3; ++row)
    {
      *elem = R36[row][col];
      elem++;
    }
  }
  return elements;
}


void Robot::addCandidate(std::vector<double> q_candidate) const
{
  if(q_candidate.size() != dofs)
  {
    std::cerr << "WARNING in InverseGeometry: adding a candidate with wrong dofs"
              << std::endl;
  }
  else
    q_candidates.push_back(q_candidate);
}

bool Robot::inAngleLimits(std::vector<double> &q) const
{
  for(uint i = 0; i < q.size(); ++i)
  {
    if(!angleOK(q[i], q_min[i], q_max[i]))
      return false;
  }
  return true;
}


vpColVector Robot::bestCandidate(const vpColVector &q0, std::vector<double> weights) const
{
  // returns position from q_candidates closest to q0

  // is no candidates just return q0
  if(q_candidates.empty())
  {
    std::cerr << "No inverse geometry candidates" << std::endl;
    return q0;
  }

  // remove out of bounds
  const auto last_inbounds{std::remove_if(q_candidates.begin(), q_candidates.end(),
                                          [&](auto &candidate){return !inAngleLimits(candidate);})};

  if(last_inbounds == q_candidates.begin())
  {
    std::cerr << "Inverse geometry candidates are out of joint limits" << std::endl;
    q_candidates.clear();
    return q0;
  }

  weights.resize(q0.size(), 1);

  const auto distanceToq0 = [&](const vpColVector &q1, const vpColVector &q2)
  {
    // return whether q1 or q2 is closer to q0
    double d1{}, d2{};
    for(uint i = 0; i < q0.size(); ++i)
    {
      d1 += weights[i]*vpMath::sqr(q0[i]-q1[i]);
      d2 += weights[i]*vpMath::sqr(q0[i]-q2[i]);
    }
    return d1 < d2;
  };

  const auto closest{*std::min_element(q_candidates.begin(), last_inbounds, distanceToq0)};
  q_candidates.clear();
  return closest;
}
