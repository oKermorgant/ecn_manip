#ifndef ECNROBOTBASE_H
#define ECNROBOTBASE_H

#include <visp/vpColVector.h>
#include <visp/vpQuaternionVector.h>
#include <visp/vpSubMatrix.h>
#include <string>
#include <memory>
#include <ecn_manip/node.h>

/*
  Generic Robot Class
  ROS-agnostic
*/

using std::string;
using std::cout;
using std::endl;

namespace ecn
{

// numbered according to GUI ints
enum class ControlMode{POSITION_MANUAL=0,
                VELOCITY_MANUAL=1,
                DIRECT_P2P=2,
                POLYNOM_P2P=3,
                STRAIGHT_LINE_P2P=4,
                VELOCITY_P2P=5};

class Robot {

public:

  // default, non-ROS constructor for the example
  Robot()  {}

  Robot(std::unique_ptr<Node> &_node);
  // get number of dof
  inline unsigned int getDofs() const {return dofs;}

  // get articular position
  inline vpColVector jointPosition() const {return node->q;}

  // set articular position
  void setJointPosition(const vpColVector &_position);
  // set articular velocity
  void setJointVelocity(const vpColVector &_velocity);

  // get joint limits
  vpColVector jointMin() const {return q_min;}
  vpColVector jointMax() const  {return q_max;}
  vpColVector jointRand() const
  {
    vpColVector qr(dofs);
    for(uint i = 0; i < dofs; ++i)
      qr[i] = ((q_max[i] - q_min[i])*rand())/RAND_MAX + q_min[i];
    return qr;
  }
  vpColVector vMax() const {return v_max;}
  vpColVector aMax() const
  {
    return 0.5*vpColVector(v_max);
  }

  // prints translation + roll pitch yaw
  void checkPose(const vpHomogeneousMatrix &M);

  // desired poses
  vpHomogeneousMatrix M0() const {return fwd?M1_:M2_;}
  vpHomogeneousMatrix Md() const {return fwd?M2_:M1_;}
  // desired screw from GUI
  vpColVector guiVelocityScrew() const
  {
    return node->desired_twist;
  }

  bool ok();
  double time() const {return node->time();}

  inline double lambda() const
  {
    return static_cast<double>(node->config.lambda);
  }
  // which mode are we in?
  inline ControlMode mode() const {return static_cast<ControlMode>(node->config.mode);}

  bool newRef() {return new_ref;}

  virtual vpColVector iterativeIK(const vpHomogeneousMatrix &fMe_des, vpColVector q0) const;

  static vpHomogeneousMatrix intermediaryPose(vpHomogeneousMatrix M1, vpHomogeneousMatrix M2, double a);

  // stop motion
  inline void stopMotion() {setJointPosition(node->q);}

  void displayFrame(const vpHomogeneousMatrix &M, std::string name = "estim_DG") const
  {
    node->sendTransform(M, name);
  }

  // inverse geometry methods
  void addCandidate(std::vector<double> q_candidate) const;
  vpColVector bestCandidate(const vpColVector &q0) const;

  // to be overloaded
  virtual void init_wMe() = 0;
  virtual vpHomogeneousMatrix fMw(const vpColVector &q) const = 0;
  virtual vpColVector inverseGeometry(const vpHomogeneousMatrix &fMe_des, const vpColVector &q0) const = 0;
  virtual vpMatrix fJw(const vpColVector &q) const = 0;

  // end-effector / wrist DK and Jacobian
  virtual vpHomogeneousMatrix fMe(const vpColVector &q) const
  {
    return fMw(q) * wMe;
  }
  virtual vpMatrix fJe(const vpColVector &q) const;

protected:
  int iter_cnt = 0;

  // joint limits
  mutable std::vector<std::vector<double> > q_candidates;

  // explode inverse geometry to meaningful values
  std::array<double, 12> explodeMatrix(const vpHomogeneousMatrix &fMe_des) const;

  // 2 desired poses for switching
  vpHomogeneousMatrix M1_, M2_;
  bool fwd = true, new_ref = true;
  vpHomogeneousMatrix wMe, fM0;   // constant matrices if needed

  // ROS interface
  std::unique_ptr<Node> node;
  uint dofs;
  std::vector<double> q_max, q_min, v_max;
  double t_gt;    // last time we checked the ground truth

  void updateDesiredPose()
  {
    iter_cnt++;
    const int iter_switch = node->cycleLength();

    if(iter_cnt % iter_switch == 0)
    {
      new_ref = true;
      if(iter_cnt % (2*iter_switch) == 0)
      {
        fwd = true;
        iter_cnt = 0;
      }
      else
        fwd = false;
    }
    else
      new_ref = false;

    node->sendTransform(Md(), "target");
  }
};

}

#endif
