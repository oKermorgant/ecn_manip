#ifndef ECNROBOTBASE_H
#define ECNROBOTBASE_H

#include <visp/vpColVector.h>
#include <visp/vpQuaternionVector.h>
#include <visp/vpSubMatrix.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <memory>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <urdf/model.h>
#include <ecn_manip/RobotConfig.h>

/*
  Generic Robot Class
  Gives ROS interface
*/

using std::string;
using std::cout;
using std::endl;

namespace ecn
{

class Robot {


public:

    enum GUI_MODE
    {
        MODE_POSITION_MANUAL,
        MODE_VELOCITY_MANUAL,
        MODE_DIRECT_P2P,
        MODE_INTERP_P2P,
        MODE_STRAIGHT_LINE_P2P,
        MODE_VELOCITY_P2P
    };


    // constructor
    Robot(const urdf::Model &model, double rate = 100);
    // get number of dof
    inline unsigned int getDofs() const {return dofs;}

    // get articular position
    inline vpColVector jointPosition() const {return q_;}

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
        for(int i = 0; i < dofs; ++i)
            qr[i] = ((q_max[i] - q_min[i])*rand())/RAND_MAX + q_min[i];
        return qr;
    }
    vpColVector vMax() const {return v_max_;}
    vpColVector aMax() const {return 0.5*v_max_;}

    // prints translation + roll pitch yaw
    void checkPose(const vpHomogeneousMatrix &M);

    // desired poses
    vpHomogeneousMatrix M0() const {return fwd?M1_:M2_;}
    vpHomogeneousMatrix Md() const {return fwd?M2_:M1_;}
    // desired twist
    vpColVector vw() const
    {
        vpColVector v(6);
        v[0] = desired_twist.linear.x;
        v[1] = desired_twist.linear.y;
        v[2] = desired_twist.linear.z;
        v[3] = desired_twist.angular.x;
        v[4] = desired_twist.angular.y;
        v[5] = desired_twist.angular.z;
        return v;
    }

    int mode() const {return config.mode;}

    bool ok();

    double lambda()
    {
        return config.lambda;
    }

    bool newRef() {return new_ref;}

    vpColVector iterativeIK(vpColVector q0, const vpHomogeneousMatrix &Md);

    static vpHomogeneousMatrix intermediaryPose(vpHomogeneousMatrix M1, vpHomogeneousMatrix M2, double a);

    // stop motion
    inline void stopMotion() {setJointPosition(q_);}

    void displayFrame(const vpHomogeneousMatrix &M, std::string name = "estim_DG") const
    {
        br->sendTransform(buildTransformStamped(M, name));
    }

    // inverse geometry methods
    void addCandidate(std::vector<double> q_candidate) const;
    vpColVector bestCandidate(const vpColVector &q0) const;

    // to be overloaded
    virtual void init_wMe() = 0;
    virtual vpHomogeneousMatrix fMw(const vpColVector &q) const = 0;
    virtual vpColVector inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const = 0;
    virtual vpMatrix fJw(const vpColVector &q) const = 0;

    // end-effector / wrist DK and Jacobian
    virtual vpHomogeneousMatrix fMe(const vpColVector &q) const
    {
        return fMw(q) * wMe;
    }
    virtual vpMatrix fJe(const vpColVector &q) const;

protected:    
    unsigned int dofs, iter_cnt = 0;
    ecn_manip::RobotConfig config;
    vpColVector q_;

    // joint limits
    std::vector<double> q_max, q_min;
    vpColVector v_max_;
    mutable std::vector<std::vector<double> > q_candidates;

    // 2 desired poses for switching
    vpHomogeneousMatrix M1_, M2_;
    bool fwd = true, new_ref = true;
    vpHomogeneousMatrix wMe, bM0;   // constant matrices if needed

    // ROS functions
    std::unique_ptr<ros::Rate> rate;
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Publisher cmd_pub, desired_pose_pub;
    ros::Subscriber position_sub, twist_sub, config_sub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tl;
    sensor_msgs::JointState joint_cmd;
    geometry_msgs::Twist desired_twist;
    std_msgs::Float32MultiArray desired_pose;
    double t_gt;    // last time we checked the ground truth

    void onReadPosition(const sensor_msgs::JointState::ConstPtr& _msg);

    void onReadTwist(const geometry_msgs::Twist &_msg)
    {
        desired_twist = _msg;
    }

    void onReadConfig(const ecn_manip::RobotConfig &_msg)
    {
        config = _msg;
    }

    geometry_msgs::TransformStamped buildTransformStamped(const vpHomogeneousMatrix &M, const std::string &frame) const
    {
        const vpTranslationVector t(M);
        vpQuaternionVector qu; M.extract(qu);
        geometry_msgs::TransformStamped transform;
        transform.transform.translation.x = t[0];
        transform.transform.translation.y = t[1];
        transform.transform.translation.z = t[2];
        transform.transform.rotation.x = qu.x();
        transform.transform.rotation.y = qu.y();
        transform.transform.rotation.z = qu.z();
        transform.transform.rotation.w = qu.w();
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "base_link";
        transform.child_frame_id  = frame;
        return transform;
    }

    void updateDesiredPose()
    {
        iter_cnt++;
        const int iter_switch = config.switch_time / rate->expectedCycleTime().toSec();

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

        br->sendTransform(buildTransformStamped(Md(), "target"));
    }
};

}

#endif
