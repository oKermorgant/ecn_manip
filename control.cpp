#include <robot_init.h>

using namespace std;

int main(int argc, char ** argv)
{
    // initialize robot class and get DOF's
    auto robot = ecn::initRobot(argc, argv, 100);
    const unsigned n = robot->getDofs();

    // main variables
    unsigned int iter = 0;          // iteration count
    vpColVector q(n);               // joint position
    vpPoseVector p;                 // operational pose
    vpColVector qCommand(n);        // joint position setpoint
    vpColVector vCommand(n);        // joint velocity setpoint

    vpMatrix J;
    vpHomogeneousMatrix M;          // current pose
    vpHomogeneousMatrix M0, Md, Mi; // previous, final and current desired poses
    vpPoseVector pd;                // desired pose
    vpColVector v;                  // desired operational velocity
    float step_count;
    const unsigned int steps = 100;

    
    // main control loop
    while(robot->ok())
    {
        // update desired pose if has changed
        if(robot->newRef())
        {
            step_count = 0;
            Md = robot->Md();
            M0 = robot->M0();
            pd.buildFrom(Md);
        }

        // get current joint positions
        q = robot->jointPosition();
        cout << "Current joint position : " << q.t() << endl;

        // Direct Geometry for end-effector
        M = robot->fMe(q);
        p.buildFrom(M);

        switch(robot->mode())
        {
        case 0:
            // just print the Direct Geometric Model
            cout << "DGM: " << p.t() << endl;
            break;

        case 1:
            // follow a given operational velocity
            v = robot->vw();

            // TODO: compute vCommand

            robot->setJointVelocity(vCommand);
            break;

        case 2:
            // find the Inverse Geometry to reach Md
            qCommand = robot->inverseGeometry(Md, q);
            robot->setJointPosition(qCommand);
            break;

        case 3:
            // go from M0 to Md in 100 steps

            // TODO: compute qCommand from M0, Md, step_count and steps

            robot->setJointPosition(qCommand);
            step_count++;
            break;

        case 4:
            // go to Md using operational velocity

            // TODO: compute joint velocity command



            robot->setJointVelocity(vCommand);
            break;
        }
    }
}
