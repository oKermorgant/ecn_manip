#include <robot_init.h>

using namespace std;

int main(int argc, char ** argv)
{
    // initialize robot class and get DOF's
    auto robot = ecn::initRobot(argc, argv, 100);
    const unsigned n = robot->getDofs();

    // main variables
    unsigned int iter = 0;      // iteration count
    vpColVector q(n);           // joint position
    vpColVector p(n);           // operational pose
    vpColVector qCommand(n);    // joint position setpoint
    vpColVector vCommand(n);    // joint velocity setpoint



    vpMatrix J;
    unsigned int mode;
    
    // main control loop
    while(robot->ok())
    {
        // get current joint positions
        q = robot->jointPosition();
        cout << "Current joint position : " << q.t() << endl;

        // Direct Kinematics for end-effector
        p = robot->fMe(q);

        // Current control mode
        mode = robot->mode();






    }
}
