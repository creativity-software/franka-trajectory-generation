#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "trajectory_gen/trajectory_generator.hpp"
#include "trajectory_gen/velocity_profile.hpp"
#include <iostream>
#include <geometry_msgs/Point.h>
#include <franka_msgs/FrankaState.h>
#include <memory>
#include <stack>


bool initialised = false;
geometry_msgs::Point initialPosition;
int changeToCircular = false;
double previousZForce = 0.0d;


void getInitialPositionCallback(const franka_msgs::FrankaState::ConstPtr& msg)
{
    initialPosition.x = msg->O_T_EE[12];
    initialPosition.y = msg->O_T_EE[13];
    initialPosition.z = msg->O_T_EE[14];
    initialised = true;
}


void getForceCallback(const franka_msgs::FrankaState::ConstPtr& msg)
{
    previousZForce = msg->O_F_ext_hat_K[2];
}

const trajectory_generator::triple NORM(0 , 0, 1);
const double RADIUS = 0.1d;  


std::shared_ptr<trajectory_generator::LinearTrajectory> createLinearTrajectoryDown(geometry_msgs::PoseStamped &msg) {
    trajectory_generator::triple start(0, 0, 0);
    trajectory_generator::triple end(0, 0, -msg.pose.position.z);
    trajectory_generator::triple c(
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z
    );

    
    trajectory_generator::LinearTrajectory linearTrajectory (c, NORM, start, end, 2, 20);
    linearTrajectory.setForceThreshold(2);
    return std::make_shared<trajectory_generator::LinearTrajectory>(linearTrajectory);
}

std::shared_ptr<trajectory_generator::CircleTrajectory2d> createCirclularTrajectory(geometry_msgs::PoseStamped &msg) {
    trajectory_generator::triple c(
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z
    );
    trajectory_generator::CircleTrajectory2d circularTrajectory(c, NORM, RADIUS, 2, 10);
    return std::make_shared<trajectory_generator::CircleTrajectory2d>(circularTrajectory);
}

std::shared_ptr<trajectory_generator::LinearTrajectory> createLinearTrajectory(geometry_msgs::PoseStamped &msg, double z) {
    trajectory_generator::triple start(0, 0, 0);
    trajectory_generator::triple end(-RADIUS, 0, z - msg.pose.position.z);
    trajectory_generator::triple c(
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z
    );

    
    trajectory_generator::LinearTrajectory linearTrajectory (c, NORM, start, end, 2, 20);
    return std::make_shared<trajectory_generator::LinearTrajectory>(linearTrajectory);
}


enum RobotState {
    DOWN_LINEAR,
    CIRCULAR,
    UP_LINEAR
};


int main(int argc, char **argv)
{
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "trajectory_gen");
    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    ros::Publisher trajectory_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_example_controller/equilibrium_pose", 1000);
    double rate = 1000;
    ros::Rate loop_rate(rate);
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    geometry_msgs::PoseStamped msg;
    int seq = 0;

    ROS_INFO("Starting trajectory generation");
    double time = 0.0;

    trajectory_pub.publish(msg);

    ros::Subscriber sub = n.subscribe("/franka_state_controller/franka_states", 1000, getInitialPositionCallback);
    while (!initialised) {
        ros::spinOnce();
    }
    sub.shutdown();

    msg.pose.position.x = initialPosition.x;
    msg.pose.position.y = initialPosition.y;
    msg.pose.position.z = initialPosition.z;
    msg.header.frame_id = "panda_link6";

    trajectory_pub.publish(msg);

    std::shared_ptr<trajectory_generator::Trajectory> trajectory = createLinearTrajectoryDown(msg);
    RobotState state = DOWN_LINEAR;

    ros::Subscriber forceSub = n.subscribe("/franka_state_controller/franka_states", 1000, getForceCallback);

    while (ros::ok())
    {
        // std::shared_ptr<trajectory_generator::Trajectory> trajectory = trajectories.top();
        double dt = 1.0d / rate;
        trajectory->update(dt, previousZForce);
        geometry_msgs::Point current_point = trajectory->getPoint();

        msg.pose.position.y = current_point.y;
        msg.pose.position.x = current_point.x;
        msg.pose.position.z = current_point.z;
        msg.pose.orientation.w = 0;
        msg.pose.orientation.x = 1;
        msg.pose.orientation.y = 0;
        msg.pose.orientation.z = 0;

        time = time + 1 / rate;
        seq += 1;

        if (trajectory->isEnded() && state == DOWN_LINEAR) {
            std::cout << "Change to trajectory: " << trajectory->getName() << std::endl;
            trajectory = createCirclularTrajectory(msg);
            state = CIRCULAR;
            continue;
        } else if (trajectory->isEnded() && state == CIRCULAR) {
            std::cout << "Change to trajectory: " << trajectory->getName() << std::endl;
            trajectory = createLinearTrajectory(msg, initialPosition.z);
            state = UP_LINEAR;
            continue;
        } else if (trajectory->isEnded() && state == UP_LINEAR) {
            break;
        } 
        
        msg.header.seq = seq;
        msg.header.frame_id = "panda_link6";


        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        trajectory_pub.publish(msg);

        // sleep for 0.5 seconds before new calculation
        ros::Duration(0.05).sleep();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
