#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
// #include "trajectory_gen/trajectory_generator.hpp"
#include "trajectory_gen/velocity_profile.hpp"
#include <iostream>
#include <geometry_msgs/Point.h>

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
    double rate = 100;
    ros::Rate loop_rate(rate);
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    geometry_msgs::PoseStamped msg;
    int seq = 0;

    geometry_msgs::Point p;
    std::cout << "Point p.x is " << p.x << "\n";

    // double p_start, double p_end, double q_dot_max, double q_double_dot
    velocity_profile::Profile profile(0, 10, 20, 10);

    ROS_INFO("Starting trajectory generation");
    double time = 0.0;
    // bool hitsGround = msg.pose.position.z == 0;
    msg.pose.position.y = 5;
    msg.pose.position.x = 5;
    msg.pose.position.z = 5;
    trajectory_pub.publish(msg);

    
    while (ros::ok())
    {
        profile.update();
        std::cout << "Update position velocity: " << profile.getQDot() << ", update time: " << profile.getTime()  << ", current position" << profile.getQ() << "\n"; 
        time = time + 1 / rate;
        seq += 1;
        // msg.pose.position.x -= 0.1;
        if (profile.isEnded()) {
            break;
        } 
    
        // ROS_INFO("Starting trajectory generation" + seq);
        // std::cout << "Hello b. " <<  msg.pose.position.x << "\n";
        msg.header.seq = seq;
        msg.header.frame_id = "panda_link7";

        // std::cout << msg.pose.position.x << std::endl;

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        trajectory_pub.publish(msg);

        // ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}