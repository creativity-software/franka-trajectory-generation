#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "trajectory_gen/trajectory_generator.hpp"
#include "trajectory_gen/velocity_profile.hpp"
#include <iostream>
#include <geometry_msgs/Point.h>
#include <franka_msgs/FrankaState.h>
#include <memory>


bool initialised = false;
geometry_msgs::Point initialPosition;
int changeToCircular = false;
double previousZForce = 0.0d;


void getInitialPositionCallback(const franka_msgs::FrankaState::ConstPtr& msg)
{
    // [0.7065002343568823, -0.7077127227431425, 0.00034773719708732474, 0.0, -0.7077127999605454, -0.7065000660134543, 0.0004994947345348464, 0.0, -0.00010782242587599956, -0.0005989912124100002, -0.9999998147919088, 0.0, 0.30691149243109883, -9.525381826038915e-05, 0.5902920594835598, 1.0]
    // x = 0.30691149243109883,  y= -9.525381826038915e-05, z = 0.5902920594835598
    // msg->O_F_ext_hat_K;//force at the e.e. take the first three components and do euclidean norm 
    // msg->O_T_EE; //transformation matrix from base to end effector. take position matrix

    //position end effector. for initialization
    //external force on the end effector. use to check when in contact with table
    // std::cout << "In state callback , z is: " << msg->O_T_EE[14]  << std::endl;
    initialPosition.x = msg->O_T_EE[12];
    initialPosition.y = msg->O_T_EE[13];
    // initialPosition.z = msg->O_T_EE[14];
    // TODO: remove once the whole loop is done
    initialPosition.z = 0.5;
    initialised = true;
//   ROS_INFO("I heard",
// //    msg->c_str()
//   );

}


void getForceCallback(const franka_msgs::FrankaState::ConstPtr& msg)
{
    // [0.7065002343568823, -0.7077127227431425, 0.00034773719708732474, 0.0, -0.7077127999605454, -0.7065000660134543, 0.0004994947345348464, 0.0, -0.00010782242587599956, -0.0005989912124100002, -0.9999998147919088, 0.0, 0.30691149243109883, -9.525381826038915e-05, 0.5902920594835598, 1.0]
    // x = 0.30691149243109883,  y= -9.525381826038915e-05, z = 0.5902920594835598
    // msg->O_F_ext_hat_K;//force at the e.e. take the first three components and do euclidean norm 
    // msg->O_T_EE; //transformation matrix from base to end effector. take position matrix
    
    // curr = 9.5, prev = 0
    // curr = 9.2 -  8.2 = 9.5 > threshold
    // if (msg->O_F_ext_hat_K[2] > 1.0d) {
    //     changeToCircular = true;
    //     std::cout << "In state callback , z force is: " << msg->O_F_ext_hat_K[2]  << std::endl;
    // }

    previousZForce = msg->O_F_ext_hat_K[2];
    //position end effector. for initialization
    //external force on the end effector. use to check when in contact with table
    // initialPosition.x = msg->O_T_EE[12];
    // initialPosition.y = msg->O_T_EE[13];
    // initialPosition.z = msg->O_T_EE[14];
    // TODO: remove once the whole loop is done
    // initialPosition.z = 0.5;
    // initialised = true;
//   ROS_INFO("I heard",
// //    msg->c_str()
//   );

}


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
    // bool hitsGround = msg.pose.position.z == 0;

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
    // ros::spin();

    trajectory_pub.publish(msg);

    std::cout << "Initial position z " << msg.pose.position.z << std::endl;
    // double p_start, double p_end, double q_dot_max, double q_double_dot
    // velocity_profile::Profile profile(0, 10, 4, 10);
    trajectory_generator::triple start(0, 0, 0);
    trajectory_generator::triple end(0, 0, -msg.pose.position.z);
    // LT(go_down) -> LT(go_to_start) -> CT(rotate) -> LT(go_to_start_down) -> LT(go_up)
    trajectory_generator::LinearTrajectory linearTrajectory (
         std::make_tuple(
         msg.pose.position.x,
         msg.pose.position.y,
         msg.pose.position.z), 
         std::make_tuple(0 , 0,1) , start, end, 2, 20);
    trajectory_generator::CircleTrajectory2d circularTrajectory(
    std::make_tuple(
    msg.pose.position.x,
    msg.pose.position.y,
    0)
        , std::make_tuple(0 , 0,1), 0.1, 2, 10);
    

    std::unique_ptr<trajectory_generator::Trajectory> trajectory = std::make_unique<trajectory_generator::LinearTrajectory>(linearTrajectory);
    bool isFinalTrajectory = false;
    // ros::Subscriber forceSub = n.subscribe("/franka_state_controller/franka_states", 1000, getForceCallback);
    while (ros::ok())
    {
        double dt = 1.0d / rate;
        trajectory->update(dt);
        geometry_msgs::Point current_point = trajectory->getPoint();
        // std::cout << "Update trajectory, " << (isFinalTrajectory ? "circular" : "linear")  << "- position x: " << current_point.x 
        // << ", y: " << current_point.y << ", z: " << current_point.z << "\n"; 
        // std::cout << "Update position velocity: " << profile.getQDot() << ", update time: " << profile.getTime()  << ", current position" << profile.getQ() << "\n"; 
        
        msg.pose.position.y = current_point.y;
        msg.pose.position.x = current_point.x;
        msg.pose.position.z = current_point.z;
        msg.pose.orientation.w = 0;
        msg.pose.orientation.x = 1;
        msg.pose.orientation.y = 0;
        msg.pose.orientation.z = 0;

        time = time + 1 / rate;
        seq += 1;
        // msg.pose.position.x -= 0.1;
        if (trajectory->isEnded() && !isFinalTrajectory) {
            isFinalTrajectory = true;
            trajectory =  std::make_unique<trajectory_generator::CircleTrajectory2d>(circularTrajectory);
            std::cout << "Change to rotation" << std::endl;
            continue;
        } else if (trajectory->isEnded() && isFinalTrajectory) {
            std::cout << "Change to end" << std::endl;
            break;
        }
    
        // ROS_INFO("Starting trajectory generation" + seq);
        // std::cout << "Hello b. " <<  msg.pose.position.x << "\n";
        msg.header.seq = seq;
        msg.header.frame_id = "panda_link6";

        // std::cout << msg.pose.position.x << std::endl;

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
