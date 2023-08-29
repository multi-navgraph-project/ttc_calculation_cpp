#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ttc_calculation_cpp/ttc_calculation_cpp.h> // Make sure the include path is correct
#include <iostream>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Global variables to store last known positions of both AGVs
nav_msgs::Odometry lastOdomAGV1;
nav_msgs::Odometry lastOdomAGV2;
nav_msgs::Odometry lastOdomAGV3;

// Callback for AGV 1
void odomCallbackAGV1(const nav_msgs::Odometry::ConstPtr &msg)
{
    lastOdomAGV1 = *msg;
}

// Callback for AGV 2
void odomCallbackAGV2(const nav_msgs::Odometry::ConstPtr &msg)
{
    lastOdomAGV2 = *msg;
}

void odomCallbackAGV3(const nav_msgs::Odometry::ConstPtr &msg)
{
    lastOdomAGV3 = *msg;
}

void convertQuaternionToEuler(const geometry_msgs::Quaternion &q, double &roll, double &pitch, double &yaw)
{
    // Convert geometry_msgs::Quaternion to tf2::Quaternion
    tf2::Quaternion tf2_quaternion;
    tf2::fromMsg(q, tf2_quaternion);

    // Convert to Euler angles
    tf2::Matrix3x3(tf2_quaternion).getRPY(roll, pitch, yaw);
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "ttc_computer_node");
    ros::NodeHandle nh;

    // Initialize subscribers
    ros::Subscriber subAGV1 = nh.subscribe("agv_1/odom", 1, odomCallbackAGV1);
    ros::Subscriber subAGV2 = nh.subscribe("agv_2/odom", 1, odomCallbackAGV2);
    ros::Subscriber subAGV3 = nh.subscribe("agv_3/odom", 1, odomCallbackAGV3);

    ROS_INFO("STARTING NODE");

    // Main loop
    ros::Rate rate(10); // 10 Hz
    while (ros::ok())
    {
        ros::spinOnce(); // Process incoming messages

        // Populate sample[0] data
        std::vector<TTCComputer::Sample> sample;
        double roll_ego, pitch_ego, yaw_ego;
        double roll, pitch, yaw;
        double roll_2, pitch_2, yaw_2;

        convertQuaternionToEuler(lastOdomAGV1.pose.pose.orientation, roll_ego, pitch_ego, yaw_ego);
        convertQuaternionToEuler(lastOdomAGV2.pose.pose.orientation, roll, pitch, yaw);
        convertQuaternionToEuler(lastOdomAGV3.pose.pose.orientation, roll_2, pitch_2, yaw_2);

        sample[0].vehicle_i.position = {lastOdomAGV1.pose.pose.position.x,
                                        lastOdomAGV1.pose.pose.position.y};

        sample[0].vehicle_i.velocity = {lastOdomAGV1.twist.twist.linear.x * cos(yaw_ego),
                                        lastOdomAGV1.twist.twist.linear.x * sin(yaw_ego)};

        sample[0].vehicle_i.heading = {cos(yaw_ego), sin(yaw_ego)};

        sample[0].vehicle_i.length = 1.853;

        sample[0].vehicle_i.width = 1.855;

        sample[0].vehicle_j.position = {lastOdomAGV2.pose.pose.position.x,
                                        lastOdomAGV2.pose.pose.position.y};

        sample[0].vehicle_j.velocity = {lastOdomAGV2.twist.twist.linear.x * cos(yaw),
                                        lastOdomAGV2.twist.twist.linear.x * sin(yaw)};

        sample[0].vehicle_j.heading = {cos(yaw), sin(yaw)};

        sample[0].vehicle_j.length = 1.853;

        sample[0].vehicle_j.width = 1.855;

        sample[1].vehicle_i.position = {lastOdomAGV1.pose.pose.position.x,
                                        lastOdomAGV1.pose.pose.position.y};

        sample[1].vehicle_i.velocity = {lastOdomAGV1.twist.twist.linear.x * cos(yaw_ego),
                                        lastOdomAGV1.twist.twist.linear.x * sin(yaw_ego)};

        sample[1].vehicle_i.heading = {cos(yaw_ego), sin(yaw_ego)};

        sample[1].vehicle_i.length = 1.853;

        sample[1].vehicle_i.width = 1.855;

        sample[1].vehicle_j.position = {lastOdomAGV3.pose.pose.position.x,
                                        lastOdomAGV3.pose.pose.position.y};

        sample[1].vehicle_j.velocity = {lastOdomAGV3.twist.twist.linear.x * cos(yaw_2),
                                        lastOdomAGV3.twist.twist.linear.x * sin(yaw_2)};

        sample[1].vehicle_j.heading = {cos(yaw_2), sin(yaw_2)};

        sample[1].vehicle_j.length = 1.853;

        sample[1].vehicle_j.width = 1.855;

        // Use lastOdomAGV1 and lastOdomAGV2 to populate sample[0]
        // You will need to convert from the ROS Odometry message to your internal data structure

        // Now call computeTTC
        try
        {
            double ttc_1 = TTCComputer::computeTTC(sample[0]);
            double ttc_2 = TTCComputer::computeTTC(sample[1]);
            ROS_INFO("Time to Collision [ego - 1]: %f | Time to Collision [ego - 2]: %f", ttc_1, ttc_2);
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("An error occurred: %s", e.what());
        }

        rate.sleep(); // Sleep to maintain loop rate
    }

    return 0;
}
