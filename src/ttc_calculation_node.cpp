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

    ROS_INFO("STARTING NODE");

    // Main loop
    ros::Rate rate(10); // 10 Hz
    while (ros::ok())
    {
        ros::spinOnce(); // Process incoming messages

        // Populate sample data
        TTCComputer::Sample sample;
        double roll_ego, pitch_ego, yaw_ego;
        double roll, pitch, yaw;

        convertQuaternionToEuler(lastOdomAGV1.pose.pose.orientation, roll_ego, pitch_ego, yaw_ego);
        convertQuaternionToEuler(lastOdomAGV2.pose.pose.orientation, roll, pitch, yaw);

        sample.vehicle_i.position = {lastOdomAGV1.pose.pose.position.x,
                                     lastOdomAGV1.pose.pose.position.y};

        sample.vehicle_i.velocity = {lastOdomAGV1.twist.twist.linear.x * cos(yaw_ego),
                                     lastOdomAGV1.twist.twist.linear.x * sin(yaw_ego)};

        sample.vehicle_i.heading = {cos(yaw_ego), sin(yaw_ego)};

        sample.vehicle_i.length = 1.853;

        sample.vehicle_i.width = 1.855;

        sample.vehicle_j.position = {lastOdomAGV2.pose.pose.position.x,
                                     lastOdomAGV2.pose.pose.position.y};

        sample.vehicle_j.velocity = {lastOdomAGV2.twist.twist.linear.x * cos(yaw),
                                     lastOdomAGV2.twist.twist.linear.x * sin(yaw)};

        sample.vehicle_j.heading = {cos(yaw), sin(yaw)};

        sample.vehicle_j.length = 1.853;

        sample.vehicle_j.width = 1.855;

        // Use lastOdomAGV1 and lastOdomAGV2 to populate sample
        // You will need to convert from the ROS Odometry message to your internal data structure

        // Now call computeTTC
        try
        {
            double ttc = TTCComputer::computeTTC(sample);
            ROS_INFO("Time to Collision: %f", ttc);
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("An error occurred: %s", e.what());
        }

        rate.sleep(); // Sleep to maintain loop rate
    }

    return 0;
}
