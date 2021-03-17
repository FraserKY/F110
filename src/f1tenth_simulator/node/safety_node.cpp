//
// Created by Fraser Kearsey on 17/03/2021.
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
// TODO: include ROS msg type headers and libraries

#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/bool.h>

#include <ros/console.h>

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;
    // TODO: create ROS subscribers and publishers

    // get laser scan and speed data
    ros::Subscriber laser_scan_data;
    ros::Subscriber odom_data;

    //create publishers for brake_bool and break topics
    ros::Publisher break_bool;
    ros::Publisher break_;

public:
    Safety() {
        n = ros::NodeHandle();
        speed = 0.0;
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages
        The subscribers should use the provided odom_callback and
        scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        */

        // Get topic names
        std::string laser_scan_topic, odom_topic, break_tpc, break_bool_tpc;
        n.getparam("scan_topic", laser_scan_topic);
        n.getparam("odom_topic", odom_topic);
        n.getparam("break_drive_topic", break_tpc);
        n.getparam("break_bool_topic", break_bool_tpc)

        // TODO: create ROS subscribers and publishers
        //subscribe to laser_scan topic
        laser_scan_data = n.subscribe(laser_scan_topic, 1, &Safety::scan_callback, this);

        //subscribe to odom topic
        odom_data = n.subscribe(odom_topic, 1, &Safety::odom_callback, this);

        break_bool = n.advertise<std_msgs::bool>(break_bool_tpc, 1);
        break_ = n.advertise<ackermann_msgs::





    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed
        speed = 0.0;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // TODO: calculate TTC

        // TODO: publish drive/brake message
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}

