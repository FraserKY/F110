//
// Created by Fraser Kearsey on 17/03/2021.
//

#include <ros/ros.h>

// For printing messages
#include <ros/console.h>


#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
// TODO: include ROS msg type headers and libraries

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>

#include <ros/console.h>

#include <math.h>
#include <algorithm>

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
        n = ros::NodeHandle("~");
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
        n.getParam("scan_topic", laser_scan_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("break_drive_topic", break_tpc);
        n.getParam("break_bool_topic", break_bool_tpc);

        //ROS_INFO_STREAM("scan topic " << laser_scan_topic << "111");

        // TODO: create ROS subscribers and publishers
        //subscribe to laser_scan topic
        laser_scan_data = n.subscribe(laser_scan_topic, 100, &Safety::scan_callback, this);
        ROS_INFO_STREAM("created laser_scan subscriber");

        //subscribe to odom topic
        odom_data = n.subscribe(odom_topic, 1, &Safety::odom_callback, this);
        ROS_INFO_STREAM("created odom_topic subscriber");

        break_bool = n.advertise<std_msgs::Bool>(break_bool_tpc, 1);
        break_ = n.advertise<ackermann_msgs::AckermannDriveStamped>(break_tpc, 1);


    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed
        speed = odom_msg->twist.twist.linear.x;
        ///ROS_INFO_STREAM("Speed:" << speed);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // TODO: calculate TTC
        // Need to calculate TTC for each beam in a laser scan message
        double TTC;
        //ROS_INFO_STREAM("scan_callback");
        
        int length = sizeof(scan_msg->ranges);
        
        for (int i = 0; i <= length; i ++)
        {
            if(speed != 0.00){
                
                if(isinf(scan_msg->ranges[i]) == 0 && isnan(scan_msg->ranges[i]) == 0)
                    {
                    // ROS_INFO_STREAM(scan_msg->angle_min);
                    //Calculate TTC

                    double r_dot = speed * cos(scan_msg->angle_min + (scan_msg->angle_increment * i));
                    
                    //ROS_INFO_STREAM(scan_msg->angle_increment);
                    //ROS_INFO_STREAM(scan_msg->angle_min + (scan_msg->angle_increment * i));
                    //ROS_INFO_STREAM("r_dot: " << r_dot);

                    TTC = (scan_msg->ranges[i]) / std::max(-r_dot, 0.00);

                
                    ROS_INFO_STREAM("TTC: " << TTC);
                    ROS_INFO_STREAM("Speed: " << speed);
                    //ROS_INFO_STREAM(scan_msg->ranges[i]);
                    if(TTC < 0.8){
                        ROS_INFO_STREAM("TTC Limit");
                    }
            }
            

            }
           
        }



        // TODO: publish drive/brake message
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    //ros::Rate rate(1); //Pump callbacks once per second
    ros::spin();
    return 0;
}

