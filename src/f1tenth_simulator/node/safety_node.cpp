//
// Created by Fraser Kearsey on 17/03/2021.
//

#include <ros/ros.h>

// For printing messages
#include <ros/console.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>

#include <ros/console.h>

#include <math.h>
#include <algorithm>

// For precomputing cosines and distance to car
#include "f1tenth_simulator/precompute.hpp"

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double car_speed;

    std::vector<double> cosines;
    std::vector<double> car_distances;

    double TTC_threshold_AEB;

    // TODO: create ROS subscribers and publishers

    // get laser scan and speed data
    ros::Subscriber laser_scan_data;
    ros::Subscriber odom_data;

    //create publishers for brake_bool and brake topics
    ros::Publisher brake_bool;
    ros::Publisher brake_;

public:
    Safety() {
        n = ros::NodeHandle("~");
        car_speed = 0.0;
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
        std::string laser_scan_topic, odom_topic, brake_tpc, brake_bool_tpc;
        n.getParam("scan_topic", laser_scan_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("brake_drive_topic", brake_tpc);
        n.getParam("brake_bool_topic", brake_bool_tpc);

        // Get TTC threshold
        n.getParam("ttc_threshold_AEB", TTC_threshold_AEB);

        //Get Variables required to pre-compute cosines and distance to car
        int scan_beams;
        double scan_fov, scan_ang_incr, wheelbase, width, scan_distance_to_base_link;
        //n.getParam("ttc_threshold", ttc_threshold);
        n.getParam("scan_beams", scan_beams);
        n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
        n.getParam("width", width);
        n.getParam("wheelbase", wheelbase);
        n.getParam("scan_field_of_view", scan_fov);
        scan_ang_incr = scan_fov / scan_beams;


        // Precompute cosine and distance to car at each angle of the laser scan
        cosines = racecar_simulator::Precompute::get_cosines(scan_beams, -scan_fov/2.0, scan_ang_incr);
        car_distances = racecar_simulator::Precompute::get_car_distances(scan_beams, wheelbase, width,
                                                      scan_distance_to_base_link, -scan_fov/2.0, scan_ang_incr);

        // create ROS subscribers and publishers
        //subscribe to laser_scan topic
        laser_scan_data = n.subscribe(laser_scan_topic, 100, &Safety::scan_callback, this);
        ROS_INFO_STREAM("created laser_scan subscriber");

        //subscribe to odom topic
        odom_data = n.subscribe(odom_topic, 1, &Safety::odom_callback, this);
        ROS_INFO_STREAM("created odom_topic subscriber");

        brake_bool = n.advertise<std_msgs::Bool>(brake_bool_tpc, 1);
        brake_ = n.advertise<ackermann_msgs::AckermannDriveStamped>(brake_tpc, 1);


    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // update current speed
        car_speed = odom_msg->twist.twist.linear.x;
        //ROS_INFO_STREAM("Speed:" << car_speed);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // calculate TTC (Time to Collision)
        // Need to calculate TTC for each beam in a laser scan message
        //bool engage_em_brake = 0;
        
        //int length;

        //ROS_INFO_STREAM("Number of values in LS ranges Array is: " << length);


        //  TODO: Create a function that checks for collision
        //Set a threshold for speed above which AEB is activated
        double abs_velocity = std::abs(car_speed);

        if(abs_velocity > 0.0)
        {
            
            // Loop through array of distance values from LIDAR and calc the TTC
            for (int i = 0; i <= 1080; i ++)   //scan_beams
            {
                // Do not process any values which are INF or NaN
                if(isinf(scan_msg->ranges[i]) == 0 && isnan(scan_msg->ranges[i]) == 0)
                {
                    // Calculate TTC
                    double r_dot = abs_velocity * cosines[i];
                    double TTC = (scan_msg->ranges[i] - car_distances[i]) / std::max(-r_dot, 0.00);
                    
                    //ROS_INFO_STREAM("Speed: " << speed);
                    //ROS_INFO_STREAM(scan_msg->ranges[i]);

                    
                    if(TTC <= (TTC_threshold_AEB * (abs_velocity / 8.26)) && isinf(TTC) == 0 && isnan(TTC) == 0){
                        ROS_INFO_STREAM("Actual TTC: " << TTC);
                        ROS_INFO_STREAM("TTC Limit: " <<  TTC_threshold_AEB * (abs_velocity / 8.26) );
                        engage_em_brake = true;
                        ROS_INFO ("Beam number" << i);
                        
                        // --- Engage Emergency Brake --- //
                        // publish drive/brake message
                        
                        // Create bool message
                        std_msgs::Bool brake_bool_msg;

                        brake_bool_msg.data = true;

                        // Send message to behaviour controller
                        brake_bool.publish(brake_bool_msg);

                        //create ackermann stamped message
                        // initialize message to be published
                        ackermann_msgs::AckermannDriveStamped drive_st_msg;
                        ackermann_msgs::AckermannDrive drive_msg;

                        drive_msg.speed = 0.0;
                        //drive_msg.steering_angle = 0;

                        drive_st_msg.drive = drive_msg;

                        //publish
                        brake_.publish(drive_st_msg);

                        //ROS_INFO_STREAM("Emergency Brake Engaged");

                        break;
                    }
                
                }

                
            }
           
        }


        

    }

};


    ///----- Other Functions ----///


int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    //ros::Rate rate(1); //Pump callbacks once per second
    ros::spin();
    return 0;
}

