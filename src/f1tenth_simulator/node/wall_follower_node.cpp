#include <ros/ros.h>

//Message Types
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <sensor_msgs/LaserScan.h>

// Other Libraries
#include <math.h>
#include <ros/console.h>

class Wall_Follower {
private:
    ros::NodeHandle n;

    // Create Subscribers
    ros::Subscriber LaserScan;

    ros::Publisher DriveMessage;

    // Define any const variables
    const double Angle_A = 60;
    const double Angle_A_Rad = Angle_A * (M_PI / 180);

    const double Kp = 2.0;
    const double Ki = 0.0;
    const double Kd = 0.0;

    double integral_err = 0.0;
    ros::Time prev_time;
    ros::Time current_time;

    bool Once = true;

    // Distance from wall setpoint
    const double SetPoint = 1;

    // TODO: Precompute cos angle of LS data

    //

public:
    Wall_Follower() {
        // Start Node
        n = ros::NodeHandle("~");

        // Get Topic Names
        std::string laser_scan_topic;
        n.getParam("scan_topic", laser_scan_topic);

        ROS_INFO_STREAM("Wall Follower Node Launched");

        // TODO: Set up subscribers and publishers
        LaserScan = n.subscribe(laser_scan_topic, 1, &Wall_Follower::LS_callback, this);

        //Wall_Follow_Drive
        DriveMessage = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);


    }

    // Create callback function for Laser Scan info

    void LS_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){

        //ROS_INFO_STREAM("WF: LS_Callback");
        // Obtain LS distance at exactly East of Car (B), and 60 degrees anticlockwise of this point (A)
        // Work out at which index 0 and 60 degrees lies
        double B = scan_msg->ranges[260];
        double A = scan_msg->ranges[((60 * (M_PI / 180)) / scan_msg->angle_increment) + 260];


        // Calculate Alpha (Alpha), the angle between exactly east of the car, and the shortest distance to the wall

        double alpha = std::atan(   (A * cos(Angle_A_Rad) - B)   /    (A * sin(Angle_A_Rad)) );   // Result in radians

        // Calculate the distance of the LIDAR sensor from the wall (Dt)

        double Dt = B * cos(alpha);

        // Calculate Dt at next time step, Distance covered in next time step (L),
        // TODO: Calculate distance at next time step based on speed

        // Current Speed * Time Step
        double L =  1 * 1 ;

        double Dt_1 = Dt + L * sin(alpha);

        // Calculate Error, Set Point (SetPoint) - Dt_1

        double Error = SetPoint - Dt_1;


        /// Debug ///
        ROS_INFO_STREAM("Beam 260: " << scan_msg->ranges[700] << " Beam @ 60 to X Axis: " << scan_msg->ranges[((60 * (M_PI / 180)) / scan_msg->angle_increment) + 260] << " Dt: " << Dt);


        // Set first prev_time
        if(Once){
            // TODO: Fix below
            //ros::Time prev_time = scan_msg->header;
            Once = false;
        }

        // Call PID Function
        double U_t =  PID_Calc(Error);

        /// Debug ///
        //ROS_INFO_STREAM("Dt: " << Dt_1 << " Error: " << Error << " Control Eff: " << U_t);
        //ROS_INFO_STREAM("Control Effort: " << U_t);

        // Call Speed Command
        double speed = car_speed(U_t);

        // Call Publish Function
        //publish_command(speed, U_t);

    }

    double PID_Calc(double &error){

        // Get Current Time
        ros::Time current_time = ros::Time::now();

        // Calculate dt
        double dt = (current_time - prev_time).toSec();

        // Sum Integral
        integral_err += error * dt;

        // Calculate Control Effort (Steering Angle)
        double U_t = Kp * error + Ki * integral_err;

        // Set prev_time equal to cur_time
        prev_time = current_time;

        // Return Control Output
        return U_t;

    }

    double car_speed(double &steering_angle){

        steering_angle = std::abs(steering_angle);

        if(steering_angle < 10 && steering_angle >= 0){
            return 0.5;
        }
        else if(steering_angle < 20 && steering_angle >= 10){
            return 0.5;
        }
        else {
            return 0.5;
        }

    }

    void publish_command(double &speed, double &steering_angle){

        // Construct Message to Send

        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        drive_msg.speed = speed;
        drive_msg.steering_angle = steering_angle;

        //ROS_INFO_STREAM("Speed: " << speed << " Steer Angle: " << steering_angle);

        drive_st_msg.drive = drive_msg;

        //publish
        DriveMessage.publish(drive_st_msg);

    }

};



int main(int argc, char ** argv) {
    ros::init(argc, argv, "Wall_Follower_Node");
    Wall_Follower wf;
    //ros::spin();

    ros::Rate loopRate(15);

    while (ros::ok()) {
        loopRate.sleep();
        ros::spinOnce();
    }

    return 0;
}