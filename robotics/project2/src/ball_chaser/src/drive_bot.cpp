#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "simple_arm/DriveToTarget.h"


// Global motor command publisher.  Note that this is modified in main().
ros::Publisher motor_command_publisher;


// This callback function executes whenever the drive_bot service is requested
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res) {

    // Log receipt of request with parameter values
    ROS_INFO("DriveToTargetRequest received - lin_x:%1.2f, ang_z:%1.2f", (float)req.linear_x, (float)req.angular_z);

    // Create twist message and add linear and angular velocities
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = req.linear_x;
    twist_msg.angular.z = req.angular_z;

    // Publish the message with linear and angular velocities
    motor_command_publisher.publish(twist_msg);

    // Wait 3 seconds to give the robot time to react
    ros::Duration(3).sleep();

    // Publish a response message
    res.msg_feedback = "Robot wheel joint velocity set to linear x: " + std::to_string(req.linear_x)
        + " and angular z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}


int main(int argc, char** argv) {

    // Initialize this ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic
    // with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Ready to send robot wheel commands");

    // Handle ROS communication events
    ros::spin();

    return 0;
}