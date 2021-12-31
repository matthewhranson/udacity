#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>


// Define a global client that can request services
ros::ServiceClient client;

// Define a global variable to keep track of whether the robot is moving
bool is_moving = false;


// This function calls the command_robot service to drive the robot in the specified direction
bool drive_robot(float lin_x, float ang_z) {

    // Log drive command
    ROS_INFO("drive_robot is calling command_robot service with lin_x:%1.2f, ang_z:%1.2f", lin_x, ang_z);

    // Create a service request with linear and angular velocity parameters
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the command_robot service
    bool succeeded = client.call(srv);
    if (!succeeded) {
        ROS_ERROR("Failed to call service command_robot");
        }
    return succeeded;
}


// This callback function continuously reads image data
void process_image_callback(const sensor_msgs::Image img) {

    // Define image width and target color
    float width = img.width;
    int white_pixel = 255;

    // Define wheel velocities.  The default is to stop when no ball is seen.
    float lin_x = 0;
    float ang_z = 0;

    // Log the image encoding.  Should be "rgb8".
    ROS_INFO("Image encoding is " + img.encoding);

    // Loop through each pixel in the image
    for (int i = 0; i < img.height * img.step; i++) {

        // If a pixel is white, figure out where it is located
        if (img.data[i] == white_pixel) {
            float ball_position = (float)i / width;

            // Ball on left
            if (ball_position < .2) {
                lin_x = 0;
                ang_z = -.1;

            // Ball straight ahead
            } else if (ball_position < .8) {
                lin_x = .1;
                ang_z = 0;

            // Ball on right
            } else {
                lin_x = 0;
                ang_z = .1;
            }

            // Once a white pixel is found, skip remaining pixels
            break;
        }
    }

    // Don't send a stop command if the robot is already stopped
    bool should_stop = (lin_x == 0) && (ang_z == 0);
    if ((!is_moving) && should_stop) {
        return;
        }

    // Drive the robot and update global movement status if service was successfully called
    if (drive_robot(lin_x, ang_z)) {
        is_moving = !should_stop;
    }
}


int main(int argc, char** argv) {

    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}