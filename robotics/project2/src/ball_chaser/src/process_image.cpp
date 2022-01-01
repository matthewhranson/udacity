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


// This callback function looks for a white ball in image data and send wheel commands to follow it if present
void process_image_callback(const sensor_msgs::Image img) {

    // Create list to hold the locations of white pixels
    std::list<int> white_pixels;

    // Loop through each pixel in the image.  This works because the image encoding ('rgb8') has one byte per column.
    for (int i = 0; i < img.height; i++) {
        for (int j = 0; j < img.step; j++) {
            if (img.data[i*img.step + j] == 255) {
                white_pixels.push_back(j);
            }
        }
    }

    // If white pixels were detected, then calculate wheel velocities
    float lin_x;
    float ang_z;
    if (white_pixels.size() > 0) {

        // Calculate ball position
        float avg_pixel = (float)std::accumulate(white_pixels.begin(), white_pixels.end(), 0.0) / (float)white_pixels.size();
        float ball_position = avg_pixel / (float)img.step;

        // Generate wheel velocities
        if (ball_position < .2) {  // Ball on left
            lin_x = 0;
            ang_z = .1;
        } else if (ball_position < .8) {  // Ball straight ahead
            lin_x = .1;
            ang_z = 0;
        } else {  // Ball on right
            lin_x = 0;
            ang_z = -.1;
        }

    } else {  // If no white pixels were detected, then stop
        lin_x = 0;
        ang_z = 0;
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
    ROS_INFO("Ready to process images");
    ros::spin();

    return 0;
}