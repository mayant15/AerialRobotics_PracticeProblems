#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"

#include <iostream>

void adjustImage(sensor_msgs::Image::Ptr img)
{
    std::cout << "Received the image. adjustImage callback called" << std::endl;

    int b;
    float c;
    
    ros::param::get("brightness", b);
    ros::param::get("contrast", c);

    if (b < 0) {
        std::cout << "Invalid parameter: brightness" << std::endl;
        return;
    }

    if (c <= 0) {
        std::cout << "Invalid parameter: contrast" << std::endl;
        return;
    }

    for (uint8_t i = 0; i < img->step * img->height; ++i) {
        img->data[i] = c * img->data[i] + b;
    }

    std::cout << "Adjusted the image" << std::endl;
}

int main(int argc, char *argv[])
{
    // Initialise ROS
    ros::init(argc, argv, "image_changer");
    
    // Create a NodeHandle
    ros::NodeHandle nh;

    nh.setParam("brightness", 1);
    nh.setParam("contrast", 0.5);

    // Subscribe to the image topic
    ros::Subscriber sub = nh.subscribe("image", 100, adjustImage);

    // spin
    ros::spin();

    return 0;
}
