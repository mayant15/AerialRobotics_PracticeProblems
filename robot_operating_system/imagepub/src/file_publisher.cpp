#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

#define DEFAULT_FREQ 3

int main(int argc, char *argv[])
{
    
    if (!(argc == 3 || argc == 2)) {
        std::cout << "Invalid Arguments" << std::endl;
        return -1;
    } 

    ros::init(argc, argv, "file_publisher");
    ros::NodeHandle nh;

    // Create parameters
    std::string file_path = argv[1];
    nh.setParam("file", file_path);
    
    int freq;
    // Use default frequency if not provided
    if (argc == 2) {
        freq = DEFAULT_FREQ;
    } else {
        freq = std::atoi(argv[2]);
    }

    nh.setParam("frequency", freq);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("image", 1);

    cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv::waitKey(30);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    ros::Rate loop_rate(freq);

    while (nh.ok()) {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
