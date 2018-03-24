#include "ros/ros.h"
#include "image_transport/image_transport.h"

#define DEFAULT_HEIGHT 512
#define DEFAULT_WIDTH 512
#define DEFAULT_FREQ 5
#define DEFAULT_SQAURE_SIZE 64

class Point
{
public:

    //Constructor
    Point(int idx_x, int idx_y){
        x = idx_x;
        y = idx_y;
    }

    //Members
    int x, y;
};

void rectangle(sensor_msgs::Image& img_msg, Point top_left, Point bottom_right, int color);

int main(int argc, char *argv[])
{
    // Validate input
    if (!(argc == 1 || argc == 5)) {
        std::cout << "Invalid Arguments" << std::endl;
        std::cout << "Usage: cb_publisher <height> <width> <square_size> <frequency>" << std::endl;
        return -1;
    }

    ros::init(argc, argv, "cb_publisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("cb_image", 1);

    int height, width, square_size, freq;

    if (argc == 1) {
        height = DEFAULT_HEIGHT;
        width = DEFAULT_WIDTH;
        square_size = DEFAULT_SQAURE_SIZE;
        freq = DEFAULT_FREQ;
    } else {
        height = std::atoi(argv[1]);
        width = std::atoi(argv[2]);
        square_size = std::atoi(argv[3]);
        freq = std::atoi(argv[4]);
    }

    nh.setParam("height", height);
    nh.setParam("width", width);
    nh.setParam("square_size", square_size);
    nh.setParam("frequency", freq);

    // Create the image message
    sensor_msgs::Image msg;
    msg.encoding = "mono8";
    msg.header.stamp = ros::Time::now();
    msg.height = height;
    msg.is_bigendian = false;
    msg.step =  width;
    msg.width = width;

    msg.data.resize(msg.step * msg.height, 0);

    for (int i = 0; i < height; i += square_size) {
        int j;

        if ((i / square_size) % 2 == 0) {
            j = square_size;
        } else {
            j = 0;
        }

        for (; j < width; j += 2 * square_size) {
            Point top_left(j, i);
            Point bottom_right(j + square_size, i + square_size);
            rectangle(msg, top_left, bottom_right, 255);
        }
    }

    ros::Rate loop_rate(freq);

    while (nh.ok()) {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void rectangle(sensor_msgs::Image& img_msg, Point top_left, Point bottom_right, int color)
{
    int sq_size = bottom_right.x - top_left.x;
    int start_idx = top_left.x + top_left.y * img_msg.width;

    for (int i = 0; i < sq_size; ++i) {
        for (int j = 0; j < sq_size; j++) {
            int idx = start_idx + j + i * (int) img_msg.width;
            img_msg.data[idx] = color;
        }
    }
}
