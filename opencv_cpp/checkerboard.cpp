#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

int main(int argc, char *argv[])
{
    // check if proper arguments are given
    if(argc != 4){
        std::cout << "Usage: checkerboard <width> <height> <side_length_of_small_block>" << std::endl;
        return -1;
    }

    int img_width = std::atoi(argv[1]);
    int img_height = std::atoi(argv[2]);
    int block_size = std::atoi(argv[3]);
    cv::Mat base_img(img_height, img_width, CV_8UC4);
    cv::Scalar white(255, 255, 255, 0.9);

    // loop over the mat
    for (int i = 0; i < img_height; i += block_size) {
        int j;

        if((i / block_size) % 2 == 0){
            j = block_size;
        } else {
            j = 0;
        }

        for (; j < img_width; j += 2 * block_size) {
            cv::Point top_left(j, i);
            cv::Point bottom_right(j + block_size, i + block_size);
            rectangle(base_img, top_left, bottom_right, white, CV_FILLED, 8, 0);
        }
    }
    
    cv::imwrite("board.jpg", base_img);

    return 0;
}
