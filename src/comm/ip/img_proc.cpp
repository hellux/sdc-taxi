#include <iostream>

#include <opencv2/opencv.hpp>

using namespace cv;

extern "C" void ip_process(void);

void ip_process(void) {
    std::cout << "hej från c++" << std::endl;

    Mat image;
    image = imread("img.png");

    if (image.data) {
        namedWindow("Display Image", WINDOW_AUTOSIZE );
        imshow("Display Image", image);
        waitKey(0);
        cvShowImage( "result", newImg);
        cv::waitKey(100);
    } else {
        printf("No image data \n");
    }
}

