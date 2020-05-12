#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "structIO.hpp"

using namespace std;

void showLidarTopview()
{
    std::vector<LidarPoint> lidarPoints;
    readLidarPts("../dat/C51_LidarPts_0000.dat", lidarPoints);

    cv::Size worldSize(10.0, 20.0); // width and height of sensor field in m
    cv::Size imageSize(1000, 2000); // corresponding top view image in pixel

    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

    // plot Lidar points into image
    for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
    {
        float xw = (*it).x; // world position in m with x facing forward from sensor
        float yw = (*it).y; // world position in m with y facing left from sensor

        int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
        int x = (-yw * imageSize.height / worldSize.height) + imageSize.width / 2;

        //cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);

        // STUDENT EXERCISE 2
        float zw = (*it).z; // world position in m with y facing left from sensor
        if(zw > -1.40){
        // EOF STUDENT EXERCISE 2

            // STUDENT EXERCISE 1
            float val = it->x;
            float maxVal = worldSize.height;
            int red = min(255, (int)(255 * abs((val - maxVal) / maxVal)));
            int green = min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
            cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, green, red), -1);
            // EOF STUDENT EXERCISE 1
        }
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // rotate image for visualization
    cv::rotate(topviewImg, topviewImg, cv::ROTATE_90_CLOCKWISE);

    // display image
    string windowName = "Top-View Perspective of LiDAR data";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, topviewImg);
    cv::resizeWindow(windowName, 1000, 500);
    cv::waitKey(0); // wait for key to be pressed

    cv::imwrite("topView.jpg", topviewImg);
}

int main()
{
    showLidarTopview();
}