#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "structIO.hpp"
#include "dataStructures.h"

using namespace std;

void loadCalibrationData(cv::Mat &P_rect_00, cv::Mat &R_rect_00, cv::Mat &RT)
{
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;

    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;

    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;

}

void showLidarTopview(const std::vector<LidarPoint> &lidarPoints, cv::Size worldSize, cv::Size imageSize)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

    // plot Lidar points into image
    for (const auto& pt : lidarPoints)
    {
        float xw = pt.x; // world position in m with x facing forward from sensor
        float yw = pt.y; // world position in m with y facing left from sensor

        int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
        int x = (-yw * imageSize.height / worldSize.height) + imageSize.width / 2;

        float zw = pt.z; // world position in m with y facing left from sensor
        if(zw > -1.40){

            float val = pt.x;
            float maxVal = worldSize.height;
            int red = min(255, (int)(255 * abs((val - maxVal) / maxVal)));
            int green = min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
            cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, green, red), -1);
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

    cv::imwrite("topview.jpg", topviewImg);

    // display image
    string windowName = "Top-View Perspective of LiDAR data";
    cv::namedWindow(windowName, 2);
    cv::imshow(windowName, topviewImg);
    cv::resizeWindow(windowName, topviewImg.size[1] / 2, topviewImg.size[0] / 2);
    cv::waitKey(0); // wait for key to be pressed
}


void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints)
{
    // store calibration data in OpenCV matrices
    cv::Mat P_rect_xx(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_xx(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    loadCalibrationData(P_rect_xx, R_rect_xx, RT);

    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        // Amount by which to shrink a bounding box - helps with oversized boxes.
        const auto shrinkFactor = 0.10;

        // pointers to all bounding boxes which enclose the current Lidar point
        vector<BoundingBox*> enclosingBoxes;

        for (auto& box : boundingBoxes)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = box.roi.x + shrinkFactor * box.roi.width / 2.0;
            smallerBox.y = box.roi.y + shrinkFactor * box.roi.height / 2.0;
            smallerBox.width = box.roi.width * (1 - shrinkFactor);
            smallerBox.height = box.roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box;
            // if it is, register the box.
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(&box);
            }
        }

        // If only a single box was registered for this LiDAR point,
        // we associate it with it.
        if (enclosingBoxes.size() == 1)
        {
            auto& box = *enclosingBoxes[0];
            box.lidarPoints.push_back(*it1);
        }
    }
}

void show3DObjects(const std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for (const auto& box : boundingBoxes)
    {
        cv::RNG rng(box.boxID);
        const auto currColor = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150), rng.uniform(0, 150));

        int top=(int)1e8, left=(int)1e8, bottom=0, right=0;
        float xwmin=(int)1e8, ywmin=(int)1e8F, ywmax=-(int)1e8F;
        for (const auto& pt : box.lidarPoints)
        {
            // World coordinates.
            const auto xw = pt.x;
            const auto yw = pt.y;

            xwmin = xwmin < xw ? xwmin : xw;
            ywmin = ywmin < yw ? ywmin : yw;
            ywmax = ywmax > yw ? ywmax : yw;

            // top-view coordinates
            const int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            const int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top < y ? top : y;
            left = left < x ? left : x;
            bottom = bottom > y ? bottom : y;
            right = right > x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", box.boxID, (int)box.lidarPoints.size());
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);

        cv::putText(topviewImg, str1, cv::Point2f(left-50, top-50), cv::FONT_ITALIC, 1, currColor);
        cv::putText(topviewImg, str2, cv::Point2f(left, top-25), cv::FONT_ITALIC, 1, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0F;
    int nMarkers = std::floor(worldSize.height / lineSpacing);
    for (auto i = 0; i < nMarkers; ++i)
    {
        const int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    cv::imwrite("objects.jpg", topviewImg);

    std::string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if (bWait)
    {
        cv::waitKey(0);
        cv::destroyWindow(windowName);
    }
}

int main()
{
    const auto worldSize = cv::Size(10.0, 25.0);
    const auto imageSize = cv::Size(1000, 2000);

    std::vector<LidarPoint> lidarPoints;
    readLidarPts("../dat/C53A3_currLidarPts.dat", lidarPoints);

    std::vector<BoundingBox> boundingBoxes;
    readBoundingBoxes("../dat/C53A3_currBoundingBoxes.dat", boundingBoxes);

    clusterLidarWithROI(boundingBoxes, lidarPoints);

    show3DObjects(boundingBoxes, worldSize, imageSize, true);

    for (const auto& box : boundingBoxes)
    {
        if (!box.lidarPoints.empty())
        {
            showLidarTopview(box.lidarPoints, worldSize, imageSize);
        }
    }

    return 0;
}