#pragma once

#include <OpenNI.h>
#include <NiTE.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <array>
#include <sstream>
#include <fstream>
#include <cmath>
#include <iostream>


#include "context.h"


enum KINECT_DEVICES : int
{
    KINECT_ID_0 = 0,
    KINECT_ID_1 = 1,
    KINECT_COUNT = 2
};


enum KINECT_STREAM : int
{
    KINECT_COLOR_0 = 0,
    KINECT_COLOR_1 = 1,
    KINECT_DEPTH_0 = 2,
    KINECT_DEPTH_1 = 3,
    KINECT_STREAM_COUNT = 4
};


inline uint64_t getTimeNow()
{
    using namespace std::chrono;
    milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    return ms.count();
}


inline bool isRotationMatrix(const cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

    return cv::norm(I, shouldBeIdentity) < 1e-6;
}


inline cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat &R)
{
    assert(isRotationMatrix(R));

    float sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + 
                         R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6;

    float x, y, z;
    if (!singular) {
        x = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        x = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y , z);
}

