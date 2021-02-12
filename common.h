#pragma once

#include "pch.h"


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


uint64_t getTimeNow();

bool isRotationMatrix(const cv::Mat &R);

cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat &R);

