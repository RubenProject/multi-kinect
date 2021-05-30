#pragma once

#include "common.h"


class Camera
{
public:
    Camera();
    ~Camera();

    bool hasIntrinsics();
    bool hasExtrinsics();

    void setIntrinsics(const cv::Mat &K, const cv::Mat &dist);
    void setExtrinsics(const cv::Mat &rvec, const cv::Mat &tvec);
    void setImageSize(int w, int h);

    void getCameraMatrix(cv::Mat &K);
    void getDistortion(cv::Mat &dist);

    bool loadCamera(const std::string &name);
    bool saveCamera(const std::string &name);

    void fromCameraToWorldCoordinates(cv::Mat &p);
    void fromWorldToCameraCoordinates(cv::Mat &p);

    void fromCameraToWorldCoordinates(cv::Point3f &p);
    void fromWorldToCameraCoordinates(cv::Point3f &p);

private:
    //Intrinsic
    cv::Mat K;
    cv::Mat dist;

    //Extrinsic
    cv::Mat rmat;
    cv::Mat tvec;

    cv::Size imageSize;
    size_t state;
};
