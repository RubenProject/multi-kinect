#pragma once

#include "pch.h"
#include "body.h"


class Frame
{
public:
    Frame(openni::SensorType type, const int width, const int height, const cv::Scalar color, nite::UserTracker *tracker);
    Frame(openni::SensorType type, const openni::VideoFrameRef &frame, nite::UserTracker *tracker);
    Frame(openni::SensorType type, const cv::Mat &img);
    ~Frame();

    int getWidth() const;
    int getHeight() const;
    size_t getBytesPerPixel() const;
    unsigned char *getData() const;
    const cv::Mat &getMat();
    size_t getDataSize() const;
    cv::Point3_<float> getDepthPixel(int i, int j) const;
    cv::Vec3b getRGBPixel(int i, int j) const;
    cv::Point3_<float> getJointPixel(int i, int j) const;
    openni::SensorType getType();

    void setUserTracker(nite::UserTracker *userTracker);

    void show(const std::string &title) const;
    bool empty() const;
    void clear();

    void drawSkeleton(const Body &tBody);
    void drawChessboard(cv::Size boardSize, std::vector<cv::Point2f> &corners);
    void draw3DPoints(std::vector<cv::Point3f> points);
    void drawFrameAxes(cv::Mat cam, cv::Mat dist, cv::Mat rvec, cv::Mat tvec);
private:
    openni::SensorType mSensorType;
    cv::Mat mFrame;
    size_t mBytesPerPixel;
    nite::UserTracker *mUserTracker;
};
