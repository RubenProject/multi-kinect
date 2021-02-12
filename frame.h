#pragma once

#include "pch.h"
#include "body.h"


class Frame
{
public:
    Frame(const int width, const int height, const cv::Scalar color);
    Frame(const openni::VideoFrameRef &frame, const std::string &mode);
    ~Frame();
    int getWidth() const;
    int getHeight() const;
    size_t getBytesPerPixel() const;
    unsigned char *getData() const;
    cv::Mat getMat();
    size_t getDataSize() const;
    void drawSkeleton(const nite::UserTracker &tUserTracker, const Body &tBody);
private:
    cv::Mat mFrame;
    size_t mBytesPerPixel;
};
