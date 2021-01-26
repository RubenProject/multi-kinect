#pragma once

#include "body.h"
#include "OpenNI.h"
#include "NiTE.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


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
    size_t getDataSize() const;
    void drawSkeleton(const nite::UserTracker &tUserTracker, const Body &tBody);
private:
    std::unique_ptr<cv::Mat> mFrame;
    size_t mBytesPerPixel;
};
