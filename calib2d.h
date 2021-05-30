#pragma once

#include "pch.h"
#include "frame.h"
#include "camera.h"


class Calib2d
{
public:
    Calib2d();
    ~Calib2d();

    bool addSample(std::shared_ptr<Frame> frame, const int streamIdx, bool showResult);
    bool loadSample(const std::string &name, const int streamIdx);
    void saveSample(std::shared_ptr<Frame> frame, const std::string &name, const int streamIdx);

    bool ready(const int streamIdx);
    bool calibrate(const int streamIdx);
    bool calibrated(const int streamIdx);

    bool getCamera(Camera &cam, const int streamIdx);

private:
    bool processSample(std::shared_ptr<Frame> frame, const int streamIdx, bool showResult);
    void createKnownBoardPositions(cv::Size size, float squareEdgeLength, std::vector<cv::Point3f> &objpoints);

    std::array<std::vector<std::vector<cv::Point2f>>, KINECT_STREAM_COUNT> mChessCorners;
    std::array<Camera, KINECT_STREAM_COUNT> mCamera;
    std::array<std::string, KINECT_STREAM_COUNT> mCameraName;

    cv::Size boardSize, imageSizeRGB, imageSizeDEPTH;
    float squareEdgeLength;
};
