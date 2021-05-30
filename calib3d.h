#pragma once

#include "pch.h"
#include "frame.h"
#include "camera.h"


class Calib3d
{
public:
    Calib3d();
    ~Calib3d();

    void getCamera(Camera &camera, const int streamIdx);
    void setCamera(Camera &camera, const int streamIdx);

    bool findExtrinsicRelation(std::shared_ptr<Frame> frame0, const int streamIdx0,
                               std::shared_ptr<Frame> frame1, const int streamIdx1);
    void testExtrinsicRelation(std::shared_ptr<Frame> frame0, const int streamIdx0,
                               std::shared_ptr<Frame> frame1, const int streamIdx1);
private:
    void createKnownBoardPositions(cv::Size size, float squareEdgeLength, std::vector<cv::Point3f> &objpoints);
    bool findPoseFromChessboard(std::shared_ptr<Frame> frame, const cv::Mat &cam, const cv::Mat &dist, 
                                cv::Mat &rvec, cv::Mat &tvec, bool showResults);

    std::array<Camera, KINECT_STREAM_COUNT> mCamera;
    cv::Size boardSize;
    float squareEdgeLength;
};
