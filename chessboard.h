#pragma once

#include "pch.h"
#include "frame.h"
#include "context.h"
#include "common.h"
#include "camera.h"


class ChessCalib2d 
{
public:
    ChessCalib2d();
    ~ChessCalib2d();

    bool addSample_Calibration(std::shared_ptr<Frame> frame, const int streamIdx, bool showResult = false);
    bool loadSample_Calibration(const std::string &name, const int streamIdx);
    void saveSample_Calibration(std::shared_ptr<Frame> frame,  const std::string &name, const int streamIdx);

    bool ready_Calibration(const int streamIdx);
    bool do_Calibration(const int streamIdx);

    bool done_Calibration(const int streamIdx);

    void getHomography();
    bool loadHomography();
    bool saveHomography();

    bool findExtrinsicsFromChessboard(std::shared_ptr<Frame> frame0, const int streamIdx0, 
                                      std::shared_ptr<Frame> frame1, const int streamIdx1);
    void test(std::shared_ptr<Frame> frame0, const int streamIdx0, std::shared_ptr<Frame> frame1, const int streamIdx1);

private:
    bool processSample_Calibration(std::shared_ptr<Frame> frame, const int streamIdx, bool showResult=false);

    void createKnownBoardPositions(cv::Size size, float squareEdgeLength, std::vector<cv::Point3f> &objpoints);
    bool findPoseFromChessboard(std::shared_ptr<Frame> frame, const cv::Mat &cameraMat, const cv::Mat &dist, 
            cv::Mat &rvec, cv::Mat &tvec, bool draw=false);

    //Cameras
    std::array<std::string, KINECT_STREAM_COUNT> mCameraName;
    std::array<Camera, KINECT_STREAM_COUNT> mCamera;

    std::array<std::vector<std::vector<cv::Point2f>>, KINECT_STREAM_COUNT> mChessCorners;
    cv::Size imageSizeRGB, imageSizeDEPTH;
    
    //chessboard dimensions
    cv::Size boardSize;
    float squareEdgeLength;
};
