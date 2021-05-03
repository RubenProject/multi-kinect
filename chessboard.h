#pragma once

#include "pch.h"
#include "frame.h"
#include "context.h"
#include "common.h"


class ChessCalib2d 
{
public:
    ChessCalib2d();
    ~ChessCalib2d();

    bool addSampleCamera(std::shared_ptr<Frame> frame, const int streamIdx, bool showResult = false);
    bool loadSampleCamera(const std::string &name, const int streamIdx);
    void saveSampleCamera(std::shared_ptr<Frame> frame,  const std::string &name, const int streamIdx);
    bool calibrateReadyCamera(const int streamIdx);
    bool calibrateCamera(const int streamIdx);

    bool readyCamera(const int streamIdx);
    void getCamera(glm::dmat3 &K, std::vector<double> &distcoeffs, const int streamIdx);
    bool loadCamera(const std::string &name, const int streamIdx);
    bool saveCamera(const std::string &name, const int streamIdx);

    bool addSampleHomography(std::shared_ptr<Frame> frame0, std::shared_ptr<Frame> &frame1, bool showResult=false);
    bool loadSampleHomography(const std::string &name0, const std::string &name1);
    void saveSampleHomography(std::shared_ptr<Frame> frame0, std::shared_ptr<Frame> frame1, 
            const std::string &name0, const std::string &name1);
    bool calibrateReadyHomography();
    bool calibrateHomography();

    void getHomography(glm::dmat3 &R, glm::dvec3 &t);
    bool loadHomography();
    bool saveHomography();

    void test(std::shared_ptr<Frame> rgb_frame, std::shared_ptr<Frame> depth_frame, const int streamIdx);

private:
    bool processSampleCamera(std::shared_ptr<Frame> frame, const int streamIdx, bool showResult=false);
    bool processSampleHomography(std::shared_ptr<Frame> frame0, std::shared_ptr<Frame> frame1, bool showResult=false);
    void createKnownBoardPositions(cv::Size size, float squareEdgeLength, std::vector<cv::Point3f> &corners);
    bool findPoseFromChessboard(std::shared_ptr<Frame> rgb_frame, const cv::Mat &cameraMat, const cv::Mat &dist, 
            cv::Mat &rvec, cv::Mat &tvec);

    cv::Mat mCameraMat0, mCameraMat1;
    cv::Mat mDistCoeffs0, mDistCoeffs1;

    std::vector<std::vector<cv::Point2f>> mChessCorners0, mChessCorners1;
    cv::Size imageSize;
    cv::Size boardSize;
    float squareEdgeLength;

    glm::dmat3 R;
    glm::dvec3 t;

    std::ofstream plotfile0, plotfile1;
};
