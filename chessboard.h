#pragma once

#include "pch.h"
#include "context.h"
#include "common.h"


//TODO Perhaps extend this to allow for iterative calibration using multiple images
class ChessCalib2d 
{
public:
    ChessCalib2d(std::shared_ptr<Context> context);
    ~ChessCalib2d();

    void addSample(const cv::Mat &img0, const cv::Mat &img1);
    void loadSample(const std::string &name0, const std::string &name1);
    void saveSample(const cv::Mat &img0, const cv::Mat &img1, const std::string &name0, const std::string &name1);
    void getHomography(glm::dmat3 &R, glm::dvec3 &t);

    bool calibrateReady();
    bool calibrate();
private:
    void processSample(const cv::Mat &img0, const cv::Mat &img1);

    std::vector<cv::Point2f> mChessCorners0;
    std::vector<cv::Point2f> mChessCorners1;
    cv::Mat K0, K1;
    bool mCalibrateReady;

    glm::dmat3 R;
    glm::dvec3 t;

    std::shared_ptr<Context> mContext;
};
