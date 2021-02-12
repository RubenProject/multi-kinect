#include "chessboard.h"


ChessCalib2d::ChessCalib2d(std::shared_ptr<Context> context)
    : mCalibrateReady(false), mContext(context)
{
    YAML::Node params = YAML::LoadFile("Res/camera_params.yaml");

    {
        double fx = params["KINECT_ID_0"]["color"]["fx"].as<double>();
        double fy = params["KINECT_ID_0"]["color"]["fy"].as<double>();
        double cx = params["KINECT_ID_0"]["color"]["cx"].as<double>();
        double cy = params["KINECT_ID_0"]["color"]["cy"].as<double>();

        double data[] = {fx,  0.0, cx, 0.0, fy,  cy, 0.0, 0.0, 1.0};
        K0 = cv::Mat(3, 3, CV_64F, data);
    }

    /*
    fx = params["KINECT_ID_0"]["depth"]["fx"];
    fy = params["KINECT_ID_0"]["depth"]["fy"];
    ix = params["KINECT_ID_0"]["depth"]["cx"];
    iy = params["KINECT_ID_0"]["depth"]["cy"];
    k1 = params["KINECT_ID_0"]["depth"]["k1"];
    k2 = params["KINECT_ID_0"]["depth"]["k2"];
    k3 = params["KINECT_ID_0"]["depth"]["k3"];
    p1 = params["KINECT_ID_0"]["depth"]["p1"];
    p2 = params["KINECT_ID_0"]["depth"]["p2"];
    */

    {
        double fx = params["KINECT_ID_1"]["color"]["fx"].as<double>();
        double fy = params["KINECT_ID_1"]["color"]["fy"].as<double>();
        double cx = params["KINECT_ID_1"]["color"]["cx"].as<double>();
        double cy = params["KINECT_ID_1"]["color"]["cy"].as<double>();

        double data[] = {fx,  0.0, cx, 0.0, fy,  cy, 0.0, 0.0, 1.0};
        K1 = cv::Mat(3, 3, CV_64F, data);
    }

    /*
    fx = params["KINECT_ID_1"]["depth"]["fx"];
    fy = params["KINECT_ID_1"]["depth"]["fy"];
    ix = params["KINECT_ID_1"]["depth"]["cx"];
    iy = params["KINECT_ID_1"]["depth"]["cy"];
    k1 = params["KINECT_ID_1"]["depth"]["k1"];
    k2 = params["KINECT_ID_1"]["depth"]["k2"];
    k3 = params["KINECT_ID_1"]["depth"]["k3"];
    p1 = params["KINECT_ID_1"]["depth"]["p1"];
    p2 = params["KINECT_ID_1"]["depth"]["p2"];
    */
}


ChessCalib2d::~ChessCalib2d()
{}


void ChessCalib2d::getHomography(glm::dmat3 &R, glm::dvec3 &t)
{
    R = this->R;
    t = this->t;
}


void ChessCalib2d::addSample(const cv::Mat &img0, const cv::Mat &img1)
{
    cv::Mat img_rgb0, img_rgb1;
    cv::cvtColor(img0, img_rgb0, cv::COLOR_BGR2RGB);
    cv::cvtColor(img1, img_rgb1, cv::COLOR_BGR2RGB);
    processSample(img_rgb0, img_rgb1);
}


void ChessCalib2d::loadSample(const std::string &name0, const std::string &name1)
{
    cv::Mat img0 = cv::imread(name0, cv::IMREAD_COLOR);
    cv::Mat img1 = cv::imread(name1, cv::IMREAD_COLOR);
    if (img0.empty() || img1.empty())
        mContext->log(libfreenect2::Logger::Error, "Failed to read image");
    processSample(img0, img1);
}


void ChessCalib2d::saveSample(const cv::Mat &img0, const cv::Mat &img1, const std::string &name0, const std::string &name1)
{
    cv::cvtColor(img0, img0, cv::COLOR_BGR2RGB);
    cv::cvtColor(img1, img1, cv::COLOR_BGR2RGB);
    cv::imwrite(name0, img0);
    cv::imwrite(name1, img1);
}


void ChessCalib2d::processSample(const cv::Mat &img0, const cv::Mat &img1)
{

    std::vector<cv::Point2f> corners0;
    std::vector<cv::Point2f> corners1;
    cv::Size patternSize = cv::Size(9, 6);
    bool found0 = cv::findChessboardCorners(img0, patternSize, corners0);
    bool found1 = cv::findChessboardCorners(img1, patternSize, corners1);

    std::cout << found0 << std::endl;
    std::cout << found1 << std::endl;

    if (found0 && found1){
        mCalibrateReady = true;
        mChessCorners0 = corners0;
        mChessCorners1 = corners1;
    }
}

bool ChessCalib2d::calibrateReady()
{
    return mCalibrateReady;
}

bool ChessCalib2d::calibrate()
{
    //TODO 
    //calculate essentialMat
    //Use essential mat in cv::recoverPose()
    //
    // get homography

    return true;
}
