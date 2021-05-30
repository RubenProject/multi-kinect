#include "chessboard.h"

#include "logger.h"


ChessCalib2d::ChessCalib2d()
{
    mCameraName[KINECT_COLOR_0] = "Res/cam/camera_RGB0_params.yaml";
    mCameraName[KINECT_COLOR_1] = "Res/cam/camera_RGB1_params.yaml";
    mCameraName[KINECT_DEPTH_0] = "Res/cam/camera_DEPTH0_params.yaml";
    mCameraName[KINECT_DEPTH_1] = "Res/cam/camera_DEPTH1_params.yaml";

    mCamera[KINECT_COLOR_0].loadCamera(mCameraName[KINECT_COLOR_0]);
    mCamera[KINECT_COLOR_1].loadCamera(mCameraName[KINECT_COLOR_1]);
    mCamera[KINECT_DEPTH_0].loadCamera(mCameraName[KINECT_DEPTH_0]);
    mCamera[KINECT_DEPTH_1].loadCamera(mCameraName[KINECT_DEPTH_1]);

    imageSizeRGB = cv::Size(1920, 1080);
    imageSizeDEPTH = cv::Size(512, 424);

    boardSize = cv::Size(9, 6);
    squareEdgeLength = 0.025f; //length of chessboard square in meter
}


ChessCalib2d::~ChessCalib2d()
{
}


bool ChessCalib2d::addSample_Calibration(std::shared_ptr<Frame> frame, const int streamIdx, bool showResult)
{
    return processSample_Calibration(frame, streamIdx, showResult);
}


bool ChessCalib2d::loadSample_Calibration(const std::string &name, const int streamIdx)
{
    std::shared_ptr<Frame> frame;
    cv::Mat img;

    switch (streamIdx) {
        case KINECT_COLOR_0:
            img = cv::imread("Samples/Cam0/" + name + ".png", cv::IMREAD_COLOR);
            break;
        case KINECT_COLOR_1:
            img = cv::imread("Samples/Cam1/" + name + ".png", cv::IMREAD_COLOR);
            break;
        case KINECT_DEPTH_0:
            img = cv::imread("Samples/Cam2/" + name + ".png", cv::IMREAD_COLOR);
            break;
        case KINECT_DEPTH_1:
            img = cv::imread("Samples/Cam3/" + name + ".png", cv::IMREAD_COLOR);
            break;
        default:
            logger->log(libfreenect2::Logger::Error, "Stream Index out of range");
            exit(0x0);
    }

    if (img.empty()) {
        return false;
    }

    frame = std::make_shared<Frame>(openni::SensorType::SENSOR_COLOR, img);

    return processSample_Calibration(frame, streamIdx, false);
}


void ChessCalib2d::saveSample_Calibration(std::shared_ptr<Frame> frame, const std::string &name, const int streamIdx)
{
    const cv::Mat &img = frame->getMat();
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

    switch (streamIdx) {
        case KINECT_COLOR_0:
            cv::imwrite("Samples/Cam0/" + name + ".png", img);
            break;
        case KINECT_COLOR_1:
            cv::imwrite("Samples/Cam1/" + name + ".png", img);
            break;
        case KINECT_DEPTH_0:
            cv::imwrite("Samples/Cam2/" + name + ".png", img);
            break;
        case KINECT_DEPTH_1:
            cv::imwrite("Samples/Cam3/" + name + ".png", img);
            break;
        default:
            logger->log(libfreenect2::Logger::Error, "Stream Index out of range");
            exit(0x0);
    }
}


bool ChessCalib2d::ready_Calibration(const int streamIdx)
{
    return mChessCorners[streamIdx].size() >= 50;
}


bool ChessCalib2d::do_Calibration(const int streamIdx)
{
    std::vector<std::vector<cv::Point3f>> objpoints(1);
    std::vector<std::vector<cv::Point2f>> imgpoints;
    cv::Size imageSize;

    if (streamIdx == KINECT_COLOR_0 || streamIdx == KINECT_COLOR_1) {
        imageSize = imageSizeRGB;
    } else if (streamIdx == KINECT_DEPTH_0 || streamIdx == KINECT_DEPTH_1) {
        imageSize = imageSizeDEPTH;
    } else {
        logger->log(libfreenect2::Logger::Error, "Stream Index out of range");
        exit(0x0);
    }
    imgpoints = mChessCorners[streamIdx];

    createKnownBoardPositions(boardSize, squareEdgeLength, objpoints[0]);
    objpoints.resize(imgpoints.size(), objpoints[0]);

    cv::Mat cameraMat;
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rVecs, tVecs;

    cv::calibrateCamera(objpoints, imgpoints, imageSize, cameraMat, distCoeffs, rVecs, tVecs);
    // CALIB_FIX_ASPECT_RATIO
    // CALIB_USE_INTRINSIC_GUESS

    mCamera[streamIdx].setIntrinsics(cameraMat, distCoeffs);
    mCamera[streamIdx].saveCamera(mCameraName[streamIdx]);

    return true;
}


bool ChessCalib2d::processSample_Calibration(std::shared_ptr<Frame> frame, const int streamIdx, bool showResult)
{
    cv::Mat gray;
    const cv::Mat &img = frame->getMat();
    std::vector<cv::Point2f> corners;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    bool found = cv::findChessboardCorners(gray, boardSize, corners, 
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found) {
        logger->log(libfreenect2::Logger::Info, "Found Sample");
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

        mChessCorners[streamIdx].push_back(corners);
    }

    if (found && showResult) {
        frame->drawChessboard(boardSize, corners);
    }

    return found;
}


bool ChessCalib2d::done_Calibration(const int streamIdx)
{
    return mCamera[streamIdx].hasIntrinsics();
}



bool ChessCalib2d::findPoseFromChessboard(std::shared_ptr<Frame> frame, const cv::Mat &cam, const cv::Mat &dist, 
        cv::Mat &rvec, cv::Mat &tvec, bool draw)
{
    cv::Mat gray, R;
    std::vector<cv::Point3f> objpoints;
    std::vector<cv::Point2f> corners;

    cv::cvtColor(frame->getMat(), gray, cv::COLOR_BGR2GRAY);

    bool found = cv::findChessboardCorners(gray, boardSize, corners, 
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

    if (found) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

        createKnownBoardPositions(boardSize, squareEdgeLength, objpoints);
        cv::solvePnP(objpoints, corners, cam, dist, rvec, tvec);

        if (draw) {
            frame->drawFrameAxes(cam, dist, rvec, tvec);
            //frame->drawChessboard(boardSize, corners);
        }

        //Invert the model coordinates to get the camera coordinates
        invertPose(rvec, tvec);
    }
    return found;
}


bool ChessCalib2d::findExtrinsicsFromChessboard(std::shared_ptr<Frame> frame0, const int streamIdx0,
                                                std::shared_ptr<Frame> frame1, const int streamIdx1)
{
    bool found0, found1;
    cv::Mat cam0, dist0, rvec0, tvec0;
    cv::Mat cam1, dist1, rvec1, tvec1;

    mCamera[streamIdx0].getCameraMatrix(cam0);
    mCamera[streamIdx0].getDistortion(dist0);
    found0 = findPoseFromChessboard(frame0, cam0, dist0, rvec0, tvec0, true);

    mCamera[streamIdx1].getCameraMatrix(cam1);
    mCamera[streamIdx1].getDistortion(dist1);
    found1 = findPoseFromChessboard(frame1, cam1, dist1, rvec1, tvec1, true);

    if (found0 && found1) {
        mCamera[streamIdx0].setExtrinsics(rvec0, tvec0);
        mCamera[streamIdx1].setExtrinsics(rvec1, tvec1);
        //std::cout << "===========" << std::endl;
        //std::cout << tvec0 << std::endl;
        //std::cout << tvec1 << std::endl;
        return true;
    }
    return false;
}


void ChessCalib2d::test(std::shared_ptr<Frame> frame0, const int streamIdx0,
                        std::shared_ptr<Frame> frame1, const int streamIdx1)
{
    bool found;
    cv::Mat cam0, dist0, rvec0, tvec0;
    cv::Mat cam1, dist1;

    mCamera[streamIdx0].getCameraMatrix(cam0);
    mCamera[streamIdx0].getDistortion(dist0);
    mCamera[streamIdx1].getCameraMatrix(cam1);
    mCamera[streamIdx1].getDistortion(dist1);
    
    found = findPoseFromChessboard(frame0, cam0, dist0, rvec0, tvec0);

    if (found) {
        //get 3d points of chessboard in frame 0
        std::vector<cv::Point3f> objpoints;
        invertPose(rvec0, tvec0);
        createKnownBoardPositions(boardSize, squareEdgeLength, objpoints);
        transformPoints(objpoints, rvec0, tvec0);

        //transform to 3d points in frame1
        cv::Mat mat_point;
        for (cv::Point3f &point : objpoints) {
            fromPointf2Mat(point, mat_point);
            mCamera[KINECT_COLOR_0].fromCameraToWorldCoordinates(mat_point);
            mCamera[KINECT_DEPTH_0].fromWorldToCameraCoordinates(mat_point);
            fromMat2Pointf(mat_point, point);
        }

        //project to 2d
        std::vector<cv::Point2f> corners;
        cv::projectPoints(objpoints, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), cam1, dist1, corners);
            
        frame1->drawChessboard(boardSize, corners);
    }
}


void ChessCalib2d::createKnownBoardPositions(cv::Size boardSize, float squareEdgeLength, std::vector<cv::Point3f> &corners)
{
    for (int i = 0; i < boardSize.height; i++){
        for (int j = 0; j < boardSize.width; j++){
            corners.push_back(cv::Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
        }
    }
}



void ChessCalib2d::getHomography()
{
}


bool ChessCalib2d::saveHomography()
{
}


bool ChessCalib2d::loadHomography()
{
}


