#include "calib2d.h"

#include "logger.h"

#define MIN_CALIBRATION_SAMPLES 50;


Calib2d::Calib2d()
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
    squareEdgeLength = 0.025f;
}


Calib2d::~Calib2d()
{}


bool Calib2d::addSample(std::shared_ptr<Frame> frame, const int streamIdx, bool showResult)
{
    return processSample(frame, streamIdx, showResult);
}


bool Calib2d::loadSample(const std::string &name, const int streamIdx)
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

    return processSample(frame, streamIdx, false);
}


void Calib2d::saveSample(std::shared_ptr<Frame> frame, const std::string &name, const int streamIdx)
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


bool Calib2d::processSample(std::shared_ptr<Frame> frame, const int streamIdx, bool showResult)
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


bool Calib2d::ready(const int streamIdx)
{
    return mChessCorners[streamIdx].size() >= MIN_CALIBRATION_SAMPLES;
}


bool Calib2d::calibrate(const int streamIdx)
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


bool Calib2d::calibrated(const int streamIdx)
{
    return mCamera[streamIdx].hasIntrinsics();
}


void Calib2d::createKnownBoardPositions(cv::Size boardSize, float squareEdgeLength, std::vector<cv::Point3f> &objpoints)
{
    for (int i = 0; i < boardSize.height; i++){
        for (int j = 0; j < boardSize.width; j++){
            objpoints.push_back(cv::Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
        }
    }
}

bool Calib2d::getCamera(Camera &cam, const int streamIdx)
{
    cam = mCamera[streamIdx];
    if (!cam.hasIntrinsics())
        return false;
    return true;
}
