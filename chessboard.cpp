#include "chessboard.h"

#include "logger.h"


ChessCalib2d::ChessCalib2d()
{
    if (!loadCamera("camera_params", KINECT_ID_0)) {
        mCameraMat0 = cv::Mat::eye(3, 3, CV_64F);
    }
    if (!loadCamera("camera_params", KINECT_ID_1)) {
        mCameraMat1 = cv::Mat::eye(3, 3, CV_64F);
    }

    imageSize = cv::Size(1920, 1080);
    boardSize = cv::Size(9, 6);
    squareEdgeLength = 0.025f; //length of chessboard square in meter

    plotfile0.open("Vis/plot0.txt", std::ofstream::out);
    plotfile1.open("Vis/plot1.txt", std::ofstream::out);
}


ChessCalib2d::~ChessCalib2d()
{
    plotfile0.close();
    plotfile1.close();
}


bool ChessCalib2d::addSampleCamera(std::shared_ptr<Frame> frame, const int streamIdx, bool showResult)
{
    return processSampleCamera(frame, streamIdx, showResult);
}


bool ChessCalib2d::loadSampleCamera(const std::string &name, const int streamIdx)
{
    std::shared_ptr<Frame> frame;
    cv::Mat img;

    if (streamIdx == KINECT_ID_0) {
        img = cv::imread("Samples/Cam0/" + name + ".png", cv::IMREAD_COLOR);
    } else if (streamIdx == KINECT_ID_1) {
        img = cv::imread("Samples/Cam1/" + name + ".png", cv::IMREAD_COLOR);
    } else {
        logger->log(libfreenect2::Logger::Error, "Stream Index out of range");
        exit(0x0);
    }

    if (img.empty()) {
        return false;
    }

    if (streamIdx == KINECT_ID_0) {
        frame = std::make_shared<Frame>("RGB0", img);
    } else { //KINECT_ID_1
        frame = std::make_shared<Frame>("RGB1", img);
    }

    return processSampleCamera(frame, streamIdx, false);
}


void ChessCalib2d::saveSampleCamera(std::shared_ptr<Frame> frame, const std::string &name, const int streamIdx)
{
    const cv::Mat &img = frame->getMat();
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

    if (streamIdx == KINECT_ID_0) {
        cv::imwrite("Samples/Cam0/" + name + ".png", img);
    } else if (streamIdx == KINECT_ID_1) {
        cv::imwrite("Samples/Cam1/" + name + ".png", img);
    } else {
        logger->log(libfreenect2::Logger::Error, "Stream Index out of range");
        exit(0x0);
    }
}


bool ChessCalib2d::calibrateReadyCamera(const int streamIdx)
{
    if (streamIdx == KINECT_ID_0) {
        return mChessCorners0.size() >= 50;
    } else if (streamIdx == KINECT_ID_1) {
        return mChessCorners1.size() >= 50;
    } else {
        logger->log(libfreenect2::Logger::Error, "Stream Index out of range");
        exit(0x0);
    }
    return false;
}


bool ChessCalib2d::calibrateCamera(const int streamIdx)
{
    std::vector<std::vector<cv::Point3f>> objpoints(1);
    std::vector<std::vector<cv::Point2f>> imgpoints;

    if (streamIdx == KINECT_ID_0) {
        imgpoints = mChessCorners0;
    } else { //KINECT_ID_1
        imgpoints = mChessCorners1;
    }

    createKnownBoardPositions(boardSize, squareEdgeLength, objpoints[0]);
    objpoints.resize(imgpoints.size(), objpoints[0]);

    cv::Mat cameraMat;
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rVecs, tVecs;

    cv::calibrateCamera(objpoints, imgpoints, imageSize, cameraMat, distCoeffs, rVecs, tVecs);
    // CALIB_FIX_ASPECT_RATIO
    // CALIB_USE_INTRINSIC_GUESS

    if (streamIdx == KINECT_ID_0) {
        mCameraMat0 = cameraMat;
        mDistCoeffs0 = distCoeffs;
        mChessCorners0.clear();
    } else { //KINECT_ID_1
        mCameraMat1 = cameraMat;
        mDistCoeffs1 = distCoeffs;
        mChessCorners1.clear();
    }
    return true;
}


bool ChessCalib2d::processSampleCamera(std::shared_ptr<Frame> frame, const int streamIdx, bool showResult)
{
    cv::Mat gray;
    const cv::Mat &img = frame->getMat();
    std::vector<cv::Point2f> corners;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    bool found = cv::findChessboardCorners(gray, boardSize, corners, 
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

        if (streamIdx == KINECT_ID_0) {
            mChessCorners0.push_back(corners);
        } else if (streamIdx == KINECT_ID_1) {
            mChessCorners1.push_back(corners);
        } else {
            return false;
        }
    }

    if (found && showResult) {
        frame->drawChessboard(boardSize, corners);
    }

    return found;
}


bool ChessCalib2d::readyCamera(const int streamIdx)
{
    if (streamIdx == KINECT_ID_0) {
        cv::Mat diff = mCameraMat0 != cv::Mat::eye(3, 3, CV_64F);
        return cv::countNonZero(diff) != 0;
    } else if (streamIdx == KINECT_ID_1) {
        cv::Mat diff = mCameraMat1 != cv::Mat::eye(3, 3, CV_64F);
        return cv::countNonZero(diff) != 0;
    } else {
        return false;
    }
}

void ChessCalib2d::getCamera(glm::dmat3 &K, std::vector<double> &distcoefs, const int streamIdx)
{
    //TODO
}


bool ChessCalib2d::loadCamera(const std::string &name, const int streamIdx)
{
    YAML::Node node;
    std::string fullname;
    if (streamIdx == KINECT_ID_0){
        fullname = "Res/cam0/" + name + ".yaml";
    } else if (streamIdx == KINECT_ID_1) {
        fullname = "Res/cam1/" + name + ".yaml";
    } else {
        logger->log(libfreenect2::Logger::Error, "Stream Index out of range");
        return false;
    }
    try {
        node = YAML::LoadFile(fullname);
    } catch (std::exception &e) {
        logger->log(libfreenect2::Logger::Info, "No camera matrix found.");
        return false;
    }
    std::vector<double> distcoeffs;
    double fx = node["fx"].as<double>();
    double fy = node["fy"].as<double>();
    double cx = node["cx"].as<double>();
    double cy = node["cy"].as<double>();
    distcoeffs = node["dist"].as<std::vector<double>>();


    double *data = new double[9]{fx,  0.0, cx, 0.0, fy,  cy, 0.0, 0.0, 1.0};
    if (streamIdx == KINECT_ID_0) {
        mCameraMat0 = cv::Mat(3, 3, CV_64F, data);
        mDistCoeffs0 = cv::Mat(distcoeffs);
    } else { //KINECT_ID_1
        mCameraMat1 = cv::Mat(3, 3, CV_64F, data);
        mDistCoeffs1 = cv::Mat(distcoeffs);
    } 
    return true;
}


bool ChessCalib2d::saveCamera(const std::string &name, const int streamIdx)
{
    YAML::Node node;
    std::string fullname;
    if (streamIdx == KINECT_ID_0) {
        fullname = "Res/cam0/" + name + ".yaml";
        node["fx"] = mCameraMat0.at<double>(0, 0);
        node["fy"] = mCameraMat0.at<double>(1, 1);
        node["cx"] = mCameraMat0.at<double>(0, 2);
        node["cy"] = mCameraMat0.at<double>(1, 2);
        for (int i = 0; i < mDistCoeffs0.rows; i++){
            node["dist"].push_back(mDistCoeffs0.at<double>(i, 0));
        }
    } else if (streamIdx == KINECT_ID_1) {
        fullname = "Res/cam1/" + name + ".yaml";
        node["fx"] = mCameraMat1.at<double>(0, 0);
        node["fy"] = mCameraMat1.at<double>(1, 1);
        node["cx"] = mCameraMat1.at<double>(0, 2);
        node["cy"] = mCameraMat1.at<double>(1, 2);
        for (int i = 0; i < mDistCoeffs1.rows; i++){
            node["dist"].push_back(mDistCoeffs1.at<double>(i, 0));
        }
    }

    std::ofstream fout(fullname.c_str());
    assert(fout.is_open());
    fout << node;
    fout.close();
    return true;
}


bool ChessCalib2d::addSampleHomography(std::shared_ptr<Frame> frame0, std::shared_ptr<Frame> &frame1, bool showResult)
{
    return processSampleHomography(frame0, frame1, showResult);
}


bool ChessCalib2d::loadSampleHomography(const std::string &name0, const std::string &name1)
{
    cv::Mat img0 = cv::imread("Samples/0_" + name0 + ".png", cv::IMREAD_COLOR);
    cv::Mat img1 = cv::imread("Samples/1_" + name1 + ".png", cv::IMREAD_COLOR);
    if (img0.empty() || img1.empty()) {
        return false;
    }
    std::shared_ptr<Frame> frame0 = std::make_shared<Frame>("RGB0", img0);
    std::shared_ptr<Frame> frame1 = std::make_shared<Frame>("RGB1", img1);
    return processSampleHomography(frame0, frame1);
}


void ChessCalib2d::saveSampleHomography(std::shared_ptr<Frame> frame0, std::shared_ptr<Frame> frame1, 
        const std::string &name0, const std::string &name1)
{
    cv::Mat img_rgb_0, img_rgb_1;
    const cv::Mat &img0 = frame0->getMat();
    const cv::Mat &img1 = frame1->getMat();
    cv::cvtColor(img0, img_rgb_0, cv::COLOR_BGR2RGB);
    cv::cvtColor(img1, img_rgb_1, cv::COLOR_BGR2RGB);
    cv::imwrite("Samples/0_" + name0 + ".png", img_rgb_0);
    cv::imwrite("Samples/1_" + name1 + ".png", img_rgb_1);
}


bool ChessCalib2d::processSampleHomography(std::shared_ptr<Frame> frame0, std::shared_ptr<Frame> frame1, bool showResult)
{
    /*cv::Mat rvec0, rvec1, tvec0, tvec1;
    const cv::Mat &img0 = frame0->getMat();
    const cv::Mat &img1 = frame1->getMat();
    bool found0=false, found1=false;

    found0 = findPoseFromChessboard(img0, mCameraMat0, mDistCoeffs0, rvec0, tvec0);
    //found1 = findPoseFromChessboard(img1, mCameraMat1, mDistCoeffs1, rvec1, tvec1);


    if (found0) {
        cv::Mat R;
        std::vector<cv::Point3f> objpoints;
        createKnownBoardPositions(boardSize, squareEdgeLength * 1000, objpoints);
        cv::Rodrigues(rvec0, R);
        R = R.t();
        tvec0 = -R * tvec0;

        for (auto &v : objpoints) {
            cv::Mat vMat = cv::Mat(v);
            vMat.convertTo(vMat, CV_64F);
            cv::Mat tMat = cv::Mat(tvec0);
            vMat = R * vMat + tMat;
            v = cv::Point3f(vMat.at<double>(0), vMat.at<double>(1), vMat.at<double>(2));
        }
        frame0->clear();
        frame0->draw3DPoints(objpoints);
    }

    if (found0 && showResult) {
        plotfile0 << tvec0.at<double>(0) << " " << tvec0.at<double>(2) << std::endl;
    }
    if (found1 && showResult) {
        plotfile1 << tvec1.at<double>(0) << " " << tvec1.at<double>(2) << std::endl;
    }

    if (found0 && found1) {
        std::cout << "---------------------------------" << std::endl;
        cv::Mat T0, T1, T0inv, T1inv, K0toK1, K1toK0;

        composeTransform(rvec0, tvec0, T0);
        composeTransform(rvec1, tvec1, T1);

        T0inv = T0.inv();
        T1inv = T1.inv();

        K0toK1 = T1inv * T0;
        K1toK0 = T0inv * T1;

        cv::Mat R0, R1, t0, t1;
        decomposeTransform(K0toK1, R0, t0);
        decomposeTransform(K1toK0, R1, t1);

        float dist0 = sqrt(t0.at<double>(0) * t0.at<double>(0) 
                         + t0.at<double>(1) * t0.at<double>(1) 
                         + t0.at<double>(2) * t0.at<double>(2));

        float dist1 = sqrt(t1.at<double>(0) * t1.at<double>(0) 
                         + t1.at<double>(1) * t1.at<double>(1) 
                         + t1.at<double>(2) * t1.at<double>(2));

        //std::cout << dist0 << " = " << dist1 << std::endl;

        //TODO normalize transform?
        // column wise or row wise
        // just based on 3, 3 coordinate
        
        //std::cout << rotationMatrixToEulerAngles(R0) * 180.0f / 3.141f << std::endl;
        //std::cout << rotationMatrixToEulerAngles(R1) * 180.0f / 3.141f << std::endl;
    }

    */
    return true;
}


bool ChessCalib2d::findPoseFromChessboard(std::shared_ptr<Frame> frame_rgb, const cv::Mat &cam, const cv::Mat &dist, 
        cv::Mat &rvec, cv::Mat &tvec)
{
    cv::Mat gray, R;
    std::vector<cv::Point3f> objpoints;
    std::vector<cv::Point2f> corners;
    cv::cvtColor(frame_rgb->getMat(), gray, cv::COLOR_BGR2GRAY);

    bool found = cv::findChessboardCorners(gray, boardSize, corners, 
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

    if (found) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

        createKnownBoardPositions(boardSize, squareEdgeLength, objpoints);
        cv::solvePnP(objpoints, corners, cam, dist, rvec, tvec);

        //frame_rgb->drawFrameAxes(cam, dist, rvec, tvec);

        //std::cout << "==========" << std::endl;
        //std::cout << tvec << std::endl;

        //objpoints.clear();

        //createKnownBoardPositions(boardSize, squareEdgeLength * 1000, objpoints);
        //cv::solvePnP(objpoints, corners, cam, dist, rvec, tvec);

        //std::cout << "++++++++++" << std::endl;
        //std::cout << tvec << std::endl;


        //cv::Mat flipMatrix = (cv::Mat_<double>(3,3) << 0,0,1,  0,1,0,  -1,0,0);

        //Invert the model coordinates to get the camera coordinates
        cv::Rodrigues(rvec, R);
        R = R.t();
        tvec = -R * tvec;
        cv::Rodrigues(R, rvec);
    }
    return found;
}


void ChessCalib2d::test(std::shared_ptr<Frame> frame_rgb, std::shared_ptr<Frame> frame_depth, const int streamIdx)
{
    cv::Mat rvec, rmat, tvec;

    cv::Mat cam, dist;
    if (streamIdx == 0) {
        cam = mCameraMat0;
        dist = mDistCoeffs0;
    } else {
        cam = mCameraMat1;
        dist = mDistCoeffs1;
    }

    bool found = findPoseFromChessboard(frame_rgb, cam, dist, rvec, tvec);


    if (found) {
        std::vector<cv::Point3f> objpoints;
        std::vector<cv::Point2f> corners;

        createKnownBoardPositions(boardSize, squareEdgeLength, objpoints);

        cv::Rodrigues(rvec, rmat);
        rmat = rmat.t();
        tvec = -rmat * tvec;
        cv::Rodrigues(rmat, rvec);

        //cv::Mat depthCam = (cv::Mat_<double>(3, 3) << 363.484,0,256, 0,363.484,212, 0,0,1);
        cv::Mat depthCam = (cv::Mat_<double>(3, 3) << 363.484,0,252.954, 0,363.484,209.023, 0,0,1);
        cv::Mat depthDist = (cv::Mat_<double>(5, 1, CV_64F) << 0.0908344, -0.271442, 0.0981953, 0, 0);
        //cv::Mat depthDist = (cv::Mat_<double>(5, 1, CV_64F) << 0, 0, 0, 0, 0);

        cv::projectPoints(objpoints, rvec, tvec, cam, dist, corners);

        frame_rgb->drawFrameAxes(cam, dist, rvec, tvec);
        frame_rgb->drawChessboard(boardSize, corners);

        objpoints.clear();
        corners.clear();

        createKnownBoardPositions(boardSize, squareEdgeLength, objpoints);

        cv::projectPoints(objpoints, rvec, tvec, depthCam, depthDist, corners);

        frame_depth->drawFrameAxes(depthCam, depthDist, rvec, tvec);
        frame_depth->drawChessboard(boardSize, corners);

        /*
        for (auto &v : objpoints) {
            cv::Mat vMat = cv::Mat(v);
            vMat.convertTo(vMat, CV_64F);
            cv::Mat tMat = cv::Mat(tvec);
            vMat = rmat * vMat + tMat;
            v = cv::Point3f(vMat.at<double>(0), vMat.at<double>(1), vMat.at<double>(2));
        }
        frame_depth->draw3DPoints(objpoints);
        */
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


bool ChessCalib2d::calibrateReadyHomography()
{
    assert(mChessCorners0.size() == mChessCorners1.size());
    return mChessCorners0.size() >= 10;
}


bool ChessCalib2d::calibrateHomography()
{
    std::vector<cv::Point2d> imageCoordinates0, imageCoordinates1;

    for (auto &v : mChessCorners0){
        for (auto &c : v) {
            imageCoordinates0.push_back(c);
        }
    }
    for (auto &v : mChessCorners1){
        for (auto &c : v) {
            imageCoordinates1.push_back(c);
        }
    }

    cv::undistortPoints(imageCoordinates0, imageCoordinates0, mCameraMat0, cv::noArray(), cv::noArray(), cv::noArray());
    cv::undistortPoints(imageCoordinates1, imageCoordinates1, mCameraMat1, cv::noArray(), cv::noArray(), cv::noArray());

    cv::Mat E, rVec, tVec, mask;
    E = cv::findEssentialMat(imageCoordinates0, imageCoordinates1, cv::Mat::eye(3, 3, CV_64F), cv::RANSAC, 0.99, 3.0, mask);
    cv::recoverPose(E, imageCoordinates0, imageCoordinates1, cv::Mat::eye(3, 3, CV_64F), rVec, tVec, mask);

    fromCV2GLM(rVec, &R);
    fromCV2GLM(tVec, &t);

    mChessCorners0.clear();
    mChessCorners1.clear();

    return true;
}


void ChessCalib2d::getHomography(glm::dmat3 &R, glm::dvec3 &t)
{
    R = this->R;
    t = this->t;
}


bool ChessCalib2d::saveHomography()
{
    YAML::Node node;

    node["Radial"] = R;
    node["Tangent"] = t;

    std::ofstream fout("Res/chess_homography.yaml");
    assert(fout.is_open());
    fout << node;
    fout.close();

    return true;
}


bool ChessCalib2d::loadHomography()
{
    YAML::Node node;

    try {
        node = YAML::LoadFile("Res/chess_homography.yaml");
    } catch (std::exception &e) {
        logger->log(libfreenect2::Logger::Error, "No Homography found.");
        return false;
    }

    R = node["Radial"].as<glm::dmat3>();
    t = node["Tangent"].as<glm::dvec3>();

    return true;
}


