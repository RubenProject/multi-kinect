#include "calib3d.h"



Calib3d::Calib3d()
{
    boardSize = cv::Size(9, 6);
    squareEdgeLength = 0.025f;
}


Calib3d::~Calib3d()
{
}


void Calib3d::getCamera(Camera &camera, const int streamIdx)
{
    camera = mCamera[streamIdx];
}


void Calib3d::setCamera(Camera &camera, const int streamIdx)
{
    mCamera[streamIdx] = camera;
}


bool Calib3d::findExtrinsicRelation(std::shared_ptr<Frame> frame0, const int streamIdx0,
                                    std::shared_ptr<Frame> frame1, const int streamIdx1)
{
    bool found0, found1;
    cv::Mat cam0, dist0, rvec0, tvec0;
    cv::Mat cam1, dist1, rvec1, tvec1;

    assert(mCamera[streamIdx0].hasIntrinsics());
    assert(mCamera[streamIdx1].hasIntrinsics());

    mCamera[streamIdx0].getCameraMatrix(cam0);
    mCamera[streamIdx0].getDistortion(dist0);
    found0 = findPoseFromChessboard(frame0, cam0, dist0, rvec0, tvec0, true);

    mCamera[streamIdx1].getCameraMatrix(cam1);
    mCamera[streamIdx1].getDistortion(dist1);
    found1 = findPoseFromChessboard(frame1, cam1, dist1, rvec1, tvec1, true);

    if (found0 && found1) {
        mCamera[streamIdx0].setExtrinsics(rvec0, tvec0);
        mCamera[streamIdx1].setExtrinsics(rvec1, tvec1);
        invertPose(rvec0, tvec0);
        invertPose(rvec1, tvec1);
        std::cout << "===========" << std::endl;
        std::cout << tvec0 << std::endl;
        std::cout << tvec1 << std::endl;
        mCamera[streamIdx0].fromCameraToWorldCoordinates(tvec0);
        mCamera[streamIdx1].fromWorldToCameraCoordinates(tvec0);
        std::cout << tvec0 << std::endl;
        return true;
    }
    return false;
}


bool Calib3d::findPoseFromChessboard(std::shared_ptr<Frame> frame, const cv::Mat &cam, const cv::Mat &dist,
                            cv::Mat &rvec, cv::Mat &tvec, bool showResults)
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
 
        if (showResults) {
            frame->drawFrameAxes(cam, dist, rvec, tvec);
            //frame->drawChessboard(boardSize, corners);
        }
 
        //Invert the model coordinates to get the camera coordinates
        invertPose(rvec, tvec);
    }
    return found;
}


void Calib3d::createKnownBoardPositions(cv::Size size, float squareEdgeLength, std::vector<cv::Point3f> &objpoints)
{
    for (int i = 0; i < size.height; i++){
        for (int j = 0; j < size.width; j++){
            objpoints.push_back(cv::Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
        }
    }
}


void Calib3d::testExtrinsicRelation(std::shared_ptr<Frame> frame0, const int streamIdx0,
                                    std::shared_ptr<Frame> frame1, const int streamIdx1)
{
    bool found;
    cv::Mat cam0, dist0, rvec0, tvec0;
    cv::Mat cam1, dist1;

    mCamera[streamIdx0].getCameraMatrix(cam0);
    mCamera[streamIdx0].getDistortion(dist0);
    mCamera[streamIdx1].getCameraMatrix(cam1);
    mCamera[streamIdx1].getDistortion(dist1);
    
    found = findPoseFromChessboard(frame0, cam0, dist0, rvec0, tvec0, true);

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

