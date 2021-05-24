#include "frame.h"

#include "common.h"
#include "logger.h"

#include <iostream>


Frame::Frame(openni::SensorType type, const int width, const int height, const cv::Scalar color, nite::UserTracker *tracker)
{
    mSensorType = type;
    mUserTracker = tracker;
    mFrame = cv::Mat(height, width, CV_8UC3, color);
    mBytesPerPixel = 3;
}


Frame::Frame(openni::SensorType type, const openni::VideoFrameRef &frame, nite::UserTracker *tracker)
{
    mSensorType = type;
    mUserTracker = tracker;
    const int width = frame.getWidth();
    const int height = frame.getHeight();
    if (type == openni::SensorType::SENSOR_IR) {
        uint16_t *tFrameData = (uint16_t*)frame.getData();
        //XXX for some reason the videostream incorrectly reports the dimensions of the frame...
        mFrame = cv::Mat(424, 512, CV_8UC3);
        mFrame.forEach<cv::Vec3b>([&](cv::Vec3b &pixel, const int *position) -> void {
               int tIdx = position[0] * width + position[1];
               uint8_t value = tFrameData[tIdx] >> 8;
               pixel[0] = value;
               pixel[1] = value;
               pixel[2] = value;
        });
        mBytesPerPixel = 3;
    } else if (type == openni::SensorType::SENSOR_COLOR) {
        cv::Vec3b *tFrameData = (cv::Vec3b*)frame.getData();
        mFrame = cv::Mat(frame.getHeight(), frame.getWidth(), CV_8UC3);
        mFrame.forEach<cv::Vec3b>([&](cv::Vec3b &pixel, const int *position) -> void {
               int idx = position[0] * width + position[1];
               pixel = tFrameData[idx];
        });
        mBytesPerPixel = 3;
    } else if (type == openni::SensorType::SENSOR_DEPTH) {
        uint16_t *tFrameData = (uint16_t*)frame.getData();
        //mFrame = cv::Mat(height, width, CV_8UC3);
        //XXX for some reason the videostream incorrectly reports the dimensions of the frame...
        mFrame = cv::Mat(424, 512, CV_8UC3);
        mFrame.forEach<cv::Vec3b>([&](cv::Vec3b &pixel, const int *position) -> void {
               int tIdx = position[0] * width + position[1];
               uint8_t value = tFrameData[tIdx] >> 4;
               pixel[0] = value;
               pixel[1] = value;
               pixel[2] = value;
        });
        mBytesPerPixel = 3;
    } else {
        logger->log(libfreenect2::Logger::Error, "Sensortype not supported!");
        exit(0x0);

    }
}


Frame::Frame(openni::SensorType type, const cv::Mat &img)
{
    mSensorType = type;
    mUserTracker = NULL;
    switch(type) {
        case openni::SensorType::SENSOR_IR:
            mFrame = img.clone();
            mBytesPerPixel = 3;
            break;
        case openni::SensorType::SENSOR_COLOR:
            mFrame = img.clone();
            mBytesPerPixel = 3;
            break;
        case openni::SensorType::SENSOR_DEPTH:
            mFrame = img.clone();
            mBytesPerPixel = 3;
            break;
        default:
            std::cerr << "Sensortype not supported!" << std::endl;
            exit(0x0);
            break;
    }
}


Frame::~Frame()
{
}


int Frame::getHeight() const
{
    return mFrame.rows;
}


int Frame::getWidth() const
{
    return mFrame.cols;
}


size_t Frame::getBytesPerPixel() const
{
    return mBytesPerPixel;
}


size_t Frame::getDataSize() const
{
    return mBytesPerPixel * mFrame.cols * mFrame.rows;
}


const cv::Mat &Frame::getMat()
{
    return mFrame;
}


unsigned char *Frame::getData() const
{
    return mFrame.data;
}


bool Frame::empty() const
{
    return mFrame.empty();
}


cv::Point3_<float> Frame::getDepthPixel(int i, int j) const
{
    float z;
    if (i < 0 && i >= mFrame.cols && j < 0 && j >= mFrame.rows) {
        return cv::Point3_<float>(0, 0, 0);
    }
    if (mSensorType == openni::SensorType::SENSOR_DEPTH) {
        z = mFrame.at<cv::Point3_<uint8_t>>(j, i).x << 4;
        return cv::Point3_<float>(i, j, z);
    } //IR frame
    return cv::Point3_<float>(0, 0, 0);
}


cv::Point3_<float> Frame::getJointPixel(int i, int j) const
{
    float x, y;
    if (i < 0 && i >= mFrame.cols && j < 0 && j >= mFrame.rows) {
        return cv::Point3_<float>(0, 0, 0);
    }
    
    int z = mFrame.at<cv::Point3_<uint8_t>>(j, i).x << 4;

    assert(mUserTracker != NULL);
    mUserTracker->convertDepthCoordinatesToJoint(i, j, z, &x, &y);
    return cv::Point3_<float>(x, y, z);
}


cv::Vec3b Frame::getRGBPixel(int i, int j) const
{
    if (i < 0 && i >= mFrame.cols && j < 0 && j >= mFrame.rows) {
        return cv::Vec3b(0, 0, 0);
    }
    return mFrame.at<cv::Vec3b>(j, i);
}


openni::SensorType Frame::getType()
{
    return mSensorType;
}


void Frame::setUserTracker(nite::UserTracker *userTracker) 
{
    mUserTracker = userTracker;
}


void Frame::drawSkeleton(const Body &tBody)
{
    assert(mUserTracker != NULL);
    glm::vec3 pHEAD = tBody.getJointAbsPosition(nite::JOINT_HEAD);
    glm::vec3 pNECK = tBody.getJointAbsPosition(nite::JOINT_NECK);
    glm::vec3 pL_SHOULDER = tBody.getJointAbsPosition(nite::JOINT_LEFT_SHOULDER);
    glm::vec3 pR_SHOULDER = tBody.getJointAbsPosition(nite::JOINT_RIGHT_SHOULDER);
    glm::vec3 pL_ELBOW = tBody.getJointAbsPosition(nite::JOINT_LEFT_ELBOW);
    glm::vec3 pR_ELBOW = tBody.getJointAbsPosition(nite::JOINT_RIGHT_ELBOW);
    glm::vec3 pL_HAND = tBody.getJointAbsPosition(nite::JOINT_LEFT_HAND);
    glm::vec3 pR_HAND = tBody.getJointAbsPosition(nite::JOINT_RIGHT_HAND);
    glm::vec3 pTORSO = tBody.getJointAbsPosition(nite::JOINT_TORSO);
    glm::vec3 pL_HIP = tBody.getJointAbsPosition(nite::JOINT_LEFT_HIP);
    glm::vec3 pR_HIP = tBody.getJointAbsPosition(nite::JOINT_RIGHT_HIP);
    glm::vec3 pL_KNEE = tBody.getJointAbsPosition(nite::JOINT_LEFT_KNEE);
    glm::vec3 pR_KNEE = tBody.getJointAbsPosition(nite::JOINT_RIGHT_KNEE);
    glm::vec3 pL_FOOT = tBody.getJointAbsPosition(nite::JOINT_LEFT_FOOT);
    glm::vec3 pR_FOOT = tBody.getJointAbsPosition(nite::JOINT_RIGHT_FOOT);

    cv::Point2f HEAD;
    cv::Point2f NECK;
    cv::Point2f L_SHOULDER;
    cv::Point2f R_SHOULDER;
    cv::Point2f L_ELBOW;
    cv::Point2f R_ELBOW;
    cv::Point2f L_HAND;
    cv::Point2f R_HAND;
    cv::Point2f TORSO;
    cv::Point2f L_HIP;
    cv::Point2f R_HIP;
    cv::Point2f L_KNEE;
    cv::Point2f R_KNEE;
    cv::Point2f L_FOOT;
    cv::Point2f R_FOOT;

    mUserTracker->convertJointCoordinatesToDepth(pHEAD.x, pHEAD.y, pHEAD.z, &HEAD.x, &HEAD.y);
    mUserTracker->convertJointCoordinatesToDepth(pNECK.x, pNECK.y, pNECK.z, &NECK.x, &NECK.y);
    mUserTracker->convertJointCoordinatesToDepth(pL_SHOULDER.x, pL_SHOULDER.y, pL_SHOULDER.z, &L_SHOULDER.x, &L_SHOULDER.y);
    mUserTracker->convertJointCoordinatesToDepth(pR_SHOULDER.x, pR_SHOULDER.y, pR_SHOULDER.z, &R_SHOULDER.x, &R_SHOULDER.y);
    mUserTracker->convertJointCoordinatesToDepth(pL_ELBOW.x, pL_ELBOW.y, pL_ELBOW.z, &L_ELBOW.x, &L_ELBOW.y);
    mUserTracker->convertJointCoordinatesToDepth(pR_ELBOW.x, pR_ELBOW.y, pR_ELBOW.z, &R_ELBOW.x, &R_ELBOW.y);
    mUserTracker->convertJointCoordinatesToDepth(pL_HAND.x, pL_HAND.y, pL_HAND.z, &L_HAND.x, &L_HAND.y);
    mUserTracker->convertJointCoordinatesToDepth(pR_HAND.x, pR_HAND.y, pR_HAND.z, &R_HAND.x, &R_HAND.y);
    mUserTracker->convertJointCoordinatesToDepth(pTORSO.x, pTORSO.y, pTORSO.z, &TORSO.x, &TORSO.y);
    mUserTracker->convertJointCoordinatesToDepth(pL_HIP.x, pL_HIP.y, pL_HIP.z, &L_HIP.x, &L_HIP.y);
    mUserTracker->convertJointCoordinatesToDepth(pR_HIP.x, pR_HIP.y, pR_HIP.z, &R_HIP.x, &R_HIP.y);
    mUserTracker->convertJointCoordinatesToDepth(pL_KNEE.x, pL_KNEE.y, pL_KNEE.z, &L_KNEE.x, &L_KNEE.y);
    mUserTracker->convertJointCoordinatesToDepth(pR_KNEE.x, pR_KNEE.y, pR_KNEE.z, &R_KNEE.x, &R_KNEE.y);
    mUserTracker->convertJointCoordinatesToDepth(pL_FOOT.x, pL_FOOT.y, pL_FOOT.z, &L_FOOT.x, &L_FOOT.y);
    mUserTracker->convertJointCoordinatesToDepth(pR_FOOT.x, pR_FOOT.y, pR_FOOT.z, &R_FOOT.x, &R_FOOT.y);


    std::cout << pTORSO.x << ", " << pTORSO.y << ", " << pTORSO.z << std::endl;
    std::cout << TORSO.x << ", " << TORSO.y << ", " << pTORSO.z << std::endl;
    //mUserTracker->convertDepthCoordinatesToJoint(TORSO.x, TORSO.y, pTORSO.z, &pTORSO.x, &pTORSO.y);
    //std::cout << pTORSO.x << ", " << pTORSO.y << ", " << pTORSO.z << std::endl;
    //exit(0x0);

    // draw joints
    cv::circle(mFrame, HEAD, 1, cv::Scalar(0, 0, 256));
    cv::circle(mFrame, NECK, 1, cv::Scalar(0, 0, 256));
    cv::circle(mFrame, L_SHOULDER, 1, cv::Scalar(0, 0, 256));
    cv::circle(mFrame, R_SHOULDER, 1, cv::Scalar(0, 0, 256));
    cv::circle(mFrame, L_ELBOW, 1, cv::Scalar(0, 0, 256));
    cv::circle(mFrame, R_ELBOW, 1, cv::Scalar(0, 0, 256));
    cv::circle(mFrame, L_HAND, 1, cv::Scalar(0, 0, 256));
    cv::circle(mFrame, R_HAND, 1, cv::Scalar(0, 0, 256));
    cv::circle(mFrame, TORSO, 1, cv::Scalar(0, 0, 256));
    cv::circle(mFrame, L_HIP, 1, cv::Scalar(0, 0, 256));
    cv::circle(mFrame, R_HIP, 1, cv::Scalar(0, 0, 256));
    cv::circle(mFrame, L_KNEE, 1, cv::Scalar(0, 0, 256));
    cv::circle(mFrame, R_KNEE, 1, cv::Scalar(0, 0, 256));
    cv::circle(mFrame, L_FOOT, 1, cv::Scalar(0, 0, 256));
    cv::circle(mFrame, R_FOOT, 1, cv::Scalar(0, 0, 256));

    // draw bones
    const int LINE_THICKNESS = 2;
    cv::line(mFrame, HEAD, NECK, cv::Scalar(0, 0, 256), LINE_THICKNESS);
    cv::line(mFrame, NECK, TORSO, cv::Scalar(0, 0, 256), LINE_THICKNESS);
    cv::line(mFrame, TORSO, L_HIP, cv::Scalar(0, 0, 256), LINE_THICKNESS);
    cv::line(mFrame, TORSO, R_HIP, cv::Scalar(0, 0, 256), LINE_THICKNESS);
    cv::line(mFrame, TORSO, L_SHOULDER, cv::Scalar(0, 0, 256), LINE_THICKNESS);
    cv::line(mFrame, L_SHOULDER, L_ELBOW, cv::Scalar(0, 0, 256), LINE_THICKNESS);
    cv::line(mFrame, L_ELBOW, L_HAND, cv::Scalar(0, 0, 256), LINE_THICKNESS);
    cv::line(mFrame, TORSO, R_SHOULDER, cv::Scalar(0, 0, 256), LINE_THICKNESS);
    cv::line(mFrame, R_SHOULDER, R_ELBOW, cv::Scalar(0, 0, 256), LINE_THICKNESS);
    cv::line(mFrame, R_ELBOW, R_HAND, cv::Scalar(0, 0, 256), LINE_THICKNESS);
    cv::line(mFrame, L_HIP, L_KNEE, cv::Scalar(0, 0, 256), LINE_THICKNESS);
    cv::line(mFrame, L_KNEE, L_FOOT, cv::Scalar(0, 0, 256), LINE_THICKNESS);
    cv::line(mFrame, R_HIP, R_KNEE, cv::Scalar(0, 0, 256), LINE_THICKNESS);
    cv::line(mFrame, R_KNEE, R_FOOT, cv::Scalar(0, 0, 256), LINE_THICKNESS);
}


void Frame::drawChessboard(cv::Size boardSize, std::vector<cv::Point2f> &corners)
{
    cv::drawChessboardCorners(mFrame, boardSize, corners, true);
}


void Frame::clear()
{
    memset(mFrame.data, 0, getDataSize());
}


void Frame::draw3DPoints(std::vector<cv::Point3f> points)
{
    cv::Point2f point2d;
    assert(mUserTracker != NULL);
    for (auto &point3d : points) {
        mUserTracker->convertJointCoordinatesToDepth(point3d.x, point3d.y, point3d.z, &point2d.x, &point2d.y);
        cv::circle(mFrame, point2d, 3, cv::Scalar(256, 0, 0));
    }
}


void Frame::drawFrameAxes(cv::Mat cam, cv::Mat dist, cv::Mat rvec, cv::Mat tvec)
{
    cv::drawFrameAxes(mFrame, cam, dist, rvec, tvec, 0.5f, 4);
}


void Frame::show(const std::string &title) const
{
    cv::imshow(title, mFrame);
    cv::waitKey(0);
    cv::destroyWindow(title);
}


