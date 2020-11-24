#include "frame.h"

#include <iostream>


// frameType == 0 is depthframe, frameType == 1 is colorframe, frameType == 2 is bodyframe
Frame::Frame(const openni::VideoFrameRef &frame, unsigned char frameType)
{
    if (frameType == 0) {
        void *data = malloc(frame.getDataSize());
        memcpy(data, frame.getData(), frame.getDataSize());
        cv::Mat tFrame(frame.getHeight(), frame.getWidth(), CV_16U, data);
        mFrame = new cv::Mat;
        tFrame.convertTo(*mFrame, CV_32F);
        mBytesPerPixel = 4;
    } else if (frameType == 1) {
        void *data = malloc(frame.getDataSize());
        memcpy(data, frame.getData(), frame.getDataSize());
        mFrame = new cv::Mat(frame.getHeight(), frame.getWidth(), CV_8UC3, data);
        mBytesPerPixel = 3;
    } else if (frameType == 2) {
        void *data = malloc(frame.getWidth() * frame.getHeight() * 3);
        uint16_t *tFrameData = (uint16_t*)frame.getData();
        int width = frame.getWidth();
        mFrame = new cv::Mat(frame.getHeight(), frame.getWidth(), CV_8UC3, data);
        mFrame->forEach<cv::Point3_<uint8_t>>([&](cv::Point3_<uint8_t> &pixel, const int *position) -> void {
                int tIdx = position[0] * width + position[1];
                uint8_t value = tFrameData[tIdx] >> 4;
                pixel.x = value;
                pixel.y = value;
                pixel.z = value;
        });
        mBytesPerPixel = 3;
    } else {
        std::cerr << "Sensortype not supported!" << std::endl;
        exit(0x0);
    }
}

Frame::~Frame()
{
    delete mFrame;
}


int Frame::getHeight() const
{
    return mFrame->rows;
}

int Frame::getWidth() const
{
    return mFrame->cols;
}

size_t Frame::getBytesPerPixel() const
{
    return mBytesPerPixel;
}

size_t Frame::getDataSize() const
{
    return mBytesPerPixel * mFrame->cols * mFrame->rows;
}

unsigned char *Frame::getData() const
{
    return mFrame->data;
}


void Frame::drawSkeleton(const nite::UserTracker &tUserTracker, const Body &tBody)
{
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
 
     tUserTracker.convertJointCoordinatesToDepth(pHEAD.x, pHEAD.y, pHEAD.z, &HEAD.x, &HEAD.y);
     tUserTracker.convertJointCoordinatesToDepth(pNECK.x, pNECK.y, pNECK.z, &NECK.x, &NECK.y);
     tUserTracker.convertJointCoordinatesToDepth(pL_SHOULDER.x, pL_SHOULDER.y, pL_SHOULDER.z, &L_SHOULDER.x, &L_SHOULDER.y);
     tUserTracker.convertJointCoordinatesToDepth(pR_SHOULDER.x, pR_SHOULDER.y, pR_SHOULDER.z, &R_SHOULDER.x, &R_SHOULDER.y);
     tUserTracker.convertJointCoordinatesToDepth(pL_ELBOW.x, pL_ELBOW.y, pL_ELBOW.z, &L_ELBOW.x, &L_ELBOW.y);
     tUserTracker.convertJointCoordinatesToDepth(pR_ELBOW.x, pR_ELBOW.y, pR_ELBOW.z, &R_ELBOW.x, &R_ELBOW.y);
     tUserTracker.convertJointCoordinatesToDepth(pL_HAND.x, pL_HAND.y, pL_HAND.z, &L_HAND.x, &L_HAND.y);
     tUserTracker.convertJointCoordinatesToDepth(pR_HAND.x, pR_HAND.y, pR_HAND.z, &R_HAND.x, &R_HAND.y);
     tUserTracker.convertJointCoordinatesToDepth(pTORSO.x, pTORSO.y, pTORSO.z, &TORSO.x, &TORSO.y);
     tUserTracker.convertJointCoordinatesToDepth(pL_HIP.x, pL_HIP.y, pL_HIP.z, &L_HIP.x, &L_HIP.y);
     tUserTracker.convertJointCoordinatesToDepth(pR_HIP.x, pR_HIP.y, pR_HIP.z, &R_HIP.x, &R_HIP.y);
     tUserTracker.convertJointCoordinatesToDepth(pL_KNEE.x, pL_KNEE.y, pL_KNEE.z, &L_KNEE.x, &L_KNEE.y);
     tUserTracker.convertJointCoordinatesToDepth(pR_KNEE.x, pR_KNEE.y, pR_KNEE.z, &R_KNEE.x, &R_KNEE.y);
     tUserTracker.convertJointCoordinatesToDepth(pL_FOOT.x, pL_FOOT.y, pL_FOOT.z, &L_FOOT.x, &L_FOOT.y);
     tUserTracker.convertJointCoordinatesToDepth(pR_FOOT.x, pR_FOOT.y, pR_FOOT.z, &R_FOOT.x, &R_FOOT.y);
 
     // draw joints
     cv::circle(*mFrame, HEAD, 1, cv::Scalar(0, 0, 256));
     cv::circle(*mFrame, NECK, 1, cv::Scalar(0, 0, 256));
     cv::circle(*mFrame, L_SHOULDER, 1, cv::Scalar(0, 0, 256));
     cv::circle(*mFrame, R_SHOULDER, 1, cv::Scalar(0, 0, 256));
     cv::circle(*mFrame, L_ELBOW, 1, cv::Scalar(0, 0, 256));
     cv::circle(*mFrame, R_ELBOW, 1, cv::Scalar(0, 0, 256));
     cv::circle(*mFrame, L_HAND, 1, cv::Scalar(0, 0, 256));
     cv::circle(*mFrame, R_HAND, 1, cv::Scalar(0, 0, 256));
     cv::circle(*mFrame, TORSO, 1, cv::Scalar(0, 0, 256));
     cv::circle(*mFrame, L_HIP, 1, cv::Scalar(0, 0, 256));
     cv::circle(*mFrame, R_HIP, 1, cv::Scalar(0, 0, 256));
     cv::circle(*mFrame, L_KNEE, 1, cv::Scalar(0, 0, 256));
     cv::circle(*mFrame, R_KNEE, 1, cv::Scalar(0, 0, 256));
     cv::circle(*mFrame, L_FOOT, 1, cv::Scalar(0, 0, 256));
     cv::circle(*mFrame, R_FOOT, 1, cv::Scalar(0, 0, 256));

     // draw bones
     const int LINE_THICKNESS = 2;
     cv::line(*mFrame, HEAD, NECK, cv::Scalar(0, 0, 256), LINE_THICKNESS);
     cv::line(*mFrame, NECK, TORSO, cv::Scalar(0, 0, 256), LINE_THICKNESS);
     cv::line(*mFrame, TORSO, L_HIP, cv::Scalar(0, 0, 256), LINE_THICKNESS);
     cv::line(*mFrame, TORSO, R_HIP, cv::Scalar(0, 0, 256), LINE_THICKNESS);
     cv::line(*mFrame, TORSO, L_SHOULDER, cv::Scalar(0, 0, 256), LINE_THICKNESS);
     cv::line(*mFrame, L_SHOULDER, L_ELBOW, cv::Scalar(0, 0, 256), LINE_THICKNESS);
     cv::line(*mFrame, L_ELBOW, L_HAND, cv::Scalar(0, 0, 256), LINE_THICKNESS);
     cv::line(*mFrame, TORSO, R_SHOULDER, cv::Scalar(0, 0, 256), LINE_THICKNESS);
     cv::line(*mFrame, R_SHOULDER, R_ELBOW, cv::Scalar(0, 0, 256), LINE_THICKNESS);
     cv::line(*mFrame, R_ELBOW, R_HAND, cv::Scalar(0, 0, 256), LINE_THICKNESS);
     cv::line(*mFrame, L_HIP, L_KNEE, cv::Scalar(0, 0, 256), LINE_THICKNESS);
     cv::line(*mFrame, L_KNEE, L_FOOT, cv::Scalar(0, 0, 256), LINE_THICKNESS);
     cv::line(*mFrame, R_HIP, R_KNEE, cv::Scalar(0, 0, 256), LINE_THICKNESS);
     cv::line(*mFrame, R_KNEE, R_FOOT, cv::Scalar(0, 0, 256), LINE_THICKNESS);
}

