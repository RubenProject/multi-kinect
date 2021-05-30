#pragma once

#include "pch.h"

#include "framelistener.h"
#include "bodylistener.h"
#include "viewer.h"
#include "frame.h"
#include "body.h"
#include "bodymanager.h"
#include "recordmanager.h"
#include "context.h"
#include "logger.h"
#include "chessboard.h"
#include "calib2d.h"
#include "calib3d.h"

class KinectApplication
{
public:
    KinectApplication();
    ~KinectApplication();
    void initialize();
    bool update();
private:
    void handlePlay();
    void handleReplay();
    void handleBodies();
    bool handleCameraCalibration(const int streamIdx);
    void handleHomography();

    void startListeners();
    void stopListeners();

    int getKinectIdx(const int streamIdx);

    std::array<openni::Device, KINECT_COUNT> mDev;
    std::shared_ptr<BodyManager> mBodyManager;
    std::shared_ptr<RecordManager> mRecordManager;
    std::shared_ptr<Viewer> mViewer;
    std::shared_ptr<ChessCalib2d> mCalibrator;
    std::shared_ptr<Calib2d> mCalibrator2d;
    std::shared_ptr<Calib3d> mCalibrator3d;

    std::array<openni::VideoStream, KINECT_STREAM_COUNT> mStreams;
    std::array<std::queue<openni::VideoFrameRef>, KINECT_STREAM_COUNT> mFrames;
    std::array<FrameListener*, KINECT_STREAM_COUNT> mFrameListeners;
    std::array<openni::SensorType, KINECT_STREAM_COUNT> mStreamFrameTypes;

    std::array<nite::UserTracker, KINECT_COUNT> mUserTrackers;
    std::array<std::queue<nite::UserTrackerFrameRef>, KINECT_COUNT> mBodyFrames;
    std::array<std::queue<uint64_t>, KINECT_COUNT> mBodyTimes;
    std::array<BodyListener*, KINECT_COUNT> mBodyListeners;

    uint64_t mReplayStartTime;
};


