#pragma once

#include "colorlistener.h"
#include "depthlistener.h"
#include "bodylistener.h"
#include "viewer.h"
#include "frame.h"
#include "body.h"
#include "bodymanager.h"
#include "recordmanager.h"
#include "context.h"

#include "OpenNI.h"
#include "NiTE.h"

#include <array>
#include <queue>
#include <map>
#include <memory>


class KinectApplication
{
public:
    KinectApplication();
    KinectApplication(std::shared_ptr<Context> context);
    ~KinectApplication();
    void initialize();
    bool update();
private:
    void handlePlay();
    void handleReplay();
    void handleBodies();

    void startListeners();
    void stopListeners();

    std::array<openni::Device, KINECT_COUNT> mDev;
    std::shared_ptr<BodyManager> mBodyManager;
    std::shared_ptr<Viewer> mViewer;
    std::shared_ptr<RecordManager> mRecordManager;
    std::shared_ptr<Context> mContext;
    std::map<int, std::string> mKinectViewerMap;

    std::array<openni::VideoStream, KINECT_STREAM_COUNT> mStreams;
    std::array<std::queue<openni::VideoFrameRef>, KINECT_STREAM_COUNT> mFrames;
    std::array<ColorListener*, KINECT_COUNT> mColorListeners;
    std::array<DepthListener*, KINECT_COUNT> mDepthListeners;

    std::array<nite::UserTracker, KINECT_COUNT> mUserTrackers;
    std::array<std::queue<nite::UserTrackerFrameRef>, KINECT_COUNT> mBodyFrames;
    std::array<std::queue<uint64_t>, KINECT_COUNT> mBodyTimes;
    std::array<BodyListener*, KINECT_COUNT> mBodyListeners;

    uint64_t mReplayStartTime;
    bool mImmediatePlay;
};


