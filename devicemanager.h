#pragma once

#include "colorlistener.h"
#include "depthlistener.h"
#include "bodylistener.h"
#include "viewer.h"
#include "frame.h"
#include "body.h"
#include "bodymanager.h"

#include "OpenNI.h"
#include "NiTE.h"

#include <queue>


class DeviceManager
{
public:
    DeviceManager();
    DeviceManager(bool depth, bool color, bool body, uint64_t delay);
    ~DeviceManager();
    void initialize();
    bool update();
private:
    //config
    bool mDrawDepth, mDrawColor, mDrawBody;
    bool mRecording;
    uint64_t mMaxDelay;

    openni::Device mDev_0, mDev_1;
    Viewer viewer;
    BodyManager bm;

    uint64_t mDevTimeDelta;
    openni::VideoStream mColorStream_0, mColorStream_1, mDepthStream_0, mDepthStream_1;
    std::queue<openni::VideoFrameRef> mColorFrames_0, mColorFrames_1, mDepthFrames_0, mDepthFrames_1;
    std::queue<uint64_t> mColorTimes_0, mColorTimes_1, mDepthTimes_0, mDepthTimes_1;

    nite::UserTracker mUserTracker_0, mUserTracker_1;
    Body mBody_0, mBody_1;
    std::queue<nite::UserTrackerFrameRef> mBodyFrames_0, mBodyFrames_1;
    std::queue<uint64_t> mBodyTimes_0, mBodyTimes_1;
};


