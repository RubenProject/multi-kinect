#pragma once

#include "colorlistener.h"
#include "depthlistener.h"
#include "viewer.h"
#include "frame.h"

#include "OpenNI.h"
#include "NiTE.h"

#include <queue>

enum KinectStream
{
    DEPTH_STREAM_0 = 0,
    DEPTH_STREAM_1 = 1,
    COLOR_STREAM_0 = 2,
    COLOR_STREAM_1 = 3,
    STREAM_COUNT = 4
};


class DeviceManager
{
public:
    DeviceManager(Viewer &viewer);
    DeviceManager(Viewer &viewer, bool depth, bool color, bool body);
    ~DeviceManager();
    void initialize();
    bool update();
private:
    openni::Device mDev_0, mDev_1;
    Viewer viewer;

    bool mDrawDepth, mDrawColor, mDrawBody;
    uint64_t mDevTimeDelta;

    openni::VideoStream mColorStream_0, mColorStream_1, mDepthStream_0, mDepthStream_1;
    std::queue<openni::VideoFrameRef> mColorFrames_0, mColorFrames_1, mDepthFrames_0, mDepthFrames_1;

    nite::UserTrackerFrameRef mUserFrame;
    nite::UserTracker mUserTracker_0, mUserTracker_1;
};


