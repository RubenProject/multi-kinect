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
#include <map>
#include <memory>


enum KINECT_STREAM : int
{
    KINECT_COLOR_0 = 0,
    KINECT_COLOR_1 = 1,
    KINECT_DEPTH_0 = 2,
    KINECT_DEPTH_1 = 3,
    KINECT_STREAM_COUNT = 4
};


enum NITE_STREAM : int
{
    BODY_STREAM_0 = 0,
    BODY_STREAM_1 = 1,
    BODY_STREAM_COUNT = 2
};



class KinectApplication
{
public:
    KinectApplication();
    KinectApplication(bool depth, bool color, bool body, int fps);
    ~KinectApplication();
    void initialize();
    bool update();
private:
    openni::Device mDev_0, mDev_1;
    std::unique_ptr<BodyManager> mBodyManager;
    std::unique_ptr<Viewer> mViewer;
    std::map<int, std::string> mKinectViewerMap;
    
    //config
    bool mDrawDepth, mDrawColor, mDrawBody;
    bool mRecording;
    int mFPS;

    openni::VideoStream mStreams[KINECT_STREAM_COUNT];
    std::queue<openni::VideoFrameRef> mFrames[KINECT_STREAM_COUNT];
    std::queue<uint64_t> mTimestamps[KINECT_STREAM_COUNT];
    std::vector<ColorListener*> mColorListeners;
    std::vector<DepthListener*> mDepthListeners;

    std::vector<nite::UserTracker> mUserTrackers;
    std::vector<std::queue<nite::UserTrackerFrameRef>> mBodyFrames;
    std::vector<std::queue<uint64_t>> mBodyTimes;
    std::vector<BodyListener*> mBodyListeners;
};


