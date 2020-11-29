#include "kinectapp.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <chrono>


inline uint64_t getTimeNow()
{
    using namespace std::chrono;
    milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    return ms.count();
}


KinectApplication::KinectApplication()
    : mDrawDepth(true), mDrawColor(true), mDrawBody(false), mFPS(30)
{
    mRecording = false;
    assert(!(mDrawDepth && mDrawBody));
    openni::OpenNI::initialize();
    openni::Array<openni::DeviceInfo> devInfoList;
    openni::OpenNI::enumerateDevices(&devInfoList);
    assert(devInfoList.getSize() == 6);
    mDev_0.open(devInfoList[0].getUri());
    mDev_1.open(devInfoList[3].getUri());

    mKinectViewerMap[KINECT_COLOR_0] = "RGB0";
    mKinectViewerMap[KINECT_COLOR_1] = "RGB1"; 
    if (mDrawDepth){
        mKinectViewerMap[KINECT_DEPTH_0] = "IR0";
        mKinectViewerMap[KINECT_DEPTH_1] = "IR1"; 
    } else if (mDrawBody) {
        mKinectViewerMap[KINECT_DEPTH_0] = "BODY0";
        mKinectViewerMap[KINECT_DEPTH_1] = "BODY1"; 
    }
}


KinectApplication::KinectApplication(bool depth, bool color, bool body, int fps) 
    : mDrawDepth(depth), mDrawColor(color), mDrawBody(body), mFPS(fps)
{
    mRecording = false;
    openni::OpenNI::initialize();
    openni::Array<openni::DeviceInfo> devInfoList;
    openni::OpenNI::enumerateDevices(&devInfoList);
    assert(devInfoList.getSize() == 6);
    mDev_0.open(devInfoList[0].getUri());
    mDev_1.open(devInfoList[3].getUri());

    mKinectViewerMap[KINECT_COLOR_0] = "RGB0";
    mKinectViewerMap[KINECT_COLOR_1] = "RGB1"; 
    if (mDrawDepth){
        mKinectViewerMap[KINECT_DEPTH_0] = "IR0";
        mKinectViewerMap[KINECT_DEPTH_1] = "IR1"; 
    } else if (mDrawBody) {
        mKinectViewerMap[KINECT_DEPTH_0] = "BODY0";
        mKinectViewerMap[KINECT_DEPTH_1] = "BODY1"; 
    }
}


KinectApplication::~KinectApplication()
{
    mStreams[KINECT_COLOR_0].removeNewFrameListener(mColorListeners[0]);
    mStreams[KINECT_COLOR_1].removeNewFrameListener(mColorListeners[1]);
    mStreams[KINECT_DEPTH_0].removeNewFrameListener(mDepthListeners[0]);
    mStreams[KINECT_DEPTH_1].removeNewFrameListener(mDepthListeners[1]);

    if (mDrawBody){
        mUserTrackers[BODY_STREAM_0].removeNewFrameListener(mBodyListeners[0]);
        mUserTrackers[BODY_STREAM_1].removeNewFrameListener(mBodyListeners[1]);
    }

    mStreams[KINECT_COLOR_0].stop();
    mStreams[KINECT_COLOR_1].stop();
    mStreams[KINECT_DEPTH_0].stop();
    mStreams[KINECT_DEPTH_1].stop();

    mDev_0.close();
    mDev_1.close();
}


void KinectApplication::initialize()
{
    mBodyManager = std::make_unique<BodyManager>();
    mViewer = std::make_unique<Viewer>();

    mUserTrackers = std::vector<nite::UserTracker>(BODY_STREAM_COUNT);
    mBodyFrames = std::vector<std::queue<nite::UserTrackerFrameRef>>(BODY_STREAM_COUNT);
    mBodyTimes = std::vector<std::queue<uint64_t>>(BODY_STREAM_COUNT);

    nite::NiTE::initialize();
    mViewer->initialize();

    mStreams[KINECT_COLOR_0].create(mDev_0, openni::SENSOR_COLOR);
    mStreams[KINECT_COLOR_1].create(mDev_1, openni::SENSOR_COLOR);
    mStreams[KINECT_DEPTH_0].create(mDev_0, openni::SENSOR_DEPTH);
    mStreams[KINECT_DEPTH_1].create(mDev_1, openni::SENSOR_DEPTH);

    mColorListeners.push_back(new ColorListener(&mFrames[KINECT_COLOR_0], &mTimestamps[KINECT_COLOR_0], mFPS));
    mColorListeners.push_back(new ColorListener(&mFrames[KINECT_COLOR_1], &mTimestamps[KINECT_COLOR_1], mFPS));
    mDepthListeners.push_back(new DepthListener(&mFrames[KINECT_DEPTH_0], &mTimestamps[KINECT_DEPTH_0], mFPS));
    mDepthListeners.push_back(new DepthListener(&mFrames[KINECT_DEPTH_1], &mTimestamps[KINECT_DEPTH_1], mFPS));

    mStreams[KINECT_COLOR_0].addNewFrameListener(mColorListeners[0]);
    mStreams[KINECT_COLOR_1].addNewFrameListener(mColorListeners[1]);
    mStreams[KINECT_DEPTH_0].addNewFrameListener(mDepthListeners[0]);
    mStreams[KINECT_DEPTH_1].addNewFrameListener(mDepthListeners[1]);

    if (mDrawBody){
        mUserTrackers[BODY_STREAM_0].create(&mDev_0);
        mUserTrackers[BODY_STREAM_1].create(&mDev_1);

        mBodyListeners.push_back(new BodyListener(&mBodyFrames[BODY_STREAM_0], &mBodyTimes[BODY_STREAM_0], -1));
        mBodyListeners.push_back(new BodyListener(&mBodyFrames[BODY_STREAM_1], &mBodyTimes[BODY_STREAM_1], -1));

        mUserTrackers[BODY_STREAM_0].addNewFrameListener(mBodyListeners[0]);
        mUserTrackers[BODY_STREAM_1].addNewFrameListener(mBodyListeners[1]);

        mBodyManager->initialize(mUserTrackers);
    }

    mStreams[KINECT_COLOR_0].start();
    mStreams[KINECT_COLOR_1].start();
    mStreams[KINECT_DEPTH_0].start();
    mStreams[KINECT_DEPTH_1].start();
}


bool KinectApplication::update()
{
    if (mDrawBody) {
        for (int i = 0; i < BODY_STREAM_COUNT; ++i){
            if (!mBodyFrames[i].empty()){
                if (mRecording) {
                    mBodyManager->addBody(mBodyFrames[i].front(), i, mBodyTimes[i].front());
                } else {
                    mBodyManager->setBody(mBodyFrames[i].front(), i);
                }
                mBodyFrames[i].pop();
                mBodyTimes[i].pop();
            }
        }
    }

    for (int i = 0; i < KINECT_STREAM_COUNT; ++i){
        if (!mFrames[i].empty()) {
            auto frame = std::make_unique<Frame>(mFrames[i].front(), mKinectViewerMap[i]);

            if (i == KINECT_DEPTH_0) {
                const Body &tBody = mBodyManager->getBody(BODY_STREAM_0);
                frame->drawSkeleton(mUserTrackers[BODY_STREAM_0], tBody);
            }
            if (i == KINECT_DEPTH_1) {
                const Body &tBody = mBodyManager->getBody(BODY_STREAM_1);
                frame->drawSkeleton(mUserTrackers[BODY_STREAM_1], tBody);
            }
            mViewer->addFrame(mKinectViewerMap[i], std::move(frame));

            mFrames[i].pop();
            mTimestamps[i].pop();
        }
    }

    return mViewer->render();
}


