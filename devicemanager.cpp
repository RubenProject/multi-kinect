#include "devicemanager.h"

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


DeviceManager::DeviceManager()
    : mDrawDepth(false), mDrawColor(true), mDrawBody(true)
{
    mMaxDelay = 3;
    mRecording = false;
    assert(!(mDrawDepth && mDrawBody));
    openni::OpenNI::initialize();
    openni::Array<openni::DeviceInfo> devInfoList;
    openni::OpenNI::enumerateDevices(&devInfoList);
    assert(devInfoList.getSize() == 6);
    mDev_0.open(devInfoList[0].getUri());
    mDev_1.open(devInfoList[3].getUri());

    nite::NiTE::initialize();
    mUserTracker_0.create(&mDev_0);
    mUserTracker_1.create(&mDev_1);
}


DeviceManager::DeviceManager(bool depth, bool color, bool body, uint64_t delay) 
    : mDrawDepth(depth), mDrawColor(color), mDrawBody(body)
{
    mMaxDelay = delay;
    mRecording = false;
    openni::OpenNI::initialize();
    openni::Array<openni::DeviceInfo> devInfoList;
    openni::OpenNI::enumerateDevices(&devInfoList);
    assert(devInfoList.getSize() == 6);
    mDev_0.open(devInfoList[0].getUri());
    mDev_1.open(devInfoList[3].getUri());

    nite::NiTE::initialize();
    mUserTracker_0.create(&mDev_0);
    mUserTracker_1.create(&mDev_1);
}


DeviceManager::~DeviceManager()
{
    mColorStream_0.stop();
    mColorStream_1.stop();
    mDepthStream_0.stop();
    mDepthStream_1.stop();
    mDev_0.close();
    mDev_1.close();
}


void DeviceManager::initialize()
{
    mColorStream_0.create(mDev_0, openni::SENSOR_COLOR);
    mColorStream_1.create(mDev_1, openni::SENSOR_COLOR);
    mDepthStream_0.create(mDev_0, openni::SENSOR_DEPTH);
    mDepthStream_1.create(mDev_1, openni::SENSOR_DEPTH);
    mUserTracker_0.create(&mDev_0);
    mUserTracker_1.create(&mDev_1);

    mColorStream_0.addNewFrameListener(new ColorListener(&mColorFrames_0, &mColorTimes_0));
    mColorStream_1.addNewFrameListener(new ColorListener(&mColorFrames_1, &mColorTimes_1));
    mDepthStream_0.addNewFrameListener(new DepthListener(&mDepthFrames_0, &mDepthTimes_0));
    mDepthStream_1.addNewFrameListener(new DepthListener(&mDepthFrames_1, &mDepthTimes_1));
    if (mDrawBody){
        mUserTracker_0.addNewFrameListener(new BodyListener(&mBodyFrames_0, &mBodyTimes_0));
        mUserTracker_1.addNewFrameListener(new BodyListener(&mBodyFrames_1, &mBodyTimes_1));
    }

    mColorStream_0.start();
    mColorStream_1.start();
    mDepthStream_0.start();
    mDepthStream_1.start();

    viewer.initialize();
    bm.initialize(mUserTracker_0, mUserTracker_1);
}


bool DeviceManager::update()
{
    Frame *frame;
    Body *tBody;
    nite::UserTrackerFrameRef tBodyFrame;
    uint64_t tTime;
    if (mDrawDepth && !mDepthFrames_0.empty()) {
        if (mDepthFrames_0.size() < mMaxDelay){
            frame = new Frame(mDepthFrames_0.front(), 0);
            viewer.addFrame("IR0", frame);
        }
        mDepthFrames_0.pop();
        mDepthTimes_0.pop();
    }
    if (mDrawDepth && !mDepthFrames_1.empty()) {
        if (mDepthFrames_1.size() < mMaxDelay){
            frame = new Frame(mDepthFrames_1.front(), 0);
            viewer.addFrame("IR1", frame);
        }
        mDepthFrames_1.pop();
        mDepthTimes_1.pop();
    }
    if (mDrawColor && !mColorFrames_0.empty()) {
        if (mColorFrames_0.size() < mMaxDelay){
            frame = new Frame(mColorFrames_0.front(), 1);
            viewer.addFrame("RGB0", frame);
        }
        mColorFrames_0.pop();
        mColorTimes_0.pop();
    }
    if (mDrawColor && !mColorFrames_1.empty()) {
        if (mColorFrames_1.size() < mMaxDelay){
            frame = new Frame(mColorFrames_1.front(), 1);
            viewer.addFrame("RGB1", frame);
        }
        mColorFrames_1.pop();
        mColorTimes_1.pop();
    }
    // Body handling
    if (mDrawBody && !mBodyFrames_0.empty()){
        tBodyFrame = mBodyFrames_0.front();
        tTime = mBodyTimes_0.front();
        if (mRecording) {
            tBody = bm.addBody(tBodyFrame, 0, tTime);
        } else {
            tBody = bm.getBody(tBodyFrame, 0);
        }
        if (tBody != NULL)
            mBody_0 = *tBody;
        delete tBody;
        mBodyFrames_0.pop();
        mBodyTimes_0.pop();
    }

    if (mDrawBody && !mBodyFrames_1.empty()){
        tBodyFrame = mBodyFrames_1.front();
        tTime = mBodyTimes_1.front();
        if (mRecording) {
            tBody = bm.addBody(tBodyFrame, 1, tTime);
        } else {
            tBody = bm.getBody(tBodyFrame, 1);
        }
        if (tBody != NULL)
            mBody_1 = *tBody;
        delete tBody;
        mBodyFrames_1.pop();
        mBodyTimes_1.pop();
    }

    if (mDrawBody && !mDepthFrames_0.empty()) {
        if (mDepthFrames_0.size() < mMaxDelay){
            frame = new Frame(mDepthFrames_0.front(), 2);
            if (bm.isReady(0)){
                frame->drawSkeleton(mUserTracker_0, mBody_0);
            }
            viewer.addFrame("BODY0", frame);
        }
        mDepthFrames_0.pop();
        mDepthTimes_0.pop();
    }
    if (mDrawBody && !mDepthFrames_1.empty()) {
        if (mDepthFrames_1.size() < mMaxDelay){
            frame = new Frame(mDepthFrames_1.front(), 2);
            if (bm.isReady(1)){
                frame->drawSkeleton(mUserTracker_1, mBody_1);
            }
            viewer.addFrame("BODY1", frame);
        }
        mDepthFrames_1.pop();
        mDepthTimes_1.pop();
    }

    return viewer.render();
}


