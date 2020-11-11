#include "devicemanager.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <chrono>


inline uint64_t current_time()
{
    using namespace std::chrono;
    milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    return ms.count();
}


DeviceManager::DeviceManager(Viewer &viewer)
    : viewer(viewer), mDrawDepth(false), mDrawColor(true), mDrawBody(true)
{
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


DeviceManager::DeviceManager(Viewer &viewer, bool depth, bool color, bool body) 
    : viewer(viewer), mDrawDepth(depth), mDrawColor(color), mDrawBody(body)
{
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

    mColorStream_0.addNewFrameListener(new ColorListener(&mColorFrames_0));
    mColorStream_1.addNewFrameListener(new ColorListener(&mColorFrames_1));
    mDepthStream_0.addNewFrameListener(new DepthListener(&mDepthFrames_0));
    mDepthStream_1.addNewFrameListener(new DepthListener(&mDepthFrames_1));

    mColorStream_0.start();
    mColorStream_1.start();
    mDepthStream_0.start();
    mDepthStream_1.start();
    viewer.initialize();
}


bool DeviceManager::update()
{
    Frame *frame;
    const int MAX_DELAY = 3;

    if (mDrawDepth && !mDepthFrames_0.empty()) {
        if (mDepthFrames_0.size() < MAX_DELAY){
            frame = new Frame(mDepthFrames_0.front(), 0);
            viewer.addFrame("IR0", frame);
        }
        mDepthFrames_0.pop();
    }
    if (mDrawDepth && !mDepthFrames_1.empty()) {
        if (mDepthFrames_1.size() < MAX_DELAY){
            frame = new Frame(mDepthFrames_1.front(), 0);
            viewer.addFrame("IR1", frame);
        }
        mDepthFrames_1.pop();
    }
    if (mDrawColor && !mColorFrames_0.empty()) {
        if (mColorFrames_0.size() < MAX_DELAY){
            frame = new Frame(mColorFrames_0.front(), 1);
            viewer.addFrame("RGB0", frame);
        }
        mColorFrames_0.pop();
    }
    if (mDrawColor && !mColorFrames_1.empty()) {
        if (mColorFrames_1.size() < MAX_DELAY){
            frame = new Frame(mColorFrames_1.front(), 1);
            viewer.addFrame("RGB1", frame);
        }
        mColorFrames_1.pop();
    }
    if (mDrawBody && !mDepthFrames_0.empty()) {
        if (mDepthFrames_0.size() < MAX_DELAY){
            frame = new Frame(mDepthFrames_0.front(), 2);
            viewer.addFrame("BODY0", frame);
        }
        mDepthFrames_0.pop();

        mUserTracker_0.readFrame(&mUserFrame);
        const nite::Array<nite::UserData> &mUsers = mUserFrame.getUsers();
        for (int i = 0; i < mUsers.getSize(); ++i){
            const nite::UserData &tUser = mUsers[i];
            if (tUser.isNew()){
                mUserTracker_0.startSkeletonTracking(tUser.getId());
            }
            const nite::Skeleton &tSkeleton = tUser.getSkeleton();
            frame->drawSkeleton(mUserTracker_0, tSkeleton);
        }
    }
    if (mDrawBody && !mDepthFrames_1.empty()) {
        if (mDepthFrames_1.size() < MAX_DELAY){
            frame = new Frame(mDepthFrames_1.front(), 2);
            viewer.addFrame("BODY1", frame);
        }
        mDepthFrames_1.pop();

        mUserTracker_1.readFrame(&mUserFrame);
        const nite::Array<nite::UserData> &mUsers = mUserFrame.getUsers();
        for (int i = 0; i < mUsers.getSize(); ++i){
            const nite::UserData &tUser = mUsers[i];
            if (tUser.isNew()){
                mUserTracker_1.startSkeletonTracking(tUser.getId());
            }
            const nite::Skeleton &tSkeleton = tUser.getSkeleton();
            frame->drawSkeleton(mUserTracker_1, tSkeleton);
        }
    }

    return viewer.render();
}


