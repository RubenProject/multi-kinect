#include "kinectapp.h"
#include "common.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>



KinectApplication::KinectApplication()
    :mImmediatePlay(true)
{
    mContext = std::make_shared<Context>();
    mContext->mDrawDepth = false;
    mContext->mDrawColor = true;
    mContext->mDrawBody = true;
    mContext->mPlay = true;
    mContext->mLoad = false;
    mContext->mStartRecord = false;
    mContext->mStopRecord = false;
    mContext->mReplay = false;
    mContext->mStartReplay = false;
    mContext->mStopReplay = false;
    mContext->mExit = false;
    mContext->mConf = 0.6;
    mContext->mFPS = 30;
    mContext->mShaderFolder = "Res/";
    mContext->mRecordFolder = "Rec/";

    assert(!(mContext->mDrawDepth && mContext->mDrawBody));

    openni::OpenNI::initialize();
    openni::Array<openni::DeviceInfo> devInfoList;
    openni::OpenNI::enumerateDevices(&devInfoList);
    assert(devInfoList.getSize() == 6);
    mDev[KINECT_ID_0].open(devInfoList[0].getUri());
    mDev[KINECT_ID_1].open(devInfoList[3].getUri());

    mKinectViewerMap[KINECT_COLOR_0] = "RGB0";
    mKinectViewerMap[KINECT_COLOR_1] = "RGB1"; 
    if (mContext->mDrawDepth){
        mKinectViewerMap[KINECT_DEPTH_0] = "IR0";
        mKinectViewerMap[KINECT_DEPTH_1] = "IR1"; 
    } else if (mContext->mDrawBody) {
        mKinectViewerMap[KINECT_DEPTH_0] = "BODY0";
        mKinectViewerMap[KINECT_DEPTH_1] = "BODY1"; 
    }
}


KinectApplication::KinectApplication(std::shared_ptr<Context> context)
    :mContext(context), mImmediatePlay(true)
{
    assert(!(mContext->mDrawDepth && mContext->mDrawBody));

    openni::OpenNI::initialize();
    openni::Array<openni::DeviceInfo> devInfoList;
    openni::OpenNI::enumerateDevices(&devInfoList);
    assert(devInfoList.getSize() == 6);
    mDev[KINECT_ID_0].open(devInfoList[0].getUri());
    mDev[KINECT_ID_1].open(devInfoList[3].getUri());

    mKinectViewerMap[KINECT_COLOR_0] = "RGB0";
    mKinectViewerMap[KINECT_COLOR_1] = "RGB1"; 
    if (mContext->mDrawDepth){
        mKinectViewerMap[KINECT_DEPTH_0] = "IR0";
        mKinectViewerMap[KINECT_DEPTH_1] = "IR1"; 
    } else if (mContext->mDrawBody) {
        mKinectViewerMap[KINECT_DEPTH_0] = "BODY0";
        mKinectViewerMap[KINECT_DEPTH_1] = "BODY1"; 
    }
}


KinectApplication::~KinectApplication()
{
    mStreams[KINECT_COLOR_0].removeNewFrameListener(mColorListeners[KINECT_ID_0]);
    mStreams[KINECT_COLOR_1].removeNewFrameListener(mColorListeners[KINECT_ID_1]);
    mStreams[KINECT_DEPTH_0].removeNewFrameListener(mDepthListeners[KINECT_ID_0]);
    mStreams[KINECT_DEPTH_1].removeNewFrameListener(mDepthListeners[KINECT_ID_1]);

    if (mContext->mDrawBody){
        mUserTrackers[KINECT_ID_0].removeNewFrameListener(mBodyListeners[KINECT_ID_0]);
        mUserTrackers[KINECT_ID_1].removeNewFrameListener(mBodyListeners[KINECT_ID_1]);
    }

    mStreams[KINECT_COLOR_0].stop();
    mStreams[KINECT_COLOR_1].stop();
    mStreams[KINECT_DEPTH_0].stop();
    mStreams[KINECT_DEPTH_1].stop();

    mDev[KINECT_ID_0].close();
    mDev[KINECT_ID_1].close();
}


void KinectApplication::initialize()
{
    mViewer = std::make_shared<Viewer>(mContext);
    mViewer->initialize();

    mStreams[KINECT_COLOR_0].create(mDev[KINECT_ID_0], openni::SENSOR_COLOR);
    mStreams[KINECT_COLOR_1].create(mDev[KINECT_ID_1], openni::SENSOR_COLOR);
    mStreams[KINECT_DEPTH_0].create(mDev[KINECT_ID_0], openni::SENSOR_DEPTH);
    mStreams[KINECT_DEPTH_1].create(mDev[KINECT_ID_1], openni::SENSOR_DEPTH);

    mColorListeners[KINECT_ID_0] = new ColorListener(&mFrames[KINECT_COLOR_0], mContext->mFPS);
    mColorListeners[KINECT_ID_1] = new ColorListener(&mFrames[KINECT_COLOR_1], mContext->mFPS);
    mDepthListeners[KINECT_ID_0] = new DepthListener(&mFrames[KINECT_DEPTH_0], mContext->mFPS);
    mDepthListeners[KINECT_ID_1] = new DepthListener(&mFrames[KINECT_DEPTH_1], mContext->mFPS);

    mStreams[KINECT_COLOR_0].addNewFrameListener(mColorListeners[KINECT_ID_0]);
    mStreams[KINECT_COLOR_1].addNewFrameListener(mColorListeners[KINECT_ID_1]);
    mStreams[KINECT_DEPTH_0].addNewFrameListener(mDepthListeners[KINECT_ID_0]);
    mStreams[KINECT_DEPTH_1].addNewFrameListener(mDepthListeners[KINECT_ID_1]);

    if (mContext->mDrawBody){
        nite::NiTE::initialize();

        mBodyManager = std::make_shared<BodyManager>(mContext);
        mRecordManager = std::make_shared<RecordManager>(mContext);

        mUserTrackers[KINECT_ID_0].create(&mDev[KINECT_ID_0]);
        mUserTrackers[KINECT_ID_1].create(&mDev[KINECT_ID_1]);

        mBodyListeners[KINECT_ID_0] = new BodyListener(&mBodyFrames[KINECT_ID_0], &mBodyTimes[KINECT_ID_0], mContext->mFPS);
        mBodyListeners[KINECT_ID_1] = new BodyListener(&mBodyFrames[KINECT_ID_1], &mBodyTimes[KINECT_ID_1], mContext->mFPS);

        mUserTrackers[KINECT_ID_0].addNewFrameListener(mBodyListeners[KINECT_ID_0]);
        mUserTrackers[KINECT_ID_1].addNewFrameListener(mBodyListeners[KINECT_ID_1]);

        mBodyManager->setUserTrackers(mUserTrackers);
        mBodyManager->setRecordManager(mRecordManager);
    } 

    mStreams[KINECT_COLOR_0].start();
    mStreams[KINECT_COLOR_1].start();
    mStreams[KINECT_DEPTH_0].start();
    mStreams[KINECT_DEPTH_1].start();
}


void KinectApplication::startListeners()
{
    mColorListeners[KINECT_ID_0]->startListening();
    mColorListeners[KINECT_ID_1]->startListening();
    mDepthListeners[KINECT_ID_0]->startListening();
    mDepthListeners[KINECT_ID_1]->startListening();
    if (mContext->mDrawBody){
        mBodyListeners[KINECT_ID_0]->startListening();
        mBodyListeners[KINECT_ID_1]->startListening();
    }
}


void KinectApplication::stopListeners()
{
    mColorListeners[KINECT_ID_0]->stopListening();
    mColorListeners[KINECT_ID_1]->stopListening();
    mDepthListeners[KINECT_ID_0]->stopListening();
    mDepthListeners[KINECT_ID_1]->stopListening();
    if (mContext->mDrawBody){
        mBodyListeners[KINECT_ID_0]->stopListening();
        mBodyListeners[KINECT_ID_1]->stopListening();
    }
    //clear the queue's
    for (int i = 0; i < KINECT_COUNT; ++i){
        std::queue<nite::UserTrackerFrameRef> empty;
        std::swap(mBodyFrames[i], empty);
    }
    //clear the queue's
    for (int i = 0; i < KINECT_STREAM_COUNT; ++i){
        std::queue<openni::VideoFrameRef> empty;
        std::swap(mFrames[i], empty);
    }
}


void KinectApplication::handleBodies()
{
    for (int i = 0; i < KINECT_COUNT; ++i){
        while (mImmediatePlay && mBodyFrames[i].size() > 2){
            mBodyFrames[i].pop();
            mBodyTimes[i].pop();
        }
        if (!mBodyFrames[i].empty()){
            mBodyManager->addBody(mBodyFrames[i].front(), i, mBodyTimes[i].front());
            mBodyFrames[i].pop();
            mBodyTimes[i].pop();
        }
    }
}


void KinectApplication::handlePlay()
{
    for (int i = 0; i < KINECT_STREAM_COUNT; ++i){
        while (mImmediatePlay && mFrames[i].size() > 2){
            mFrames[i].pop();
        }
        if (!mFrames[i].empty()) {
            auto frame = std::make_unique<Frame>(mFrames[i].front(), mKinectViewerMap[i]);
            if (mContext->mDrawBody && i == KINECT_DEPTH_0 
                && mBodyManager->getReadyState(KINECT_ID_0)) {
                const Body &body = mBodyManager->getBody(KINECT_ID_0);
                frame->drawSkeleton(mUserTrackers[KINECT_ID_1], body);
            }
            if (mContext->mDrawBody && i == KINECT_DEPTH_1 
                && mBodyManager->getReadyState(KINECT_ID_1)) {
                const Body &body = mBodyManager->getBody(KINECT_ID_1);
                frame->drawSkeleton(mUserTrackers[KINECT_ID_1], body);
            }
            mViewer->addFrame(mKinectViewerMap[i], std::move(frame));
            mFrames[i].pop();
        }
    }
}


void KinectApplication::handleReplay()
{
    for (int i = 0; i < KINECT_COUNT; ++i){
        const cv::Scalar bgColor(0.f, 0.f, 50.f);
        auto frame = std::make_unique<Frame>(640, 480, bgColor);
        const Body &body = mBodyManager->getNextBody(i, getTimeNow() - mReplayStartTime);
        frame->drawSkeleton(mUserTrackers[i], body);
        mViewer->addFrame(mKinectViewerMap[i], std::move(frame));
    }
}


bool KinectApplication::update()
{
    if (mContext->mStartRecord) {
        std::cerr << "starting recording" << std::endl;
        mBodyManager->startRecording();
        mContext->mStartRecord = false;
        mImmediatePlay = false;
    }

    if (mContext->mStopRecord) {
        std::cerr << "stopping recording" << std::endl;
        mBodyManager->stopRecording();
        mContext->mStopRecord = false;
        mImmediatePlay = true;
    }

    if (mContext->mStartReplay) {
        mContext->mReplay = true;
        mContext->mPlay = false;
        mBodyManager->startReplay();
        this->stopListeners();
        mReplayStartTime = getTimeNow();
        mContext->mStartReplay = false;
    }

    if (mContext->mStopReplay) {
        mContext->mReplay = false;
        mContext->mPlay = true;
        this->startListeners();
        mContext->mStopReplay = false;
    }

    if (mContext->mLoad) {
        //TODO change this to be a prompt
        std::cout << "loading" << std::endl;
        mBodyManager->load(0);
        mContext->mLoad = false;;
    }

    if (mContext->mDrawBody) {
        this->handleBodies();
    }

    if (mContext->mPlay) {
        this->handlePlay();
    } 

    if (mContext->mReplay) {
        this->handleReplay();
    }

    return mContext->mExit || mViewer->render();
}


