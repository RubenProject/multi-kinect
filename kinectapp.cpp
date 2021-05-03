#include "kinectapp.h"

#include "common.h"
#include "context.h"
#include "logger.h"


KinectApplication::KinectApplication()
{
    assert(!(context->mDrawDepth && context->mDrawBody));

    openni::OpenNI::initialize();
    openni::Array<openni::DeviceInfo> devInfoList;
    openni::OpenNI::enumerateDevices(&devInfoList);
    assert(devInfoList.getSize() == 6);
    mDev[KINECT_ID_0].open(devInfoList[0].getUri());
    mDev[KINECT_ID_1].open(devInfoList[3].getUri());
    //mDev[KINECT_ID_0].setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    //mDev[KINECT_ID_1].setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    mDev[KINECT_ID_0].setDepthColorSyncEnabled(true);
    mDev[KINECT_ID_1].setDepthColorSyncEnabled(true);
}


KinectApplication::~KinectApplication()
{
    if (context->mDrawBody){
        //mUserTrackers[KINECT_ID_0].removeNewFrameListener(mBodyListeners[KINECT_ID_0]);
        //mUserTrackers[KINECT_ID_1].removeNewFrameListener(mBodyListeners[KINECT_ID_1]);
        nite::NiTE::shutdown();
    }
    openni::OpenNI::shutdown();


    //mStreams[KINECT_COLOR_0].removeNewFrameListener(mColorListeners[KINECT_ID_0]);
    //mStreams[KINECT_COLOR_1].removeNewFrameListener(mColorListeners[KINECT_ID_1]);
    //mStreams[KINECT_DEPTH_0].removeNewFrameListener(mDepthListeners[KINECT_ID_0]);
    //mStreams[KINECT_DEPTH_1].removeNewFrameListener(mDepthListeners[KINECT_ID_1]);

    //mStreams[KINECT_COLOR_0].stop();
    //mStreams[KINECT_COLOR_1].stop();
    //mStreams[KINECT_DEPTH_0].stop();
    //mStreams[KINECT_DEPTH_1].stop();

    //mDev[KINECT_ID_0].close();
    //mDev[KINECT_ID_1].close();
}


void KinectApplication::initialize()
{
    mViewer = std::make_shared<Viewer>();
    mViewer->initialize();

    mStreams[KINECT_COLOR_0].create(mDev[KINECT_ID_0], openni::SENSOR_COLOR);
    mStreams[KINECT_COLOR_1].create(mDev[KINECT_ID_1], openni::SENSOR_COLOR);
    mStreams[KINECT_DEPTH_0].create(mDev[KINECT_ID_0], openni::SENSOR_DEPTH);
    mStreams[KINECT_DEPTH_1].create(mDev[KINECT_ID_1], openni::SENSOR_DEPTH);

    mColorListeners[KINECT_ID_0] = new ColorListener(&mFrames[KINECT_COLOR_0], context->mFPS);
    mColorListeners[KINECT_ID_1] = new ColorListener(&mFrames[KINECT_COLOR_1], context->mFPS);
    mDepthListeners[KINECT_ID_0] = new DepthListener(&mFrames[KINECT_DEPTH_0], context->mFPS);
    mDepthListeners[KINECT_ID_1] = new DepthListener(&mFrames[KINECT_DEPTH_1], context->mFPS);

    mStreams[KINECT_COLOR_0].addNewFrameListener(mColorListeners[KINECT_ID_0]);
    mStreams[KINECT_COLOR_1].addNewFrameListener(mColorListeners[KINECT_ID_1]);
    mStreams[KINECT_DEPTH_0].addNewFrameListener(mDepthListeners[KINECT_ID_0]);
    mStreams[KINECT_DEPTH_1].addNewFrameListener(mDepthListeners[KINECT_ID_1]);

    if (context->mDrawBody){
        nite::NiTE::initialize();

        mBodyManager = std::make_shared<BodyManager>();
        mRecordManager = std::make_shared<RecordManager>();

        mUserTrackers[KINECT_ID_0].create(&mDev[KINECT_ID_0]);
        mUserTrackers[KINECT_ID_1].create(&mDev[KINECT_ID_1]);

        mBodyListeners[KINECT_ID_0] = new BodyListener(&mBodyFrames[KINECT_ID_0], &mBodyTimes[KINECT_ID_0], context->mFPS);
        mBodyListeners[KINECT_ID_1] = new BodyListener(&mBodyFrames[KINECT_ID_1], &mBodyTimes[KINECT_ID_1], context->mFPS);

        mUserTrackers[KINECT_ID_0].addNewFrameListener(mBodyListeners[KINECT_ID_0]);
        mUserTrackers[KINECT_ID_1].addNewFrameListener(mBodyListeners[KINECT_ID_1]);

        mBodyManager->setUserTrackers(mUserTrackers);
        mBodyManager->setRecordManager(mRecordManager);
        mCalibrator = std::make_shared<ChessCalib2d>();
    } 

    mStreams[KINECT_COLOR_0].start();
    mStreams[KINECT_COLOR_1].start();
    mStreams[KINECT_DEPTH_0].start();
    mStreams[KINECT_DEPTH_1].start();


    // Test setup

    /*
    glm::dmat3 R;
    glm::dvec3 t;
    mCalibrator->getHomography(R, t);
    mBodyManager->setHomography(R, t);
    */


    //stopListeners();
    //mBodyManager->load(1);
    //mBodyManager->process();
    //context->mStartReplay = true;
}


void KinectApplication::startListeners()
{
    mColorListeners[KINECT_ID_0]->startListening();
    mColorListeners[KINECT_ID_1]->startListening();
    mDepthListeners[KINECT_ID_0]->startListening();
    mDepthListeners[KINECT_ID_1]->startListening();
    if (context->mDrawBody){
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
    if (context->mDrawBody){
        mBodyListeners[KINECT_ID_0]->stopListening();
        mBodyListeners[KINECT_ID_1]->stopListening();
    }
    //clear the queue's
    for (int i = 0; i < KINECT_COUNT; ++i){
        std::queue<nite::UserTrackerFrameRef> empty;
        std::swap(mBodyFrames[i], empty);
    }
    for (int i = 0; i < KINECT_STREAM_COUNT; ++i){
        std::queue<openni::VideoFrameRef> empty;
        std::swap(mFrames[i], empty);
    }
}


void KinectApplication::handleBodies()
{
    for (int i = 0; i < KINECT_COUNT; ++i){
        while (context->mImmediate && mBodyFrames[i].size() > 2){
            mBodyFrames[i].pop();
            mBodyTimes[i].pop();
        }
        if (!mBodyFrames[i].empty()){
            mBodyManager->update(mBodyFrames[i].front(), i, mBodyTimes[i].front());
            mBodyFrames[i].pop();
            mBodyTimes[i].pop();
        }
    }
}


void KinectApplication::handlePlay()
{
    for (int i = 0; i < KINECT_STREAM_COUNT; ++i){
        while (context->mImmediate && mFrames[i].size() > 2){
            mFrames[i].pop();
        }
        if (!mFrames[i].empty()) {
            std::string frameHeader = context->mKinectViewerMap[i];
            nite::UserTracker *tracker = &mUserTrackers[i/2];
            auto frame = std::make_shared<Frame>(frameHeader, mFrames[i].front(), tracker);
            if (context->mDrawBody && i == KINECT_DEPTH_0 
                && mBodyManager->getReadyState(KINECT_ID_0)) {
                const Body &body = mBodyManager->getBody(KINECT_ID_0);
                frame->drawSkeleton(body);
            }
            if (context->mDrawBody && i == KINECT_DEPTH_1 
                && mBodyManager->getReadyState(KINECT_ID_1)) {
                const Body &body = mBodyManager->getBody(KINECT_ID_1);
                frame->drawSkeleton(body);
            }
            mViewer->addFrame(frameHeader, frame);
            mFrames[i].pop();
        }
    }
}


void KinectApplication::handleReplay()
{
    for (int i = 0; i < KINECT_COUNT; ++i){
        std::string frameHeader = context->mKinectViewerMap[i];
        const cv::Scalar bgColor(0.f, 0.f, 50.f);
        auto frame = std::make_shared<Frame>(frameHeader, 640, 480, bgColor, &mUserTrackers[i]);
        const Body &body = mBodyManager->getNextBody(i, getTimeNow() - mReplayStartTime);
        frame->drawSkeleton(body);
        mViewer->addFrame(frameHeader, frame);
    }
}


bool KinectApplication::handleCameraCalibration(const int streamIdx)
{
    static uint64_t lastTime[2] = {getTimeNow(), getTimeNow()};
    static int index[2] = {0, 0};
    const std::string streamName = context->mKinectViewerMap[streamIdx];

    if (mCalibrator->readyCamera(streamIdx)) {
        mCalibrator->saveCamera("camera_params", streamIdx);
        return true;
    } else if (mCalibrator->calibrateReadyCamera(streamIdx)) {
        stopListeners();
        mCalibrator->calibrateCamera(streamIdx);
        startListeners();
        return false;
    } else if (mCalibrator->loadSampleCamera(std::to_string(index[streamIdx]), streamIdx)) {
        index[streamIdx]++;
        return false;
    } else if (getTimeNow() - lastTime[streamIdx] > 1000) {
        std::shared_ptr<Frame> frame = mViewer->getFrame(streamName);
        if (mCalibrator->addSampleCamera(frame, streamIdx)) {
            mCalibrator->saveSampleCamera(frame, std::to_string(index[streamIdx]), streamIdx);
            index[streamIdx]++;
        }
        lastTime[streamIdx] = getTimeNow();
        return false;
    }
    return false;
}


void KinectApplication::handleHomography()
{
    static uint64_t lastTime = getTimeNow();
    static int index = 0;

    if (false && mCalibrator->calibrateReadyHomography()) {
        stopListeners();
        mCalibrator->calibrateHomography();
        mCalibrator->saveHomography();
        startListeners();
        context->mHomography = false;
    //} else if (mCalibrator->loadSampleHomography(std::to_string(index), std::to_string(index))) {
        //index++;
    } else { //if (getTimeNow() - lastTime > 1000) {
        std::shared_ptr<Frame> frame0 = mViewer->getFrame("RGB0");
        std::shared_ptr<Frame> frame1 = mViewer->getFrame("RGB1");

        if (mCalibrator->addSampleHomography(frame0, frame1, true)) {
            //mCalibrator->saveSampleHomography(frame0, frame1, std::to_string(index), std::to_string(index));
            index++;
        }
        lastTime = getTimeNow();
    }
}


bool KinectApplication::update()
{
    static uint64_t startTime = getTimeNow();

    if (context->mStartRecord) {
        if (mCalibrator->loadHomography()){
            glm::dmat3 R;
            glm::dvec3 t;
            mCalibrator->getHomography(R, t);
            mBodyManager->setHomography(R, t);
            logger->log(libfreenect2::Logger::Info, "Starting Recording");
            mViewer->setWindowTitle("Recording");
            mBodyManager->startRecording();
            context->mStartRecord = false;
            context->mImmediate = false;
        } else {
            logger->log(libfreenect2::Logger::Info, "No homography found");
        }
    }

    if (context->mStopRecord) {
        logger->log(libfreenect2::Logger::Info, "Stopping Recording");
        mViewer->setWindowTitle("Playing");
        mBodyManager->stopRecording();
        context->mStopRecord = false;
        context->mImmediate = true;
    }

    if (context->mStartReplay) {
        logger->log(libfreenect2::Logger::Info, "Replaying");
        context->mReplay = true;
        context->mPlay = false;
        mBodyManager->startReplay();
        stopListeners();
        mReplayStartTime = getTimeNow();
        context->mStartReplay = false;
    }

    if (context->mStopReplay) {
        logger->log(libfreenect2::Logger::Info, "Stopping Replay");
        mViewer->setWindowTitle("Playing");
        context->mReplay = false;
        context->mPlay = true;
        startListeners();
        context->mStopReplay = false;
    }

    if (context->mLoad) {
        int recordingIdx;
        logger->log(libfreenect2::Logger::Info, "Enter Recording ID to be loaded:");
        std::cin >> recordingIdx;
        logger->log(libfreenect2::Logger::Info, "Loading: " + std::to_string(recordingIdx));
        mBodyManager->load(recordingIdx);
        context->mLoad = false;;
    }

    if (context->mDrawBody) {
        handleBodies();
    }

    if (context->mPlay) {
        handlePlay();
    } 

    if (context->mReplay) {
        handleReplay();
    }

    if (context->mCamera) {
        mViewer->setWindowTitle("Calibrating Cameras");
        bool calibrated = true;
        if (!handleCameraCalibration(KINECT_ID_0)) {
            calibrated = false;
        }
        if (!handleCameraCalibration(KINECT_ID_1)) {
            calibrated = false;
        }
        if (calibrated) {
            context->mCamera = false;
        }
    }

    if (getTimeNow() - startTime >= 1000){
        logger->setLevel(libfreenect2::Logger::None);
        std::shared_ptr<Frame> frame_rgb0 = mViewer->getFrame("RGB0");
        std::shared_ptr<Frame> frame_depth0 = mViewer->getFrame("BODY0");
        mCalibrator->test(frame_rgb0, frame_depth0, 0);

        std::shared_ptr<Frame> frame_rgb1 = mViewer->getFrame("RGB1");
        std::shared_ptr<Frame> frame_depth1 = mViewer->getFrame("BODY1");
        mCalibrator->test(frame_rgb1, frame_depth1, 1);
        //startTime = getTimeNow();
    }

    if (context->mHomography) {
        mViewer->setWindowTitle("Calibrating Homography");
        if (!mCalibrator->readyCamera(KINECT_ID_0) 
            || !mCalibrator->readyCamera(KINECT_ID_1)){
            context->mHomography = false;
        } else { 
            handleHomography();
        }
    }

    return context->mExit || mViewer->render();
}


