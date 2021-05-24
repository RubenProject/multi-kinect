#include "kinectapp.h"

#include "pch.h"

#include "common.h"
#include "context.h"
#include "logger.h"


KinectApplication::KinectApplication()
{
    //validate config
    assert(!(context->mDrawDepth && context->mDrawBody));
    assert(!(context->mDrawBody && context->mDrawIR));
    assert(!(context->mDrawDepth && context->mDrawIR));

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


    mStreamFrameTypes[0] = openni::SensorType::SENSOR_COLOR;
    mStreamFrameTypes[1] = openni::SensorType::SENSOR_COLOR;
    if (context->mDrawDepth) {
        mStreamFrameTypes[2] = openni::SensorType::SENSOR_DEPTH;
        mStreamFrameTypes[3] = openni::SensorType::SENSOR_DEPTH;
    } else if (context->mDrawIR) {
        mStreamFrameTypes[2] = openni::SensorType::SENSOR_IR;
        mStreamFrameTypes[3] = openni::SensorType::SENSOR_IR;
    }
}


KinectApplication::~KinectApplication()
{
    if (context->mDrawBody){
        nite::NiTE::shutdown();
    }
    openni::OpenNI::shutdown();
}


void KinectApplication::initialize()
{
    mViewer = std::make_shared<Viewer>();
    mViewer->initialize();
    mCalibrator = std::make_shared<ChessCalib2d>();

    mStreams[KINECT_COLOR_0].create(mDev[KINECT_ID_0], openni::SENSOR_COLOR);
    mStreams[KINECT_COLOR_1].create(mDev[KINECT_ID_1], openni::SENSOR_COLOR);
    //mStreams[KINECT_DEPTH_0].create(mDev[KINECT_ID_0], openni::SENSOR_DEPTH);
    //mStreams[KINECT_DEPTH_1].create(mDev[KINECT_ID_1], openni::SENSOR_DEPTH);
    mStreams[KINECT_DEPTH_0].create(mDev[KINECT_ID_0], openni::SENSOR_IR);
    mStreams[KINECT_DEPTH_1].create(mDev[KINECT_ID_1], openni::SENSOR_IR);

    mFrameListeners[0] = new FrameListener(&mFrames[KINECT_COLOR_0], context->mFPS);
    mFrameListeners[1] = new FrameListener(&mFrames[KINECT_COLOR_1], context->mFPS);
    mFrameListeners[2] = new FrameListener(&mFrames[KINECT_DEPTH_0], context->mFPS);
    mFrameListeners[3] = new FrameListener(&mFrames[KINECT_DEPTH_1], context->mFPS);

    mStreams[KINECT_COLOR_0].addNewFrameListener(mFrameListeners[0]);
    mStreams[KINECT_COLOR_1].addNewFrameListener(mFrameListeners[1]);
    mStreams[KINECT_DEPTH_0].addNewFrameListener(mFrameListeners[2]);
    mStreams[KINECT_DEPTH_1].addNewFrameListener(mFrameListeners[3]);

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
    } 

    mStreams[KINECT_COLOR_0].start();
    mStreams[KINECT_COLOR_1].start();
    mStreams[KINECT_DEPTH_0].start();
    mStreams[KINECT_DEPTH_1].start();

    //auto frame_rgb = std::make_shared<Frame>(openni::SensorType::SENSOR_COLOR, 1920, 1080, cv::Scalar(0, 0, 0), nullptr);
    //auto frame_ir = std::make_shared<Frame>(openni::SensorType::SENSOR_IR, 512, 424, cv::Scalar(0, 0, 0), nullptr);
    //mViewer->addFrame(frame_rgb, KINECT_ID_0);
    //mViewer->addFrame(frame_rgb, KINECT_ID_1);
    //mViewer->addFrame(frame_ir, KINECT_ID_0);
    //mViewer->addFrame(frame_ir, KINECT_ID_1);

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

    // wait to get one frame of each camera...
    sleep(2);
}


int KinectApplication::getKinectIdx(const int streamIdx)
{
    //simple solution works for now...
    return streamIdx % 2;
}


void KinectApplication::startListeners()
{
    mFrameListeners[0]->startListening();
    mFrameListeners[1]->startListening();
    mFrameListeners[2]->startListening();
    mFrameListeners[3]->startListening();
    if (context->mDrawBody){
        mBodyListeners[KINECT_ID_0]->startListening();
        mBodyListeners[KINECT_ID_1]->startListening();
    }
}


void KinectApplication::stopListeners()
{
    mFrameListeners[0]->stopListening();
    mFrameListeners[1]->stopListening();
    mFrameListeners[2]->stopListening();
    mFrameListeners[3]->stopListening();
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
            int kinectIdx = getKinectIdx(i);
            openni::SensorType type = mStreamFrameTypes[i];
            nite::UserTracker *tracker = &mUserTrackers[kinectIdx];
            auto frame = std::make_shared<Frame>(type, mFrames[i].front(), tracker);
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
            mViewer->addFrame(frame, kinectIdx);
            mFrames[i].pop();
        }
    }
}


void KinectApplication::handleReplay()
{
    for (int i = 0; i < KINECT_COUNT; ++i){
        int kinectIdx = getKinectIdx(i);
        openni::SensorType type = mStreamFrameTypes[i];
        const cv::Scalar bgColor(0.f, 0.f, 50.f);
        auto frame = std::make_shared<Frame>(type, 640, 480, bgColor, &mUserTrackers[i]);
        const Body &body = mBodyManager->getNextBody(i, getTimeNow() - mReplayStartTime);
        frame->drawSkeleton(body);
        mViewer->addFrame(frame, kinectIdx);
    }
}


bool KinectApplication::handleCameraCalibration(const int streamIdx)
{
    static uint64_t lastTime[KINECT_STREAM_COUNT] = {getTimeNow(), getTimeNow(), getTimeNow(), getTimeNow()};
    static int index[KINECT_STREAM_COUNT] = {0, 0, 0, 0};
    const std::string streamName = context->mKinectViewerMap[streamIdx];
    const int kinectIdx = getKinectIdx(streamIdx);

    if (mCalibrator->readyCamera(streamIdx)) {
        mCalibrator->saveCamera(streamIdx);
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
        auto frame = mViewer->getFrame(mStreamFrameTypes[streamIdx], kinectIdx);
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
        std::shared_ptr<Frame> frame0 = mViewer->getFrame(openni::SensorType::SENSOR_COLOR, 0);
        std::shared_ptr<Frame> frame1 = mViewer->getFrame(openni::SensorType::SENSOR_COLOR, 1);

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
        if (!handleCameraCalibration(KINECT_COLOR_0)) {
            calibrated = false;
        }
        if (!handleCameraCalibration(KINECT_COLOR_1)) {
            calibrated = false;
        }
        if (!handleCameraCalibration(KINECT_DEPTH_0)) {
            calibrated = false;
        }
        if (!handleCameraCalibration(KINECT_DEPTH_1)) {
            calibrated = false;
        }
        if (calibrated) {
            context->mCamera = false;
        }
    }

    if (getTimeNow() - startTime >= 1000){
        //logger->setLevel(libfreenect2::Logger::None);
        std::shared_ptr<Frame> frame_rgb0 = mViewer->getFrame(openni::SensorType::SENSOR_COLOR, 0);
        std::shared_ptr<Frame> frame_ir0 = mViewer->getFrame(openni::SensorType::SENSOR_IR, 0);
        mCalibrator->addRGB2DEPTH(frame_rgb0, frame_ir0, 0);

        //std::shared_ptr<Frame> frame_rgb1 = mViewer->getFrame(openni::SensorType::SENSOR_COLOR, 1);
        //std::shared_ptr<Frame> frame_depth1 = mViewer->getFrame(openni::SensorType::SENSOR_DEPTH, 1);
        //mCalibrator->test(frame_rgb1, frame_depth1, 1);
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


