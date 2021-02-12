#include "kinectapp.h"
#include "common.h"


KinectApplication::KinectApplication()
{
    mContext = std::make_shared<Context>();

    assert(!(mContext->mDrawDepth && mContext->mDrawBody));

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

    //std::cout << mDev[KINECT_ID_0].getImageRegistrationMode() << std::endl;
    //std::cout << mDev[KINECT_ID_1].getImageRegistrationMode() << std::endl;
}


KinectApplication::KinectApplication(std::shared_ptr<Context> context)
    :mContext(context)
{
    assert(!(mContext->mDrawDepth && mContext->mDrawBody));

    openni::OpenNI::initialize();
    openni::Array<openni::DeviceInfo> devInfoList;
    openni::OpenNI::enumerateDevices(&devInfoList);
    assert(devInfoList.getSize() == 6);
    mDev[KINECT_ID_0].open(devInfoList[0].getUri());
    mDev[KINECT_ID_1].open(devInfoList[3].getUri());
}


KinectApplication::~KinectApplication()
{
    if (mContext->mDrawBody){
        mUserTrackers[KINECT_ID_0].removeNewFrameListener(mBodyListeners[KINECT_ID_0]);
        mUserTrackers[KINECT_ID_1].removeNewFrameListener(mBodyListeners[KINECT_ID_1]);
        nite::NiTE::shutdown();
    }

    mStreams[KINECT_COLOR_0].removeNewFrameListener(mColorListeners[KINECT_ID_0]);
    mStreams[KINECT_COLOR_1].removeNewFrameListener(mColorListeners[KINECT_ID_1]);
    mStreams[KINECT_DEPTH_0].removeNewFrameListener(mDepthListeners[KINECT_ID_0]);
    mStreams[KINECT_DEPTH_1].removeNewFrameListener(mDepthListeners[KINECT_ID_1]);

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


    // Test setup
    //stopListeners();
    
    //mBodyManager->load(1);
    //mBodyManager->save();
    //assert(mBodyManager->calibrateReady());
    //mBodyManager->calibrate();
    //mBodyManager->process();
    //mContext->mStartReplay = true;
    mContext->mCalibrate = true;
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
    for (int i = 0; i < KINECT_STREAM_COUNT; ++i){
        std::queue<openni::VideoFrameRef> empty;
        std::swap(mFrames[i], empty);
    }
}


void KinectApplication::handleBodies()
{
    for (int i = 0; i < KINECT_COUNT; ++i){
        while (mContext->mImmediate && mBodyFrames[i].size() > 2){
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
        while (mContext->mImmediate && mFrames[i].size() > 2){
            mFrames[i].pop();
        }
        if (!mFrames[i].empty()) {
            std::string frameHeader = mContext->mKinectViewerMap[i];
            auto frame = std::make_unique<Frame>(mFrames[i].front(), frameHeader);
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
            mViewer->addFrame(frameHeader, std::move(frame));
            mFrames[i].pop();
        }
    }
}


void KinectApplication::handleReplay()
{
    for (int i = 0; i < KINECT_COUNT; ++i){
        std::string frameHeader = mContext->mKinectViewerMap[i];
        const cv::Scalar bgColor(0.f, 0.f, 50.f);
        auto frame = std::make_unique<Frame>(640, 480, bgColor);
        const Body &body = mBodyManager->getNextBody(i, getTimeNow() - mReplayStartTime);
        frame->drawSkeleton(mUserTrackers[i], body);
        mViewer->addFrame(frameHeader, std::move(frame));
    }
}


bool KinectApplication::update()
{
    static uint64_t startTime = getTimeNow();
    if (mContext->mStartCalibrate) {
        mContext->log(libfreenect2::Logger::Info, "Starting Calibration");
        mViewer->setWindowTitle("Calibrating");
        mContext->mCalibrate = true;
        mContext->mStartCalibrate = false;
    }

    if (mContext->mStopCalibrate) {
        mContext->log(libfreenect2::Logger::Info, "Stopping Calibration");
        mViewer->setWindowTitle("Playing");
        mContext->mCalibrate = false;
        mContext->mStopCalibrate = false;
        mBodyManager->stopCalibrate();
    }

    if (mContext->mStartRecord) {
        mContext->log(libfreenect2::Logger::Info, "Starting Recording");
        mViewer->setWindowTitle("Recording");
        mBodyManager->startRecording();
        mContext->mStartRecord = false;
        mContext->mImmediate = false;
    }

    if (mContext->mStopRecord) {
        mContext->log(libfreenect2::Logger::Info, "Stopping Recording");
        mViewer->setWindowTitle("Playing");
        mBodyManager->stopRecording();
        mContext->mStopRecord = false;
        mContext->mImmediate = true;
    }

    if (mContext->mStartReplay) {
        mContext->log(libfreenect2::Logger::Info, "Replaying");
        mContext->mReplay = true;
        mContext->mPlay = false;
        mBodyManager->startReplay();
        stopListeners();
        mReplayStartTime = getTimeNow();
        mContext->mStartReplay = false;
    }

    if (mContext->mStopReplay) {
        mContext->log(libfreenect2::Logger::Info, "Live Feed");
        mContext->mReplay = false;
        mContext->mPlay = true;
        startListeners();
        mContext->mStopReplay = false;
    }

    if (mContext->mLoad) {
        int recordingIdx;
        mContext->log(libfreenect2::Logger::Info, "Enter Recording ID to be loaded:");
        std::cin >> recordingIdx;
        mContext->log(libfreenect2::Logger::Info, "Loading: " + std::to_string(recordingIdx));
        mBodyManager->load(recordingIdx);
        mContext->mLoad = false;;
    }

    if (mContext->mDrawBody) {
        handleBodies();
    }

    if (mContext->mPlay) {
        handlePlay();
    } 

    if (mContext->mReplay) {
        handleReplay();
    }

    if (mContext->mCalibrate) {
        //while (mFrames[KINECT_COLOR_0].empty()) {}
        //while (mFrames[KINECT_COLOR_1].empty()) {}
        //auto frame0 = std::make_unique<Frame>(mFrames[KINECT_COLOR_0].back(), "RGB0");
        //auto frame1 = std::make_unique<Frame>(mFrames[KINECT_COLOR_1].back(), "RGB1");
        ChessCalib2d calibrator(mContext);
        //calibrator.saveSample(frame0->getMat(), frame1->getMat(), "Samples/Kinect0.png", "Samples/Kinect1.png");
        //calibrator.addSample(frame0->getMat(), frame1->getMat());
        calibrator.loadSample("Samples/Kinect0.png", "Samples/Kinect1.png");
        if (calibrator.calibrateReady()) {
            stopListeners();
            calibrator.calibrate();
            startListeners();
        }
    }

    return mContext->mExit || mViewer->render();
}


