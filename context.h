#pragma once

#include "pch.h"
#include "common.h"
#include "logger.h"


struct Context 
{
    Context() 
        :mDrawDepth(false), mDrawColor(true),  mDrawBody(true),
        mPlay(true), mLoad(false), 
        mCalibrate(false), mStartCalibrate(false), mStopCalibrate(false),
        mStartRecord(false), mStopRecord(false),
        mReplay(false), mStartReplay(false), mStopReplay(false),
        mConf(0.6f), mImmediate(true), mFPS(30),
        mRecordFolder("Rec/"), mShaderFolder("Res/"),
        mExit(false)
    {
        mLogger = new Logger("log.txt");
        if (mLogger->good()) {
            libfreenect2::setGlobalLogger(mLogger);
        }
        mKinectViewerMap[KINECT_COLOR_0] = "RGB0";
        mKinectViewerMap[KINECT_COLOR_1] = "RGB1";
        if (mDrawDepth) {
            mKinectViewerMap[KINECT_DEPTH_0] = "IR0";
            mKinectViewerMap[KINECT_DEPTH_1] = "IR1";
        } else if (mDrawBody) {
            mKinectViewerMap[KINECT_DEPTH_0] = "BODY0";
            mKinectViewerMap[KINECT_DEPTH_1] = "BODY1";
        }

    }
    //TODO add constructor with default arguments too

    void log(libfreenect2::Logger::Level level, const std::string &message)
    {
        mLogger->log(level, message);
    }

    bool mDrawDepth;
    bool mDrawColor;
    bool mDrawBody;

    bool mPlay;
    bool mLoad;

    bool mCalibrate;
    bool mStartCalibrate;
    bool mStopCalibrate;

    bool mStartRecord;
    bool mStopRecord;

    bool mReplay;
    bool mStartReplay;
    bool mStopReplay;


    float mConf;
    bool mImmediate;
    int mFPS;

    std::string mRecordFolder;
    std::string mShaderFolder;

    Logger *mLogger;
    std::map<int, std::string> mKinectViewerMap;

    bool mExit;
};
