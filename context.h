#pragma once

#include "pch.h"
#include "common.h"


struct Context 
{
    Context() 
        :mDrawDepth(false), mDrawColor(true),  mDrawBody(true),
        mPlay(true), mLoad(false), mCamera(false), mHomography(false),
        mStartRecord(false), mStopRecord(false),
        mReplay(false), mStartReplay(false), mStopReplay(false),
        mConf(0.6f), mImmediate(true), mFPS(30),
        mRecordFolder("Rec/"), mShaderFolder("Res/"),
        mExit(false)
    {
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

    bool mDrawDepth;
    bool mDrawColor;
    bool mDrawBody;

    bool mPlay;
    bool mLoad;

    bool mCamera;
    bool mHomography;
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

    std::map<int, std::string> mKinectViewerMap;

    bool mExit;
};

extern Context *context;


