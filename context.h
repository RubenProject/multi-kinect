#pragma once

#include "pch.h"
#include "common.h"


struct Context 
{
    Context() 
        :mDrawDepth(false), mDrawColor(true),  mDrawBody(false), mDrawIR(true),
        mPlay(true), mLoad(false), mCamera(false), mHomography(false),
        mStartRecord(false), mStopRecord(false),
        mReplay(false), mStartReplay(false), mStopReplay(false),
        mConf(0.6f), mImmediate(true), mFPS(30),
        mRecordFolder("Rec/"), mShaderFolder("Res/"),
        mExit(false)
    {}

    bool mDrawDepth;
    bool mDrawColor;
    bool mDrawBody;
    bool mDrawIR;

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


