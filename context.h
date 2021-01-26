#pragma once

#include "common.h"


struct Context 
{
    bool mDrawDepth;
    bool mDrawColor;
    bool mDrawBody;

    bool mPlay;
    bool mLoad;
    bool mStartRecord;
    bool mStopRecord;
    bool mReplay;
    bool mStartReplay;
    bool mStopReplay;
    bool mExit;

    float mConf;
    int mFPS;

    std::string mRecordFolder;
    std::string mShaderFolder;
};
