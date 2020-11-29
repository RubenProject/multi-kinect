#pragma once

#include <queue>
#include <OpenNI.h>
#include <NiTE.h>

class BodyListener : public nite::UserTracker::NewFrameListener
{
public:
    BodyListener(std::queue<nite::UserTrackerFrameRef> *frames, std::queue<uint64_t> *times, int fps);
    void onNewFrame(nite::UserTracker& ut);
private:
    std::queue<nite::UserTrackerFrameRef> *frames;
    std::queue<uint64_t> *times;

    int mFPS;
    uint64_t mLastTime;
};
