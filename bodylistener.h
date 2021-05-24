#pragma once

#include "pch.h"


class BodyListener : public nite::UserTracker::NewFrameListener
{
public:
    BodyListener(std::queue<nite::UserTrackerFrameRef> *frames, std::queue<uint64_t> *times, int fps);
    void onNewFrame(nite::UserTracker& ut);
    void startListening();
    void stopListening();
private:
    std::queue<nite::UserTrackerFrameRef> *frames;
    std::queue<uint64_t> *times;

    int mFPS;
    uint64_t mLastTime;
    bool listening;
};
