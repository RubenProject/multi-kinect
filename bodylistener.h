#pragma once

#include <queue>
#include <OpenNI.h>
#include <NiTE.h>

class BodyListener : public nite::UserTracker::NewFrameListener
{
public:
    BodyListener(std::queue<nite::UserTrackerFrameRef> *frames, std::queue<uint64_t> *times);
    void onNewFrame(nite::UserTracker& ut);
private:
    std::queue<nite::UserTrackerFrameRef> *frames;
    std::queue<uint64_t> *times;
};
