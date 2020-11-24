#include "bodylistener.h"
#include <chrono>

inline uint64_t getTimeNow()
{
    using namespace std::chrono;
    milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    return ms.count();
}

BodyListener::BodyListener(std::queue<nite::UserTrackerFrameRef> *frames, std::queue<uint64_t> *times)
{
    this->frames = frames;
    this->times = times;
}

void BodyListener::onNewFrame(nite::UserTracker& ut)
{
    nite::UserTrackerFrameRef frame;
    ut.readFrame(&frame);
    frames->push(frame);
    uint64_t time = getTimeNow();
    times->push(time);
}
