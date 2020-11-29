#include "bodylistener.h"
#include <chrono>

inline uint64_t getTimeNow()
{
    using namespace std::chrono;
    milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    return ms.count();
}

BodyListener::BodyListener(std::queue<nite::UserTrackerFrameRef> *frames, std::queue<uint64_t> *times, int fps)
{
    this->frames = frames;
    this->times = times;
    this->mFPS = fps;
    this->mLastTime = 0.0;
}

void BodyListener::onNewFrame(nite::UserTracker& ut)
{
    nite::UserTrackerFrameRef frame;
    uint64_t time = getTimeNow();

    if (mFPS == -1 || time - mLastTime > 1000.0 / mFPS) {
        ut.readFrame(&frame);
        frames->push(frame);
        times->push(time);
        mLastTime = time;
    }
}
