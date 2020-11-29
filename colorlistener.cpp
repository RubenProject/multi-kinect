#include "colorlistener.h"
#include <chrono>

inline uint64_t getTimeNow()
{
    using namespace std::chrono;
    milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    return ms.count();
}

ColorListener::ColorListener(std::queue<openni::VideoFrameRef> *frames, std::queue<uint64_t> *times, int fps)
{
    this->frames = frames;
    this->times = times;
    this->mFPS = fps;
    this->mLastTime = 0.0;
}

void ColorListener::onNewFrame(openni::VideoStream& vs)
{
    openni::VideoFrameRef frame;
    uint64_t time = getTimeNow();

    if (mFPS == -1 || time - mLastTime > 1000.0 / mFPS){
        vs.readFrame(&frame);
        frames->push(frame);
        times->push(time);
        mLastTime = time;
    }
}
