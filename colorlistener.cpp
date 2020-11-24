#include "colorlistener.h"
#include <chrono>

inline uint64_t getTimeNow()
{
    using namespace std::chrono;
    milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    return ms.count();
}

ColorListener::ColorListener(std::queue<openni::VideoFrameRef> *frames, std::queue<uint64_t> *times)
{
    this->frames = frames;
    this->times = times;
}

void ColorListener::onNewFrame(openni::VideoStream& vs)
{
    openni::VideoFrameRef frame;
    vs.readFrame(&frame);
    frames->push(frame);
    uint64_t time = getTimeNow();
    times->push(time);
}
