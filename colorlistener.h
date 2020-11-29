#pragma once

#include <queue>
#include <OpenNI.h>

class ColorListener : public openni::VideoStream::NewFrameListener
{
public:
    ColorListener(std::queue<openni::VideoFrameRef> *frames, std::queue<uint64_t> *times, int fps);
    void onNewFrame(openni::VideoStream& vs);
private:
    std::queue<openni::VideoFrameRef> *frames;
    std::queue<uint64_t> *times;

    int mFPS;
    uint64_t mLastTime;
};
