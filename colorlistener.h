#pragma once

#include <queue>
#include <OpenNI.h>

class ColorListener : public openni::VideoStream::NewFrameListener
{
public:
    ColorListener(std::queue<openni::VideoFrameRef> *frames, int fps);
    void onNewFrame(openni::VideoStream& vs);
    void startListening();
    void stopListening();
private:
    std::queue<openni::VideoFrameRef> *frames;

    int mFPS;
    uint64_t mLastTime;
    bool listening;
};
