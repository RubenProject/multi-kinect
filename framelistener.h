#pragma once

#include "pch.h"


class FrameListener : public openni::VideoStream::NewFrameListener
{
public:
    FrameListener(std::queue<openni::VideoFrameRef> *frames, int fps);
    void onNewFrame(openni::VideoStream& vs);
    void startListening();
    void stopListening();
private:
    std::queue<openni::VideoFrameRef> *frames;

    int mFPS;
    uint64_t mLastTime;
    bool listening;
};
