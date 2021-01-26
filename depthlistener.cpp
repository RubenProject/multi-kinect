#include "depthlistener.h"
#include "common.h"


DepthListener::DepthListener(std::queue<openni::VideoFrameRef> *frames, int fps)
{
    this->frames = frames;
    this->mFPS = fps;
    this->mLastTime = 0.0;
    this->listening = true;
}


void DepthListener::startListening()
{
    this->listening = true;
}


void DepthListener::stopListening()
{
    this->listening = false;
}


void DepthListener::onNewFrame(openni::VideoStream& vs)
{
    openni::VideoFrameRef frame;
    uint64_t time = getTimeNow();

    if (listening && (mFPS == -1 || time - mLastTime > 1000.0 / mFPS)) {
        vs.readFrame(&frame);
        frames->push(frame);
        mLastTime = time;
    }
}
