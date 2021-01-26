#include "colorlistener.h"
#include "common.h"


ColorListener::ColorListener(std::queue<openni::VideoFrameRef> *frames, int fps)
{
    this->frames = frames;
    this->mFPS = fps;
    this->mLastTime = 0.0;
    this->listening = true;
}


void ColorListener::startListening()
{
    this->listening = true;
}


void ColorListener::stopListening()
{
    this->listening = false;
}


void ColorListener::onNewFrame(openni::VideoStream& vs)
{
    openni::VideoFrameRef frame;
    uint64_t time = getTimeNow();

    if (listening && (mFPS == -1 || time - mLastTime > 1000.0 / mFPS)) {
        vs.readFrame(&frame);
        frames->push(frame);
        mLastTime = time;
    }
}
