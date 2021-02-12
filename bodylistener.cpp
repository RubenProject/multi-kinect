#include "bodylistener.h"
#include "common.h"


BodyListener::BodyListener(std::queue<nite::UserTrackerFrameRef> *frames, std::queue<uint64_t> *times, int fps)
{
    this->frames = frames;
    this->times = times;
    this->mFPS = fps;
    this->mLastTime = 0.0;
    this->listening = true;
}


void BodyListener::startListening()
{
    this->listening = true;
}


void BodyListener::stopListening()
{
    this->listening = false;
}


void BodyListener::onNewFrame(nite::UserTracker& ut)
{
    nite::UserTrackerFrameRef frame;
    uint64_t time = getTimeNow();

    if (listening && (mFPS == -1 || time - mLastTime > 1000.0 / mFPS)) {
        ut.readFrame(&frame);
        frames->push(frame);
        times->push(time);
        mLastTime = time;
    }
}
