#include "depthlistener.h"

DepthListener::DepthListener(std::queue<openni::VideoFrameRef> *frames)
{
    this->frames = frames;
}

void DepthListener::onNewFrame(openni::VideoStream& vs)
{
    openni::VideoFrameRef frame;
    vs.readFrame(&frame);
    frames->push(frame);
}
