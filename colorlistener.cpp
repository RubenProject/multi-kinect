#include "colorlistener.h"
#include <iostream>

ColorListener::ColorListener(std::queue<openni::VideoFrameRef> *frames)
{
    this->frames = frames;
}

void ColorListener::onNewFrame(openni::VideoStream& vs)
{
    openni::VideoFrameRef frame;
    vs.readFrame(&frame);
    frames->push(frame);
}
