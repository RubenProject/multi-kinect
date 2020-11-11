#ifndef DEPTHLISTENER_H
#define DEPTHLISTENER_H

#include <queue>
#include <OpenNI.h>

class DepthListener : public openni::VideoStream::NewFrameListener
{
public:
    DepthListener(std::queue<openni::VideoFrameRef> *frames);
    void onNewFrame(openni::VideoStream& vs);
private:
    std::queue<openni::VideoFrameRef> *frames;
};

#endif // DEPTHLISTENER_H
