#ifndef COLORLISTENER_H
#define COLORLISTENER_H

#include <queue>
#include <OpenNI.h>

class ColorListener : public openni::VideoStream::NewFrameListener
{
public:
    ColorListener(std::queue<openni::VideoFrameRef> *frames);
    void onNewFrame(openni::VideoStream& vs);
private:
    std::queue<openni::VideoFrameRef> *frames;
    bool isUpdate;
};

#endif // COLORLISTENER_H
