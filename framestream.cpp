#include "framestream.h"




FrameStream::FrameStream(const openni::Device &dev, openni::SensorType type, int fps)
{
    mSensorType = type;
    mFrameListener = new FrameListener(&mFrames, fps);
    mStream.addNewFrameListener(mFrameListener);
}


FrameStream::~FrameStream()
{
}


void FrameStream::start()
{
    mFrameListener->startListening();
}


void FrameStream::stop()
{
    mFrameListener->stopListening();
}


openni::SensorType FrameStream::getSensorType()
{
    return mSensorType;
}
