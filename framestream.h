#pragma once


#include "pch.h"

#include "framelistener.h"
#include "frame.h"


class FrameStream
{
public:
    FrameStream(const openni::Device &dev, openni::SensorType type, int fps);
    ~FrameStream();

    void start();
    void stop();

    openni::SensorType getSensorType();
private:
    FrameListener *mFrameListener;
    std::queue<openni::VideoFrameRef> mFrames;
    openni::SensorType mSensorType;
};
