#pragma once

#include "body.h"

#include <OpenNI.h>
#include <NiTE.h>
#include "glm/glm.hpp"

#include <vector>


class BodyManager
{
public:
    BodyManager();
    ~BodyManager();

    void initialize(const std::vector<nite::UserTracker> &tTrackers);

    void addBody(const nite::UserTrackerFrameRef &frame, const int tStreamIdx, uint64_t time);
    const Body &getBody(const int tStreamIdx);
    void setBody(const nite::UserTrackerFrameRef &frame, const int tStreamIdx);

    void startRecording();
    void stopRecording();

    void process();
    void save() const;
    void load();
private:
    glm::mat4x4 H;
    std::vector<nite::UserTracker> mUserTrackers;
    std::vector<std::vector<Body>> mBodyVectors;
    std::vector<Body> mLastBodies;
    std::vector<bool> mReadyStates;
};
