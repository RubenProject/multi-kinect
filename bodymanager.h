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

    void initialize(const nite::UserTracker &tracker_0, const nite::UserTracker &tracker_1);
    Body *addBody(const nite::UserTrackerFrameRef &frame, const int tStreamIdx, uint64_t time);
    Body *getBody(const nite::UserTrackerFrameRef &frame, const int tStreamIdx);
    bool isReady(const int tStreamIdx) const;

    void startRecording();
    void stopRecording();

    void process();
    void save() const;
    void load();
private:
    glm::mat4x4 H;
    nite::UserTracker mUserTracker_0, mUserTracker_1;
    std::vector<Body> mBodyVector_0, mBodyVector_1;

    bool mBodyReadyState_0, mBodyReadyState_1;
};
