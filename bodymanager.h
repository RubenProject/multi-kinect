#pragma once

#include "pch.h"

#include "common.h"
#include "recordmanager.h"
#include "body.h"
#include "context.h"


class BodyManager
{
public:
    BodyManager();
    ~BodyManager();

    void setUserTrackers(const std::array<nite::UserTracker, KINECT_COUNT> &tTrackers);
    void setRecordManager(std::shared_ptr<RecordManager> rm);

    bool getReadyState(const int &tStreamIdx) const;

    void update(const nite::UserTrackerFrameRef &frame, const int tStreamIdx, uint64_t time);

    const Body &getBody(const int &tStreamIdx);
    const Body &getNextBody(const int &tStreamIdx, const uint64_t tTime);
    void setBody(const nite::UserTrackerFrameRef &frame, const int tStreamIdx);

    void getHomography(glm::dmat3x3 &R, glm::dvec3 &t) const;
    void setHomography(const glm::dmat3x3 &R, const glm::dvec3 &t);

    void rotateBodies(const glm::dmat3 &R, const int tStreamIdx);
    void scaleBodies(const glm::dvec3 &S, const int tStreamIdx);
    void translateBodies(const glm::dvec3 &t, const int tStreamIdx);

    void startRecording();
    void stopRecording();
    void startReplay();
    void stopCalibrate();

    void save();
    void load(const int recordingIdx);
    void exportDataset() const;

    bool calibrateReady();
    void calibrate();
    void process();
private:
    void clearBodies();

    std::shared_ptr<RecordManager> mRecordManager;
    std::array<nite::UserTracker, KINECT_COUNT> mUserTrackers;
    std::array<std::vector<Body>, KINECT_COUNT> mBodyVectors;

    // affine transformation between two camera's
    glm::dmat3x3 R;
    glm::dvec3 t;
    std::vector<Body> mCombinedBodyVector;

    std::array<Body, KINECT_COUNT> mLastBodies;
    std::array<bool, KINECT_COUNT> mReadyStates;
    std::array<size_t, KINECT_COUNT> mReplayIdx;

    bool mCalibrated;
    bool mRecording;
};
