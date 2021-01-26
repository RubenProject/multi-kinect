#pragma once

#include "common.h"
#include "recordmanager.h"
#include "body.h"


class BodyManager
{
public:
    BodyManager(std::shared_ptr<Context> context);
    ~BodyManager();

    void setUserTrackers(const std::array<nite::UserTracker, KINECT_COUNT> &tTrackers);
    void setRecordManager(std::shared_ptr<RecordManager> rm);

    bool getReadyState(const int tStreamIdx) const;

    void addBody(const nite::UserTrackerFrameRef &frame, const int tStreamIdx, uint64_t time);
    const Body &getBody(const int tStreamIdx);
    const Body &getNextBody(const int tStreamIdx, const uint64_t tTime);
    void setBody(const nite::UserTrackerFrameRef &frame, const int tStreamIdx);

    void startRecording();
    void stopRecording();
    void startReplay();

    void save();
    void load(const int recordingIdx);
    void exportDataset() const;
private:
    void process();
    bool calibrateReady();
    void calibrate();
    void clearBodies();

    std::shared_ptr<RecordManager> mRecordManager;
    std::array<nite::UserTracker, KINECT_COUNT> mUserTrackers;

    std::array<std::vector<Body>, KINECT_COUNT> mBodyVectors;
    std::vector<Body> mCombinedBodyVector;
    glm::mat4x4 H;

    std::array<Body, KINECT_COUNT> mLastBodies;
    std::array<bool, KINECT_COUNT> mReadyStates;
    std::array<size_t, KINECT_COUNT> mReplayIdx;

    bool mCalibrated;
    bool mRecording;
    uint64_t mRecStartTime;
    std::shared_ptr<Context> mContext;
};
