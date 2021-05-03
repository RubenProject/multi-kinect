#include "bodymanager.h"
#include "logger.h"


//TODO add a calibration phase where body streams are send to a calibration function
//TODO add a quality meassure to the recordings 
//- howmuch downtime is there for the bodystreams
//- how good is the skeleton confidence

BodyManager::BodyManager()
    : mCalibrated(true), mRecording(false)
{
    for (auto &state : mReadyStates){
        state = false;
    }
}


BodyManager::~BodyManager() 
{}


void BodyManager::update(const nite::UserTrackerFrameRef &frame, const int tStreamIdx, uint64_t time)
{
    mReadyStates[tStreamIdx] = false;
    const nite::Array<nite::UserData> &tUsers = frame.getUsers();
    for (int i = 0; i < tUsers.getSize(); ++i){
        const nite::UserData &tUser = tUsers[i];
        if (tUser.isNew()){
            mUserTrackers[tStreamIdx].startSkeletonTracking(tUser.getId());
        }
        if (tUser.isLost()){
            continue;
        }
        const nite::Skeleton &tSkeleton = tUser.getSkeleton();

        if (tUser.isVisible()){
            Body body(tSkeleton, frame.getFloor(), time);
            if (body.getConfidence() <= context->mConf)
                continue;
            mLastBodies[tStreamIdx] = body;
            mReadyStates[tStreamIdx] = true;

            if (mRecording || context->mHomography) {
                mBodyVectors[tStreamIdx].push_back(body);
            } 
            break;;
        }
    }

    if (context->mHomography && calibrateReady()){
        logger->log(libfreenect2::Logger::Info, "Calibrating...");
        calibrate();
        clearBodies();
    }
}


void BodyManager::setUserTrackers(const std::array<nite::UserTracker, KINECT_COUNT> &tTrackers)
{
    mUserTrackers = tTrackers;
}


void BodyManager::setRecordManager(std::shared_ptr<RecordManager> rm)
{
    this->mRecordManager = rm;
}


bool BodyManager::getReadyState(const int &tStreamIdx) const
{
    return mReadyStates[tStreamIdx];
}


const Body &BodyManager::getBody(const int &tStreamIdx)
{
    return mLastBodies[tStreamIdx];
}


const Body &BodyManager::getNextBody(const int &tStreamIdx, const uint64_t time)
{
    if (mReplayIdx[tStreamIdx] < mBodyVectors[tStreamIdx].size()
        && time > mBodyVectors[tStreamIdx][mReplayIdx[tStreamIdx]].getTimeStamp()) {
        return mBodyVectors[tStreamIdx][mReplayIdx[tStreamIdx]++];
    }
    if (mReplayIdx[tStreamIdx] == mBodyVectors[tStreamIdx].size()) {
        context->mReplay = false;
        context->mStopReplay = true;
    }
    return mBodyVectors[tStreamIdx][mReplayIdx[tStreamIdx]];
}


void BodyManager::getHomography(glm::dmat3x3 &R, glm::dvec3 &t) const
{
    R = this->R;
    t = this->t;
}


void BodyManager::setHomography(const glm::dmat3x3 &R, const glm::dvec3 &t)
{
    this->R = R;
    this->t = t;
}


bool BodyManager::calibrateReady()
{
    // at least 10 seconds of footage on both camera's
    // 500 body frames
    // max 0.5 seconds of downtime
    // XXX debug lowered numbers
    const uint64_t minRunTime = 5000;
    const size_t minSamples = 50;
    const uint64_t maxDownTime = 500;

    for (size_t i = 0; i < mBodyVectors.size(); ++i){
        if (mBodyVectors[i].empty()) {
            return false;
        }
        uint64_t begin = mBodyVectors[i].front().getTimeStamp();
        uint64_t end = mBodyVectors[i].back().getTimeStamp();
        if (end - begin < minRunTime) {
            return false;
        }
        if (mBodyVectors[i].size() <= minSamples) {
            return false;
        }
    }
    return true;
}


void BodyManager::calibrate()
{
    const uint64_t maxTimeDelta = 20;
    const uint64_t minSampleInterval = 500;
    const float minConfidence = 0.65;
    const int minSampleCount = 10;

    //calculate a set of best pairs
    // - Could include low velocity of joints
    std::vector<std::pair<int, int>> candidates;
    uint64_t nextSampleTime = 0;
    for (size_t i = 0; i < mBodyVectors[KINECT_ID_0].size(); ++i){
        if (mBodyVectors[KINECT_ID_0][i].getTimeStamp() < nextSampleTime){
            continue;
        }
        if (mBodyVectors[KINECT_ID_0][i].getConfidence() < minConfidence) {
            continue;
        }

        uint64_t minTimeDelta = 1000;
        size_t minTimeIdx = 0;
        const uint64_t time0 = mBodyVectors[KINECT_ID_0][i].getTimeStamp();
        for (size_t j = 0; j < mBodyVectors[KINECT_ID_1].size(); ++j){
            const uint64_t time1 = mBodyVectors[KINECT_ID_1][j].getTimeStamp();
            if (mBodyVectors[KINECT_ID_1][j].getConfidence() > minConfidence
                && time0 > time1 && time0 - time1 <= minTimeDelta){
                minTimeDelta = time0 - time1;
                minTimeIdx = j;
            }
            if (mBodyVectors[KINECT_ID_1][j].getConfidence() > minConfidence
                && time0 <= time1 && time1 - time0 <= minTimeDelta){
                minTimeDelta = time1 - time0;
                minTimeIdx= j;
            }
        }

        if (minTimeDelta < maxTimeDelta){
            candidates.push_back(std::make_pair<int, int>(i, minTimeIdx));
            nextSampleTime += minSampleInterval;
        }
    }

    if (candidates.size() < minSampleCount){
        logger->log(libfreenect2::Logger::Debug, "Not enough samples for calibration...");
        return;
    }

    //TODO
    // calculate the homography using the pairs of bodies
    // check what are the most reliable joints to use for triangulation

    std::vector<nite::JointType> stableJoints({nite::JOINT_LEFT_SHOULDER,
                                               nite::JOINT_RIGHT_SHOULDER,
                                               nite::JOINT_TORSO,
                                               nite::JOINT_HEAD});
    std::vector<cv::Point3f> worldCoords0;
    std::vector<cv::Point3f> worldCoords1;
    
    for (size_t i = 0; i < candidates.size(); ++i) {
        Body body0 = mBodyVectors[KINECT_ID_0][candidates[i].first];
        Body body1 = mBodyVectors[KINECT_ID_1][candidates[i].second];
        for (auto jointIdx : stableJoints){
            glm::dvec3 pos0 = body0.getJointAbsPosition(jointIdx);
            glm::dvec3 pos1 = body1.getJointAbsPosition(jointIdx);
            worldCoords0.push_back(cv::Point3d(pos0.x, pos0.y, pos0.z));
            worldCoords1.push_back(cv::Point3d(pos1.x, pos1.y, pos1.z));
        }
    }

    cv::Mat inliers;
    cv::Mat h;

    int success = cv::estimateAffine3D(worldCoords0, worldCoords1, h, inliers);

    if (success) {
        std::cout << h << std::endl;
        R = glm::dmat3(h.at<double>(0, 0), h.at<double>(0, 1), h.at<double>(0, 2),
                       h.at<double>(1, 0), h.at<double>(1, 1), h.at<double>(1, 2),
                       h.at<double>(2, 0), h.at<double>(2, 1), h.at<double>(2, 2));
        t = glm::dvec3(h.at<double>(0, 3), h.at<double>(1, 3), h.at<double>(2, 3));
        mCalibrated = true;
    } else {
        logger->log(libfreenect2::Logger::Debug, "Calibration failed");
        mCalibrated = false;
    }
}


void BodyManager::stopCalibrate()
{
}


void BodyManager::process()
{
    for (size_t i = 0; i < mBodyVectors[KINECT_ID_0].size(); ++i){
        Body b = mBodyVectors[KINECT_ID_0][i];
        b.transform(R, t);
        mCombinedBodyVector.push_back(b);
    }

    //std::swap(mBodyVectors[KINECT_ID_0], mBodyVectors[KINECT_ID_1]);
    mBodyVectors[KINECT_ID_0] = mCombinedBodyVector;

    //const int testIdx = 100;
    //uint64_t time0 = mCombinedBodyVector[testIdx].getTimeStamp();
    //size_t pairIdx = 0;
    //uint64_t pairDelta = 500;
//
    //for (size_t i = 0; i < mBodyVectors[KINECT_ID_1].size(); ++i){
        //uint64_t time1 = mBodyVectors[KINECT_ID_1][i].getTimeStamp();
        //if (time0 > time1 && time0 - time1 <= pairDelta){
            //pairDelta = time0 - time1;
            //pairIdx = i;
        //}
        //if (time0 <= time1 && time1 - time0 <= pairDelta){
            //pairDelta = time1 - time0;
            //pairIdx = i;
        //}
    //}



    //std::cout << mCombinedBodyVector[testIdx] << std::endl;
    //std::cout << mBodyVectors[KINECT_ID_1] << std::endl;
    
}


void BodyManager::startRecording()
{
    if (mCalibrated){
        clearBodies();
        mRecording = true;
    } else {
        logger->log(libfreenect2::Logger::Debug, "No homography found!");
    }
}


void BodyManager::stopRecording()
{
    mRecording = false;

    // make all timestamps relative to the first frame
    uint64_t startTime = std::min(mBodyVectors[KINECT_ID_0].begin()->getTimeStamp(), 
                                  mBodyVectors[KINECT_ID_1].begin()->getTimeStamp());
    for (auto &v : mBodyVectors[KINECT_ID_0]){
        v.setTimeStamp(v.getTimeStamp() - startTime);
    }
    for (auto &v : mBodyVectors[KINECT_ID_1]){
        v.setTimeStamp(v.getTimeStamp() - startTime);
    }

    save();
    process();
    exportDataset();
}


void BodyManager::startReplay()
{
    for (auto &idx : mReplayIdx) {
        idx = 0;
    }
}


void BodyManager::save()
{
    std::array<YAML::Node, KINECT_COUNT> root;
    for (size_t i = 0; i < KINECT_COUNT; ++i){
        for (auto &body : mBodyVectors[i]){
            root[i].push_back(body.serialize());
        }
    }
    mRecordManager->writeRecording(root);
}


void BodyManager::load(int recordingIdx)
{
    Body body;
    
    clearBodies();
    
    std::array<YAML::Node, KINECT_COUNT> root;
    mRecordManager->readRecording(root, recordingIdx);
    for (size_t i = 0; i < KINECT_COUNT; ++i) {
        for (size_t j = 0; j < root[i].size(); ++j) {
            body.deserialize(root[i][j]);
            mBodyVectors[i].push_back(body);
        }
    }
}


void BodyManager::rotateBodies(const glm::dmat3 &R, const int tStreamIdx)
{
    for (auto &body : mBodyVectors[tStreamIdx])
        body.transform(R, glm::dvec3(0.0));
}


void BodyManager::scaleBodies(const glm::dvec3 &S, const int tStreamIdx)
{
    glm::dmat3 R = glm::dmat3(S.x, 0.0, 0.0,
                              0.0, S.y, 0.0,
                              0.0, 0.0, S.z);
    for (auto &body : mBodyVectors[tStreamIdx])
        body.transform(R, glm::dvec3(0.0));
}


void BodyManager::translateBodies(const glm::dvec3 &t, const int tStreamIdx)
{
    for (auto &body : mBodyVectors[tStreamIdx])
        body.transform(glm::dmat3(1.0), t);
}


void BodyManager::exportDataset() const
{
    //TODO
}


void BodyManager::clearBodies()
{
    for (auto v : mBodyVectors){
        v.clear();
    }
}


