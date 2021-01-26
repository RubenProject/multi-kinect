#include "bodymanager.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>


//TODO add a calibration phase where body streams are send to a calibration function
//TODO add a quality meassure to the recordings 
//- howmuch downtime is there for the bodystreams
//- how good is the skeleton confidence

BodyManager::BodyManager(std::shared_ptr<Context> context) 
    : mCalibrated(false), mRecording(false), mRecStartTime(0), mContext(context)
{
    for (auto &state : mReadyStates){
        state = false;
    }
}


BodyManager::~BodyManager() 
{}


void BodyManager::setUserTrackers(const std::array<nite::UserTracker, KINECT_COUNT> &tTrackers)
{
    mUserTrackers = tTrackers;
}


void BodyManager::setRecordManager(std::shared_ptr<RecordManager> rm)
{
    this->mRecordManager = rm;
}


void BodyManager::addBody(const nite::UserTrackerFrameRef &frame, const int tStreamIdx, uint64_t time)
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
            if (body.getConfidence() <= mContext->mConf)
                continue;
            mLastBodies[tStreamIdx] = body;
            mReadyStates[tStreamIdx] = true;

            if (!mCalibrated) {
                mBodyVectors[tStreamIdx].push_back(body);
                if (calibrateReady()){
                    std::cout << "calibrating..." << std::endl;
                    calibrate();
                    clearBodies();
                }
            } else if (mRecording) {
                mBodyVectors[tStreamIdx].push_back(body);
            } 
            break;;
        }
    }
}


bool BodyManager::getReadyState(const int tStreamID) const
{
    return mReadyStates[tStreamID];
}


const Body &BodyManager::getBody(const int tStreamID)
{
    return mLastBodies[tStreamID];
}


const Body &BodyManager::getNextBody(const int tStreamIdx, const uint64_t time)
{
    if (mReplayIdx[tStreamIdx] < mBodyVectors[tStreamIdx].size()
        && time > mBodyVectors[tStreamIdx][mReplayIdx[tStreamIdx]].getTimeStamp()) {
        return mBodyVectors[tStreamIdx][mReplayIdx[tStreamIdx]++];
    }
    if (mReplayIdx[tStreamIdx] == mBodyVectors[tStreamIdx].size()) {
        mContext->mReplay = false;
        mContext->mStopReplay = true;
    }
    return mBodyVectors[tStreamIdx][mReplayIdx[tStreamIdx]];
}


bool BodyManager::calibrateReady()
{
    // at least 10 seconds of footage on both camera's
    // 500 body frames
    // max 0.5 seconds of downtime
    // XXX debug lowered numbers
    const uint64_t minRunTime = 1000;
    const size_t minSamples = 50;
    const uint64_t maxDownTime = 500;

    for (size_t i = 0; i < mBodyVectors.size(); ++i){
        if (mBodyVectors[i].empty()) {
            return false;
        }
        uint64_t begin = mBodyVectors[i].begin()->getTimeStamp();
        uint64_t end = mBodyVectors[i].end()->getTimeStamp();
        if (end - begin < minRunTime) {
            return false;
        }
        if (mBodyVectors[i].size() <= minSamples) {
            return false;
        }
        /*uint64_t last = 0;
        uint64_t now = 0;
        for (auto v : mBodyVectors[i]){
            now = v.getTimeStamp();
            if (last != 0 && now - last > maxDownTime) {
                std::cout << "downtime" << std::endl;
                return false;
            }
            last = now;
        }
        */
    }
    return true;
}


void BodyManager::calibrate()
{
    const uint64_t maxTimeDelta = 20;
    const uint64_t minSampleInterval = 500;
    const float minConfidence = 0.65;
    const int minSampleCount = 10;
    H = glm::mat4x4(1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0);

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
        std::cout << "Not enough samples for calibration..." << std::endl;
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
            glm::vec3 pos0 = body0.getJointAbsPosition(jointIdx);
            glm::vec3 pos1 = body1.getJointAbsPosition(jointIdx);
            worldCoords0.push_back(cv::Point3f(pos0.x, pos0.y, pos0.z));
            worldCoords1.push_back(cv::Point3f(pos1.x, pos1.y, pos1.z));
        }
    }

    cv::Mat inliers;
    cv::Mat h;

    int ret = cv::estimateAffine3D(worldCoords0, worldCoords1, h, inliers);
    cv::Mat r = h(cv::Rect(0, 0, 3, 3)).clone();
    cv::Mat t = h(cv::Rect(3, 0, 1, 3)).clone();
    if (ret) {
        mCalibrated = true;
    } else {
        std::cout << "calibration failed" << std::endl;
    }
}


void BodyManager::process()
{
    /*const uint64_t max_past = 1000, max_future = 1000;
    const int past_count = 6, future_count = 6;

    for (int i = 0; i < mBodyVectors[0].size(); i++){
        std::vector<Body*> past;
        std::vector<Body*> past_cand;
        std::vector<Body*> future;
        std::vector<Body*> future_cand;
        for (int j = i-1; j >= 0; --j){
            if (mBodyVectors[0][i].getTimeStamp() - mBodyVectors[0][j].getTimeStamp() < max_past)
                past_cand.push_back(&mBodyVectors[j]);
            else {
                break;
            }
        }




        for (int j = i+1; j < mBodyVectors[0].size(); ++j){
            if (mBodyVectors[0][j].getTimeStamp() - mBodyVectors[0][i].getTimestamp() < max_future)
                future_cand.push_back(&mBodyVectors[j]);
            else {
                break;
            }
        }




        for (int j = 0; j < past.size(); ++j)
            mBodyVectors[0][i].setTrajectory(&past[past_count - j], j);
        }

        mBodyVectors[0][i].setTrajectory(&mBodyVectors[0][i], past_count + 1);

        for (int j = 0; j < future.size(); ++j)
            mBodyVectors[0][i].setTrajectory(&future[future_count - j], past_count + 1 + j);
        }

    }

    H = glm::mat4x4(1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0);

    for (auto body : mBodyVectors[0]){
        auto joint0 = body.getJointRelPosition(nite::JOINT_HEAD);
        std::cout << joint0.x << ", " << joint0.y << ", " << joint0.z << std::endl;
        body.convertCoordinates(H);
        auto joint1 = body.getJointRelPosition(nite::JOINT_HEAD);
        std::cout << joint1.x << ", " << joint1.y << ", " << joint1.z << std::endl;
        exit(0x0);
    }

    */

}


void BodyManager::startRecording()
{
    if (mCalibrated){
        clearBodies();
        mRecording = true;
    } else {
        std::cout << "Not done calibrating!" << std::endl;
    }
}


void BodyManager::stopRecording()
{
    mRecording = false;

    // make all times relative to the first frame
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
    std::vector<std::string> bodyString(KINECT_COUNT, "");
    for (size_t i = 0; i < mBodyVectors.size(); i++){
        for (auto body : mBodyVectors[i]){
            bodyString[i] += body.toString() + "\n";
        }
    }
    mRecordManager->writeRecording(bodyString);
}


void BodyManager::load(int recordingIdx)
{
    Body body;
    std::string line;
    
    this->clearBodies();
    
    std::vector<std::string> bodyString;
    mRecordManager->readRecording(bodyString, recordingIdx);
    for (int i = 0; i < KINECT_COUNT; ++i) {
        std::stringstream ss;
        ss.str(bodyString[i]);
        while (std::getline(ss, line)){
            body.fromString(line);
            mBodyVectors[i].push_back(body);
        }
    }
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


