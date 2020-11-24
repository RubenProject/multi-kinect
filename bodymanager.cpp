#include "bodymanager.h"

#include <iostream>


BodyManager::BodyManager()
    : mBodyReadyState_0(false), mBodyReadyState_1(false)
{
}


BodyManager::~BodyManager()
{
}


void BodyManager::initialize(const nite::UserTracker &tracker_0, const nite::UserTracker &tracker_1)
{
    mUserTracker_0 = tracker_0;
    mUserTracker_1 = tracker_1;
}


Body *BodyManager::addBody(const nite::UserTrackerFrameRef &frame, const int tStreamID, uint64_t time)
{
    Body *body = new Body();
    const nite::Array<nite::UserData> &tUsers = frame.getUsers();
    for (int i = 0; i < tUsers.getSize(); ++i){
        const nite::UserData &tUser = tUsers[i];
        if (tUser.isNew()){
            if (tStreamID == 0) {
                mUserTracker_0.startSkeletonTracking(tUser.getId());
            } else if (tStreamID == 1){
                mUserTracker_1.startSkeletonTracking(tUser.getId());
            } else {
                std::cout << "Invalid stream!" << std::endl;
                exit(0x0);
            }
        }
        const nite::Skeleton &tSkeleton = tUser.getSkeleton();
        body->initialize(tSkeleton, time);
    }

    if (tStreamID == 0){
        mBodyReadyState_0 = true;
        mBodyVector_0.push_back(*body);
    } else if (tStreamID == 1) {
        mBodyReadyState_1 = true;
        mBodyVector_1.push_back(*body);
    } else {
        std::cout << "Invalid stream!" << std::endl;
        exit(0x0);
    }
    return body;
}


Body *BodyManager::getBody(const nite::UserTrackerFrameRef &frame, const int tStreamID)
{
    Body *body;
    const nite::Array<nite::UserData> &tUsers = frame.getUsers();
    for (int i = 0; i < tUsers.getSize(); ++i){
        const nite::UserData &tUser = tUsers[i];
        if (tUser.isNew()){
            if (tStreamID == 0) {
                mUserTracker_0.startSkeletonTracking(tUser.getId());
            } else if (tStreamID == 1){
                mUserTracker_1.startSkeletonTracking(tUser.getId());
            } else {
                std::cout << "Invalid stream!" << std::endl;
                exit(0x0);
            }
        }

        const nite::Skeleton &tSkeleton = tUser.getSkeleton();
        if (tSkeleton.getState() == nite::SKELETON_TRACKED){
            body = new Body();
            body->initialize(tSkeleton, -1);
            if (tStreamID == 0)
                mBodyReadyState_0 = true;
            if (tStreamID == 1)
                mBodyReadyState_1 = true;
            return body;
        }
    } 
    if (tStreamID == 0)
        mBodyReadyState_0 = false;
    if (tStreamID == 1)
        mBodyReadyState_1 = false;
    return NULL;
}


bool BodyManager::isReady(const int tStreamID) const
{
    if (tStreamID == 0) {
        return mBodyReadyState_0;
    } else if (tStreamID == 1){
        return mBodyReadyState_1;
    } else {
        std::cout << "Invalid stream!" << std::endl;
        exit(0x0);
        return false;
    }
}


void BodyManager::process()
{
    for (size_t i = 0; i < mBodyVector_0.size(); i++){
        mBodyVector_0[i];
    }

    for (size_t i = 0; i < mBodyVector_1.size(); i++){
        mBodyVector_1[i];
    }

    H = glm::mat4x4(1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0);
    for (auto body : mBodyVector_0){
        auto joint0 = body.getJointRelPosition(nite::JOINT_HEAD);
        std::cout << joint0.x << ", " << joint0.y << ", " << joint0.z << std::endl;
        body.convertCoordinates(H);
        auto joint1 = body.getJointRelPosition(nite::JOINT_HEAD);
        std::cout << joint1.x << ", " << joint1.y << ", " << joint1.z << std::endl;
        exit(0x0);
    }


}


void BodyManager::startRecording()
{
    //TODO
}


void BodyManager::stopRecording()
{
    //TODO
}


void BodyManager::save() const
{
    //TODO
}

void BodyManager::load()
{
    //TODO
}


