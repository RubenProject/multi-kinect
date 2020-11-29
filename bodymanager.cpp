#include "bodymanager.h"

#include <iostream>


BodyManager::BodyManager()
{
}


BodyManager::~BodyManager()
{
}


void BodyManager::initialize(const std::vector<nite::UserTracker> &tTrackers)
{
    mUserTrackers = tTrackers;
    mBodyVectors = std::vector<std::vector<Body>>(tTrackers.size());
    mLastBodies = std::vector<Body>(tTrackers.size());
    mReadyStates = std::vector<bool>(tTrackers.size(), false);
}


void BodyManager::addBody(const nite::UserTrackerFrameRef &frame, const int tStreamID, uint64_t time)
{
    /*
    Body body;
    const nite::Array<nite::UserData> &tUsers = frame.getUsers();
    for (int i = 0; i < tUsers.getSize(); ++i){
        const nite::UserData &tUser = tUsers[i];
        if (tUser.isNew()){
            mUserTrackers[tStreamID]->startSkeletonTracking(tUser.getId());
        }
        const nite::Skeleton &tSkeleton = tUser.getSkeleton();
        body.initialize(tSkeleton, time);
    }

    mBodyVectors[tStreamID].push_back(body);
    mBodyReadyStates[tStreamID] = true;
    */
}


const Body &BodyManager::getBody(const int tStreamID)
{
    return mLastBodies[tStreamID];
}


void BodyManager::setBody(const nite::UserTrackerFrameRef &frame, const int tStreamID)
{
    const nite::Array<nite::UserData> &tUsers = frame.getUsers();
    for (int i = 0; i < tUsers.getSize(); ++i){
        const nite::UserData &tUser = tUsers[i];
        if (tUser.isNew()){
            mUserTrackers[tStreamID].startSkeletonTracking(tUser.getId());
        }
        const nite::Skeleton &tSkeleton = tUser.getSkeleton();
        if (tSkeleton.getState() == nite::SKELETON_TRACKED){
            Body body(tSkeleton, -1);
            mLastBodies[tStreamID] = body;
            break;
        }
    } 
}


void BodyManager::process()
{
    for (size_t i = 0; i < mBodyVectors[0].size(); i++){
        //mBodyVector_0[i];
    }

    for (size_t i = 0; i < mBodyVectors[1].size(); i++){
        //mBodyVector_1[i];
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


