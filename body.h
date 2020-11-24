#pragma once

#define GLM_FORCE_SWIZZLE
#include "glm/glm.hpp"

#include "OpenNI.h"
#include "NiTE.h"

#include <vector>


struct Plane
{
    glm::vec3 point;
    glm::vec3 normal;
};


class Body
{
public:
    Body();
    ~Body();
    bool initialize(const nite::Skeleton &tSkeleton, uint64_t time);
    void convertCoordinates(const glm::mat4x4 &H);

    glm::vec3 getRootPosition() const;
    glm::vec3 getForward() const;
    glm::vec3 getRight() const;
    glm::vec3 getUp() const;
    uint64_t getTimeStamp() const;
    void setTimeStamp(uint64_t time);

    nite::Plane getFloorPlane() const;
    void setFloorPlane(nite::Plane plane);

    glm::vec3 getJointAbsPosition(const nite::JointType tJointID) const;
    glm::vec3 getJointRelPosition(const nite::JointType tJointID) const;
    void setJointPosition(const nite::JointType tJointID, nite::Point3f value);
    void setJointPosition(const nite::JointType tJointID, glm::vec3 value);
    void setJointPositions(const nite::Skeleton &tSkeleton);

    glm::vec4 getJointOrientation(const nite::JointType tJointID) const;
    void setJointOrientations(const nite::JointType tJointID, nite::Quaternion value);
    void setJointOrientations(const nite::JointType tJointID, glm::vec4 value);
    void setJointOrientations(const nite::Skeleton &tSkeleton);

    Body *getTrajectory(int i) const;
    void setTrajectory(Body *body, int i);

private:
    glm::vec3 mPosROOT;
    glm::vec3 mUp, mForward, mRight;
    Plane mFloorPlane;
    uint64_t mTimeStamp;

    glm::vec3 mPosHEAD;
    glm::vec3 mPosNECK;
    glm::vec3 mPosL_SHOULDER;
    glm::vec3 mPosR_SHOULDER;
    glm::vec3 mPosL_ELBOW;
    glm::vec3 mPosR_ELBOW;
    glm::vec3 mPosL_HAND;
    glm::vec3 mPosR_HAND;
    glm::vec3 mPosTORSO;
    glm::vec3 mPosL_HIP;
    glm::vec3 mPosR_HIP;
    glm::vec3 mPosL_KNEE;
    glm::vec3 mPosR_KNEE;
    glm::vec3 mPosL_FOOT;
    glm::vec3 mPosR_FOOT;

    glm::vec4 mQuatHEAD;
    glm::vec4 mQuatNECK;
    glm::vec4 mQuatL_SHOULDER;
    glm::vec4 mQuatR_SHOULDER;
    glm::vec4 mQuatL_ELBOW;
    glm::vec4 mQuatR_ELBOW;
    glm::vec4 mQuatL_HAND;
    glm::vec4 mQuatR_HAND;
    glm::vec4 mQuatTORSO;
    glm::vec4 mQuatL_HIP;
    glm::vec4 mQuatR_HIP;
    glm::vec4 mQuatL_KNEE;
    glm::vec4 mQuatR_KNEE;
    glm::vec4 mQuatL_FOOT;
    glm::vec4 mQuatR_FOOT;

    // surrounding trajectory
    std::vector<Body*> mTrajectory;
};
