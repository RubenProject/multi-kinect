#pragma once

#include "pch.h"


const int TRAJECTORY_SIZE = 12;


struct Plane
{
    glm::dvec3 point;
    glm::dvec3 normal;
};


class Body
{
public:
    Body();
    Body(const nite::Skeleton &tSkeleton, const nite::Plane &plane, uint64_t tTime);
    ~Body();

    YAML::Node serialize();
    void deserialize(const YAML::Node &body);

    void transform(const glm::dmat3x3 &R, const glm::dvec3 &t);
    void rotateRoot(const glm::dmat3x3 &R);
    double compareTo(const Body &other);

    glm::dvec3 getRootPosition() const;
    glm::dvec3 getForward() const;
    glm::dvec3 getRight() const;
    glm::dvec3 getUp() const;
    uint64_t getTimeStamp() const;
    void setTimeStamp(uint64_t time);

    void getFloorPlane(Plane &plane);
    void getFloorPlane(nite::Plane &plane);
    void setFloorPlane(Plane &plane);
    void setFloorPlane(nite::Plane &plane);

    glm::dvec3 getJointAbsPosition(const nite::JointType tJointID) const;
    glm::dvec3 getJointRelPosition(const nite::JointType tJointID) const;
    void setJointPosition(const glm::dvec3 &pos, const int i);
    void setJointPositions(const nite::Skeleton &tSkeleton);

    glm::dvec4 getJointOrientation(const nite::JointType tJointID) const;
    void setJointOrientation(const glm::dvec4 &quat, const int i);
    void setJointOrientations(const nite::Skeleton &tSkeleton);

    float getConfidence() const;
    void calcConfidence(const nite::Skeleton &tSkeleton);
    void calcRootOrientations();

    Body *getTrajectory(int i) const;
    void setTrajectory(Body *body, int i);

private:
    glm::dvec3 mPosROOT;
    glm::dvec3 mUp, mForward, mRight;

    glm::dvec3 mPosHEAD;
    glm::dvec3 mPosNECK;
    glm::dvec3 mPosL_SHOULDER;
    glm::dvec3 mPosR_SHOULDER;
    glm::dvec3 mPosL_ELBOW;
    glm::dvec3 mPosR_ELBOW;
    glm::dvec3 mPosL_HAND;
    glm::dvec3 mPosR_HAND;
    glm::dvec3 mPosTORSO;
    glm::dvec3 mPosL_HIP;
    glm::dvec3 mPosR_HIP;
    glm::dvec3 mPosL_KNEE;
    glm::dvec3 mPosR_KNEE;
    glm::dvec3 mPosL_FOOT;
    glm::dvec3 mPosR_FOOT;

    glm::dvec4 mQuatHEAD;
    glm::dvec4 mQuatNECK;
    glm::dvec4 mQuatL_SHOULDER;
    glm::dvec4 mQuatR_SHOULDER;
    glm::dvec4 mQuatL_ELBOW;
    glm::dvec4 mQuatR_ELBOW;
    glm::dvec4 mQuatL_HAND;
    glm::dvec4 mQuatR_HAND;
    glm::dvec4 mQuatTORSO;
    glm::dvec4 mQuatL_HIP;
    glm::dvec4 mQuatR_HIP;
    glm::dvec4 mQuatL_KNEE;
    glm::dvec4 mQuatR_KNEE;
    glm::dvec4 mQuatL_FOOT;
    glm::dvec4 mQuatR_FOOT;

    Plane mFloorPlane;
    uint64_t mTimeStamp;
    float mPosConf;
    float mOrientConf;

    // surrounding trajectory
    std::vector<Body*> mTrajectory;
};
