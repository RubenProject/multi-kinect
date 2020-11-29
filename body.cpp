#include "body.h"

#include <iostream>


inline glm::vec3 toGlmVec3(const nite::Point3f &in)
{
    return glm::vec3(in.x, in.y, in.z);
}


inline glm::vec3 toGlmVec3(const glm::vec4 &in)
{
    glm::vec4 out = in;
    out /= out[3];
    return glm::vec3(out.w, out.x, out.y);
}


inline glm::vec4 toGlmVec4(const nite::Quaternion &in)
{
    return glm::vec4(in.w, in.x, in.y, in.z);
}


inline nite::Point3f toNitePoint3f(const glm::vec3 &in)
{
    return nite::Point3f(in.x, in.y, in.z);
}


inline nite::Quaternion toNiteQuat(const glm::vec4 &in)
{
    return nite::Quaternion(in.w, in.x, in.y, in.z);
}

Body::Body()
{}

Body::Body(const nite::Skeleton &tSkeleton, uint64_t time)
{
    setJointPositions(tSkeleton);
    setJointOrientations(tSkeleton);

    glm::vec3 tShoulder, tHip;
    tShoulder = toGlmVec3(tSkeleton.getJoint(nite::JOINT_LEFT_SHOULDER).getPosition());
    tShoulder -= toGlmVec3(tSkeleton.getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition());

    tHip = toGlmVec3(tSkeleton.getJoint(nite::JOINT_LEFT_HIP).getPosition());
    tHip -= toGlmVec3(tSkeleton.getJoint(nite::JOINT_RIGHT_HIP).getPosition());

    mRight = tHip + tShoulder;
    mRight = glm::normalize(mRight);
    mUp = glm::vec3(0, 1, 0);
    mForward = glm::cross(mRight, mUp);

    mTimeStamp = time;
    mTrajectory = std::vector<Body*>(TRAJECTORY_SIZE, NULL);
}


Body::~Body()
{}


void Body::convertCoordinates(const glm::mat4x4 &H)
{
    mPosHEAD = toGlmVec3(H * glm::vec4(mPosHEAD, 1.0f));
    mPosNECK = toGlmVec3(H * glm::vec4(mPosNECK, 1.0f));
    mPosL_SHOULDER = toGlmVec3(H * glm::vec4(mPosL_SHOULDER, 1.0f));
    mPosR_SHOULDER = toGlmVec3(H * glm::vec4(mPosR_SHOULDER, 1.0f));
    mPosL_ELBOW = toGlmVec3(H * glm::vec4(mPosL_ELBOW, 1.0f));
    mPosR_ELBOW = toGlmVec3(H * glm::vec4(mPosR_ELBOW, 1.0f));
    mPosL_HAND = toGlmVec3(H * glm::vec4(mPosL_HAND, 1.0f));
    mPosR_HAND = toGlmVec3(H * glm::vec4(mPosR_HAND, 1.0f));
    mPosTORSO = toGlmVec3(H * glm::vec4(mPosTORSO, 1.0f));
    mPosL_HIP = toGlmVec3(H * glm::vec4(mPosL_HIP, 1.0f));
    mPosR_HIP = toGlmVec3(H * glm::vec4(mPosR_HIP, 1.0f));
    mPosL_KNEE = toGlmVec3(H * glm::vec4(mPosL_KNEE, 1.0f));
    mPosR_KNEE = toGlmVec3(H * glm::vec4(mPosR_KNEE, 1.0f));
    mPosL_FOOT = toGlmVec3(H * glm::vec4(mPosL_FOOT, 1.0f));
    mPosR_FOOT = toGlmVec3(H * glm::vec4(mPosR_FOOT, 1.0f));

    mPosROOT = toGlmVec3(H * glm::vec4(mPosROOT, 1.0f));

    //TODO
    //update rotations as well
}


nite::Plane Body::getFloorPlane() const
{
    return nite::Plane(nite::Point3f(mFloorPlane.point.x, mFloorPlane.point.y, mFloorPlane.point.z),
                       nite::Point3f(mFloorPlane.normal.x, mFloorPlane.normal.y, mFloorPlane.normal.z));
}


void Body::setFloorPlane(nite::Plane plane)
{
    mFloorPlane.point = {plane.point.x, plane.point.y, plane.point.z};
    mFloorPlane.normal = {plane.normal.x, plane.normal.y, plane.normal.z};
}


glm::vec3 Body::getJointAbsPosition(const nite::JointType tJointID) const
{
    switch (tJointID)
    {
        case nite::JOINT_HEAD: return mPosHEAD;
        case nite::JOINT_NECK: return mPosNECK;
        case nite::JOINT_LEFT_SHOULDER: return mPosL_SHOULDER;
        case nite::JOINT_RIGHT_SHOULDER: return mPosR_SHOULDER;
        case nite::JOINT_LEFT_ELBOW: return mPosL_ELBOW;
        case nite::JOINT_RIGHT_ELBOW: return mPosR_ELBOW;
        case nite::JOINT_LEFT_HAND: return mPosL_HAND;
        case nite::JOINT_RIGHT_HAND: return mPosR_HAND;
        case nite::JOINT_TORSO: return mPosTORSO;
        case nite::JOINT_LEFT_HIP: return mPosL_HIP;
        case nite::JOINT_RIGHT_HIP: return mPosR_HIP;
        case nite::JOINT_LEFT_KNEE: return mPosL_KNEE;
        case nite::JOINT_RIGHT_KNEE: return mPosR_KNEE;
        case nite::JOINT_LEFT_FOOT: return mPosL_FOOT;
        case nite::JOINT_RIGHT_FOOT: return mPosR_FOOT;
        default:
            std::cout << "Joint type not supported!" << std::endl;
            return glm::vec3(0.0, 0.0, 0.0);
    }
}

glm::vec3 Body::getJointRelPosition(const nite::JointType tJointID) const
{
    return getJointAbsPosition(tJointID) - mPosROOT;
}

glm::vec3 Body::getRootPosition() const
{
    return mPosROOT;
}


glm::vec4 Body::getJointOrientation(const nite::JointType tJointID) const
{
    switch (tJointID)
    {
        case nite::JOINT_HEAD: return mQuatHEAD;
        case nite::JOINT_NECK: return mQuatNECK;
        case nite::JOINT_LEFT_SHOULDER: return mQuatL_SHOULDER;
        case nite::JOINT_RIGHT_SHOULDER: return mQuatR_SHOULDER;
        case nite::JOINT_LEFT_ELBOW: return mQuatL_ELBOW;
        case nite::JOINT_RIGHT_ELBOW: return mQuatR_ELBOW;
        case nite::JOINT_LEFT_HAND: return mQuatL_HAND;
        case nite::JOINT_RIGHT_HAND: return mQuatR_HAND;
        case nite::JOINT_TORSO: return mQuatTORSO;
        case nite::JOINT_LEFT_HIP: return mQuatL_HIP;
        case nite::JOINT_RIGHT_HIP: return mQuatR_HIP;
        case nite::JOINT_LEFT_KNEE: return mQuatL_KNEE;
        case nite::JOINT_RIGHT_KNEE: return mQuatR_KNEE;
        case nite::JOINT_LEFT_FOOT: return mQuatL_FOOT;
        case nite::JOINT_RIGHT_FOOT: return mQuatR_FOOT;
        default:
            std::cout << "Joint type not supported!" << std::endl;
            return glm::vec4(0.0, 0.0, 0.0, 0.0);
    }
}


glm::vec3 Body::getForward() const
{
    return mForward;
}


glm::vec3 Body::getRight() const
{
    return mRight;
}


glm::vec3 Body::getUp() const
{
    return mUp;
}


uint64_t Body::getTimeStamp() const
{
    return mTimeStamp;
}

void Body::setTimeStamp(uint64_t time)
{
    mTimeStamp = time;
}


void Body::setJointPositions(const nite::Skeleton &tSkeleton)
{
    mPosL_HIP = toGlmVec3(tSkeleton.getJoint(nite::JOINT_LEFT_HIP).getPosition());
    mPosR_HIP = toGlmVec3(tSkeleton.getJoint(nite::JOINT_RIGHT_HIP).getPosition());

    mPosROOT = mPosL_HIP;
    mPosROOT += mPosR_HIP;
    mPosROOT /=  2;

    mPosHEAD = toGlmVec3(tSkeleton.getJoint(nite::JOINT_HEAD).getPosition());
    mPosNECK = toGlmVec3(tSkeleton.getJoint(nite::JOINT_NECK).getPosition());
    mPosL_SHOULDER = toGlmVec3(tSkeleton.getJoint(nite::JOINT_LEFT_SHOULDER).getPosition());
    mPosR_SHOULDER = toGlmVec3(tSkeleton.getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition());
    mPosL_ELBOW = toGlmVec3(tSkeleton.getJoint(nite::JOINT_LEFT_ELBOW).getPosition());
    mPosR_ELBOW = toGlmVec3(tSkeleton.getJoint(nite::JOINT_RIGHT_ELBOW).getPosition());
    mPosL_HAND = toGlmVec3(tSkeleton.getJoint(nite::JOINT_LEFT_HAND).getPosition());
    mPosR_HAND = toGlmVec3(tSkeleton.getJoint(nite::JOINT_RIGHT_HAND).getPosition());
    mPosTORSO = toGlmVec3(tSkeleton.getJoint(nite::JOINT_TORSO).getPosition());
    mPosL_KNEE = toGlmVec3(tSkeleton.getJoint(nite::JOINT_LEFT_KNEE).getPosition());
    mPosR_KNEE = toGlmVec3(tSkeleton.getJoint(nite::JOINT_RIGHT_KNEE).getPosition());
    mPosL_FOOT = toGlmVec3(tSkeleton.getJoint(nite::JOINT_LEFT_FOOT).getPosition());
    mPosR_FOOT = toGlmVec3(tSkeleton.getJoint(nite::JOINT_RIGHT_FOOT).getPosition());
}


void Body::setJointOrientations(const nite::Skeleton &tSkeleton)
{
    mQuatHEAD = toGlmVec4(tSkeleton.getJoint(nite::JOINT_HEAD).getOrientation());
    mQuatNECK = toGlmVec4(tSkeleton.getJoint(nite::JOINT_NECK).getOrientation());
    mQuatL_SHOULDER = toGlmVec4(tSkeleton.getJoint(nite::JOINT_LEFT_SHOULDER).getOrientation());
    mQuatR_SHOULDER = toGlmVec4(tSkeleton.getJoint(nite::JOINT_RIGHT_SHOULDER).getOrientation());
    mQuatL_ELBOW = toGlmVec4(tSkeleton.getJoint(nite::JOINT_LEFT_ELBOW).getOrientation());
    mQuatR_ELBOW = toGlmVec4(tSkeleton.getJoint(nite::JOINT_RIGHT_ELBOW).getOrientation());
    mQuatL_HAND = toGlmVec4(tSkeleton.getJoint(nite::JOINT_LEFT_HAND).getOrientation());
    mQuatR_HAND = toGlmVec4(tSkeleton.getJoint(nite::JOINT_RIGHT_HAND).getOrientation());
    mQuatTORSO = toGlmVec4(tSkeleton.getJoint(nite::JOINT_TORSO).getOrientation());
    mQuatL_HIP = toGlmVec4(tSkeleton.getJoint(nite::JOINT_LEFT_HIP).getOrientation());
    mQuatR_HIP = toGlmVec4(tSkeleton.getJoint(nite::JOINT_RIGHT_HIP).getOrientation());
    mQuatL_KNEE = toGlmVec4(tSkeleton.getJoint(nite::JOINT_LEFT_KNEE).getOrientation());
    mQuatR_KNEE = toGlmVec4(tSkeleton.getJoint(nite::JOINT_RIGHT_KNEE).getOrientation());
    mQuatL_FOOT = toGlmVec4(tSkeleton.getJoint(nite::JOINT_LEFT_FOOT).getOrientation());
    mQuatR_FOOT = toGlmVec4(tSkeleton.getJoint(nite::JOINT_RIGHT_FOOT).getOrientation());
}


Body *Body::getTrajectory(int i) const
{
    if (i < 0 || i > 11){
        return mTrajectory[i];
    }
    return NULL;
}


void Body::setTrajectory(Body *body, int i)
{
    if (i < 0 || i > 11){
        mTrajectory[i] = body;
    }
}


