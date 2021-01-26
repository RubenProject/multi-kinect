#include "body.h"

#include "context.h"

#include <sstream>
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


inline std::string glmToString(const glm::vec3 &in)
{
    return std::to_string(in.x) + " "
        + std::to_string(in.y) + " " 
        + std::to_string(in.z);
}


inline std::string glmToString(const glm::vec4 &in)
{
    return std::to_string(in.w) + " "
        + std::to_string(in.x) + " "
        + std::to_string(in.y) + " " 
        + std::to_string(in.z);
}


Body::Body()
{}


Body::Body(const nite::Skeleton &tSkeleton, const nite::Plane &plane, uint64_t time)
{
    setJointPositions(tSkeleton);
    setJointOrientations(tSkeleton);
    calcConfidence(tSkeleton);

    glm::vec3 tShoulder, tHip;
    tShoulder = toGlmVec3(tSkeleton.getJoint(nite::JOINT_LEFT_SHOULDER).getPosition());
    tShoulder -= toGlmVec3(tSkeleton.getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition());

    tHip = toGlmVec3(tSkeleton.getJoint(nite::JOINT_LEFT_HIP).getPosition());
    tHip -= toGlmVec3(tSkeleton.getJoint(nite::JOINT_RIGHT_HIP).getPosition());

    mRight = tHip + tShoulder;
    mRight = glm::normalize(mRight);
    //up direction is relative to the ground plane
    mUp = mFloorPlane.normal;
    mForward = glm::cross(mRight, mUp);

    mFloorPlane.point = glm::vec3(plane.point.x, plane.point.y, plane.point.z);
    mFloorPlane.normal = glm::vec3(plane.normal.x, plane.normal.y, plane.normal.z);

    mTimeStamp = time;
    mTrajectory = std::vector<Body*>(TRAJECTORY_SIZE, NULL);
}


Body::~Body()
{}


Body Body::fromString(const std::string &s) 
{
    std::istringstream iss(s);
    float x, y, z, w;
    for (int i = 0; i < NITE_JOINT_COUNT; ++i) {
        iss >> x >> y >> z;
        setJointPosition(glm::vec3(x, y, z), i);
    }
    for (int i = 0; i < NITE_JOINT_COUNT; ++i) {
        iss >> w >> x >> y >> z;
        setJointOrientation(glm::vec4(w, x, y, z), i);
    }

    iss >> x >> y >> z;
    mFloorPlane.point = glm::vec3(x, y, z);
    iss >> x >> y >> z;
    mFloorPlane.normal = glm::vec3(x, y, z);

    iss >> mTimeStamp;
    //ss >> mPosConf;
    //ss >> mOrientConf;
    
    return *this;
}


std::string Body::toString() {
    std::stringstream ss;
    ss << glmToString(getJointAbsPosition(nite::JOINT_HEAD)) << " ";
    ss << glmToString(getJointAbsPosition(nite::JOINT_NECK)) << " ";
    ss << glmToString(getJointAbsPosition(nite::JOINT_LEFT_SHOULDER)) << " ";
    ss << glmToString(getJointAbsPosition(nite::JOINT_RIGHT_SHOULDER)) << " ";
    ss << glmToString(getJointAbsPosition(nite::JOINT_LEFT_ELBOW)) << " ";
    ss << glmToString(getJointAbsPosition(nite::JOINT_RIGHT_ELBOW)) << " ";
    ss << glmToString(getJointAbsPosition(nite::JOINT_LEFT_HAND)) << " ";
    ss << glmToString(getJointAbsPosition(nite::JOINT_RIGHT_HAND)) << " ";
    ss << glmToString(getJointAbsPosition(nite::JOINT_TORSO)) << " ";
    ss << glmToString(getJointAbsPosition(nite::JOINT_LEFT_HIP)) << " ";
    ss << glmToString(getJointAbsPosition(nite::JOINT_RIGHT_HIP)) << " ";
    ss << glmToString(getJointAbsPosition(nite::JOINT_LEFT_KNEE)) << " ";
    ss << glmToString(getJointAbsPosition(nite::JOINT_RIGHT_KNEE)) << " ";
    ss << glmToString(getJointAbsPosition(nite::JOINT_LEFT_FOOT)) << " ";
    ss << glmToString(getJointAbsPosition(nite::JOINT_RIGHT_FOOT)) << " ";

    ss << glmToString(getJointOrientation(nite::JOINT_HEAD)) << " ";
    ss << glmToString(getJointOrientation(nite::JOINT_NECK)) << " ";
    ss << glmToString(getJointOrientation(nite::JOINT_LEFT_SHOULDER)) << " ";
    ss << glmToString(getJointOrientation(nite::JOINT_RIGHT_SHOULDER)) << " ";
    ss << glmToString(getJointOrientation(nite::JOINT_LEFT_ELBOW)) << " ";
    ss << glmToString(getJointOrientation(nite::JOINT_RIGHT_ELBOW)) << " ";
    ss << glmToString(getJointOrientation(nite::JOINT_LEFT_HAND)) << " ";
    ss << glmToString(getJointOrientation(nite::JOINT_RIGHT_HAND)) << " ";
    ss << glmToString(getJointOrientation(nite::JOINT_TORSO)) << " ";
    ss << glmToString(getJointOrientation(nite::JOINT_LEFT_HIP)) << " ";
    ss << glmToString(getJointOrientation(nite::JOINT_RIGHT_HIP)) << " ";
    ss << glmToString(getJointOrientation(nite::JOINT_LEFT_KNEE)) << " ";
    ss << glmToString(getJointOrientation(nite::JOINT_RIGHT_KNEE)) << " ";
    ss << glmToString(getJointOrientation(nite::JOINT_LEFT_FOOT)) << " ";
    ss << glmToString(getJointOrientation(nite::JOINT_RIGHT_FOOT)) << " ";

    ss << glmToString(mFloorPlane.point) << " ";
    ss << glmToString(mFloorPlane.normal) << " ";

    ss << mTimeStamp << " ";
    ss << mPosConf << " ";
    ss << mOrientConf << " ";

    return ss.str();
}


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


void Body::setJointPosition(const glm::vec3 &pos, const int i)
{
    switch (i)
    {
        case 0: mPosHEAD = pos; break;
        case 1: mPosNECK = pos; break;
        case 2: mPosL_SHOULDER = pos; break;
        case 3: mPosR_SHOULDER = pos; break;
        case 4: mPosL_ELBOW = pos; break;
        case 5: mPosR_ELBOW = pos; break;
        case 6: mPosL_HAND = pos; break;
        case 7: mPosR_HAND = pos; break;
        case 8: mPosTORSO = pos; break;
        case 9: mPosL_HIP = pos; break;
        case 10: mPosR_HIP = pos; break;
        case 11: mPosL_KNEE = pos; break;
        case 12: mPosR_KNEE = pos; break;
        case 13: mPosL_FOOT = pos; break;
        case 14: mPosR_FOOT = pos; break;
        default: break;
    }
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


void Body::setJointOrientation(const glm::vec4 &quat, const int i)
{
    switch (i)
    {
        case 0: mQuatHEAD = quat; break;
        case 1: mQuatNECK = quat; break;
        case 2: mQuatL_SHOULDER = quat; break;
        case 3: mQuatR_SHOULDER = quat; break;
        case 4: mQuatL_ELBOW = quat; break;
        case 5: mQuatR_ELBOW = quat; break;
        case 6: mQuatL_HAND = quat; break;
        case 7: mQuatR_HAND = quat; break;
        case 8: mQuatTORSO = quat; break;
        case 9: mQuatL_HIP = quat; break;
        case 10: mQuatR_HIP = quat; break;
        case 11: mQuatL_KNEE = quat; break;
        case 12: mQuatR_KNEE = quat; break;
        case 13: mQuatL_FOOT = quat; break;
        case 14: mQuatR_FOOT = quat; break;
        default: break;
    }
}


void Body::calcConfidence(const nite::Skeleton &tSkeleton)
{
    float avgPos = 0.f;
    avgPos += tSkeleton.getJoint(nite::JOINT_HEAD).getPositionConfidence();
    avgPos += tSkeleton.getJoint(nite::JOINT_NECK).getPositionConfidence();
    avgPos += tSkeleton.getJoint(nite::JOINT_LEFT_SHOULDER).getPositionConfidence();
    avgPos += tSkeleton.getJoint(nite::JOINT_RIGHT_SHOULDER).getPositionConfidence();
    avgPos += tSkeleton.getJoint(nite::JOINT_LEFT_ELBOW).getPositionConfidence();
    avgPos += tSkeleton.getJoint(nite::JOINT_RIGHT_ELBOW).getPositionConfidence();
    avgPos += tSkeleton.getJoint(nite::JOINT_LEFT_HAND).getPositionConfidence();
    avgPos += tSkeleton.getJoint(nite::JOINT_RIGHT_HAND).getPositionConfidence();
    avgPos += tSkeleton.getJoint(nite::JOINT_TORSO).getPositionConfidence();
    avgPos += tSkeleton.getJoint(nite::JOINT_LEFT_HIP).getPositionConfidence();
    avgPos += tSkeleton.getJoint(nite::JOINT_RIGHT_HIP).getPositionConfidence();
    avgPos += tSkeleton.getJoint(nite::JOINT_LEFT_KNEE).getPositionConfidence();
    avgPos += tSkeleton.getJoint(nite::JOINT_RIGHT_KNEE).getPositionConfidence();
    avgPos += tSkeleton.getJoint(nite::JOINT_LEFT_FOOT).getPositionConfidence();
    avgPos += tSkeleton.getJoint(nite::JOINT_RIGHT_FOOT).getPositionConfidence();

    float avgOrient = 0.f;
    avgOrient += tSkeleton.getJoint(nite::JOINT_HEAD).getOrientationConfidence();
    avgOrient += tSkeleton.getJoint(nite::JOINT_NECK).getOrientationConfidence();
    avgOrient += tSkeleton.getJoint(nite::JOINT_LEFT_SHOULDER).getOrientationConfidence();
    avgOrient += tSkeleton.getJoint(nite::JOINT_RIGHT_SHOULDER).getOrientationConfidence();
    avgOrient += tSkeleton.getJoint(nite::JOINT_LEFT_ELBOW).getOrientationConfidence();
    avgOrient += tSkeleton.getJoint(nite::JOINT_RIGHT_ELBOW).getOrientationConfidence();
    avgOrient += tSkeleton.getJoint(nite::JOINT_LEFT_HAND).getOrientationConfidence();
    avgOrient += tSkeleton.getJoint(nite::JOINT_RIGHT_HAND).getOrientationConfidence();
    avgOrient += tSkeleton.getJoint(nite::JOINT_TORSO).getOrientationConfidence();
    avgOrient += tSkeleton.getJoint(nite::JOINT_LEFT_HIP).getOrientationConfidence();
    avgOrient += tSkeleton.getJoint(nite::JOINT_RIGHT_HIP).getOrientationConfidence();
    avgOrient += tSkeleton.getJoint(nite::JOINT_LEFT_KNEE).getOrientationConfidence();
    avgOrient += tSkeleton.getJoint(nite::JOINT_RIGHT_KNEE).getOrientationConfidence();
    avgOrient += tSkeleton.getJoint(nite::JOINT_LEFT_FOOT).getOrientationConfidence();
    avgOrient += tSkeleton.getJoint(nite::JOINT_RIGHT_FOOT).getOrientationConfidence();

    mPosConf = avgPos / 15;
    mOrientConf = avgOrient / 15;
}


float Body::getConfidence() const
{
    //TODO update to also use orientation confidence
    return mPosConf;
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


