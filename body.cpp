#include "body.h"

#include "context.h"


inline glm::dvec3 toGlmVec3(const nite::Point3f &in)
{
    return glm::dvec3(in.x, in.y, in.z);
}


inline glm::dvec4 toGlmVec4(const nite::Quaternion &in)
{
    return glm::dvec4(in.w, in.x, in.y, in.z);
}

inline std::string glmToString(const glm::dvec3 &in)
{
    return std::to_string(in.x) + " "
        + std::to_string(in.y) + " " 
        + std::to_string(in.z);
}


inline std::string glmToString(const glm::dvec4 &in)
{
    return std::to_string(in.w) + " "
        + std::to_string(in.x) + " "
        + std::to_string(in.y) + " " 
        + std::to_string(in.z);
}


namespace YAML {
    template<>
    struct convert<glm::dvec3> {
        static Node encode(const glm::dvec3& rhs) {
            Node node;
            node.push_back(rhs.x);
            node.push_back(rhs.y);
            node.push_back(rhs.z);
            return node;
        }

        static bool decode(const Node& node, glm::dvec3& rhs) {
            if(!node.IsSequence() || node.size() != 3) {
                return false;
            }
            rhs.x = node[0].as<double>();
            rhs.y = node[1].as<double>();
            rhs.z = node[2].as<double>();
            return true;
        }
    };

    template<>
    struct convert<glm::dvec4> {
        static Node encode(const glm::dvec4& rhs) {
            Node node;
            node.push_back(rhs.x);
            node.push_back(rhs.y);
            node.push_back(rhs.z);
            node.push_back(rhs.w);
            return node;
        }

        static bool decode(const Node& node, glm::dvec4& rhs) {
            if(!node.IsSequence() || node.size() != 4) {
                return false;
            }
            rhs.x = node[0].as<double>();
            rhs.y = node[1].as<double>();
            rhs.z = node[2].as<double>();
            rhs.w = node[3].as<double>();
            return true;
        }
    };
}


Body::Body()
{}


Body::Body(const nite::Skeleton &tSkeleton, const nite::Plane &plane, uint64_t time)
{
    setJointPositions(tSkeleton);
    setJointOrientations(tSkeleton);
    calcConfidence(tSkeleton);


    mFloorPlane.point = glm::dvec3(plane.point.x, plane.point.y, plane.point.z);
    mFloorPlane.normal = glm::dvec3(plane.normal.x, plane.normal.y, plane.normal.z);

    calcRootOrientations();

    mTimeStamp = time;
    mTrajectory = std::vector<Body*>(TRAJECTORY_SIZE, NULL);
}


Body::~Body()
{}


YAML::Node Body::serialize()
{
    YAML::Node body;
    body["JOINT_POSITIONS"].push_back(getJointAbsPosition(nite::JOINT_HEAD));
    body["JOINT_POSITIONS"].push_back(getJointAbsPosition(nite::JOINT_NECK));
    body["JOINT_POSITIONS"].push_back(getJointAbsPosition(nite::JOINT_LEFT_SHOULDER));
    body["JOINT_POSITIONS"].push_back(getJointAbsPosition(nite::JOINT_RIGHT_SHOULDER));
    body["JOINT_POSITIONS"].push_back(getJointAbsPosition(nite::JOINT_LEFT_ELBOW));
    body["JOINT_POSITIONS"].push_back(getJointAbsPosition(nite::JOINT_RIGHT_ELBOW));
    body["JOINT_POSITIONS"].push_back(getJointAbsPosition(nite::JOINT_LEFT_HAND));
    body["JOINT_POSITIONS"].push_back(getJointAbsPosition(nite::JOINT_RIGHT_HAND));
    body["JOINT_POSITIONS"].push_back(getJointAbsPosition(nite::JOINT_TORSO));
    body["JOINT_POSITIONS"].push_back(getJointAbsPosition(nite::JOINT_LEFT_HIP));
    body["JOINT_POSITIONS"].push_back(getJointAbsPosition(nite::JOINT_RIGHT_HIP));
    body["JOINT_POSITIONS"].push_back(getJointAbsPosition(nite::JOINT_LEFT_KNEE));
    body["JOINT_POSITIONS"].push_back(getJointAbsPosition(nite::JOINT_RIGHT_KNEE));
    body["JOINT_POSITIONS"].push_back(getJointAbsPosition(nite::JOINT_LEFT_FOOT));
    body["JOINT_POSITIONS"].push_back(getJointAbsPosition(nite::JOINT_RIGHT_FOOT));

    body["JOINT_ORIENTATIONS"].push_back(getJointOrientation(nite::JOINT_HEAD));
    body["JOINT_ORIENTATIONS"].push_back(getJointOrientation(nite::JOINT_NECK));
    body["JOINT_ORIENTATIONS"].push_back(getJointOrientation(nite::JOINT_LEFT_SHOULDER));
    body["JOINT_ORIENTATIONS"].push_back(getJointOrientation(nite::JOINT_RIGHT_SHOULDER));
    body["JOINT_ORIENTATIONS"].push_back(getJointOrientation(nite::JOINT_LEFT_ELBOW));
    body["JOINT_ORIENTATIONS"].push_back(getJointOrientation(nite::JOINT_RIGHT_ELBOW));
    body["JOINT_ORIENTATIONS"].push_back(getJointOrientation(nite::JOINT_LEFT_HAND));
    body["JOINT_ORIENTATIONS"].push_back(getJointOrientation(nite::JOINT_RIGHT_HAND));
    body["JOINT_ORIENTATIONS"].push_back(getJointOrientation(nite::JOINT_TORSO));
    body["JOINT_ORIENTATIONS"].push_back(getJointOrientation(nite::JOINT_LEFT_HIP));
    body["JOINT_ORIENTATIONS"].push_back(getJointOrientation(nite::JOINT_RIGHT_HIP));
    body["JOINT_ORIENTATIONS"].push_back(getJointOrientation(nite::JOINT_LEFT_KNEE));
    body["JOINT_ORIENTATIONS"].push_back(getJointOrientation(nite::JOINT_RIGHT_KNEE));
    body["JOINT_ORIENTATIONS"].push_back(getJointOrientation(nite::JOINT_LEFT_FOOT));
    body["JOINT_ORIENTATIONS"].push_back(getJointOrientation(nite::JOINT_RIGHT_FOOT));

    body["FLOOR_PLANE"].push_back(mFloorPlane.point);
    body["FLOOR_PLANE"].push_back(mFloorPlane.normal);

    body["TIME"] = mTimeStamp;
    body["POSITION_CONFIDENCE"] = mPosConf;
    body["ORIENTATION_CONFIDENCE"] = mOrientConf;
    
    return body;
}


void Body::deserialize(const YAML::Node &body)
{
    mPosHEAD = body["JOINT_POSITIONS"][0].as<glm::dvec3>();
    mPosNECK = body["JOINT_POSITIONS"][1].as<glm::dvec3>();
    mPosL_SHOULDER = body["JOINT_POSITIONS"][2].as<glm::dvec3>();
    mPosR_SHOULDER = body["JOINT_POSITIONS"][3].as<glm::dvec3>();
    mPosL_ELBOW = body["JOINT_POSITIONS"][4].as<glm::dvec3>();
    mPosR_ELBOW = body["JOINT_POSITIONS"][5].as<glm::dvec3>();
    mPosL_HAND = body["JOINT_POSITIONS"][6].as<glm::dvec3>();
    mPosR_HAND = body["JOINT_POSITIONS"][7].as<glm::dvec3>();
    mPosTORSO = body["JOINT_POSITIONS"][8].as<glm::dvec3>();
    mPosL_HIP = body["JOINT_POSITIONS"][9].as<glm::dvec3>();
    mPosR_HIP = body["JOINT_POSITIONS"][10].as<glm::dvec3>();
    mPosL_KNEE = body["JOINT_POSITIONS"][11].as<glm::dvec3>();
    mPosR_KNEE = body["JOINT_POSITIONS"][12].as<glm::dvec3>();
    mPosL_FOOT = body["JOINT_POSITIONS"][13].as<glm::dvec3>();
    mPosR_FOOT = body["JOINT_POSITIONS"][14].as<glm::dvec3>();

    mFloorPlane.point = body["FLOOR_PLANE"][0].as<glm::dvec3>();
    mFloorPlane.normal = body["FLOOR_PLANE"][1].as<glm::dvec3>();

    mTimeStamp = body["TIME"].as<uint64_t>();
    mPosConf = body["POSITION_CONFIDENCE"].as<float>();
    mOrientConf = body["ORIENTATION_CONFIDENCE"].as<float>();

    calcRootOrientations();
}


void Body::transform(const glm::dmat3x3 &R, const glm::dvec3 &t)
{
    mPosHEAD = R * mPosHEAD + t;
    mPosNECK = R * mPosNECK + t;
    mPosL_SHOULDER = R * mPosL_SHOULDER + t;
    mPosR_SHOULDER = R * mPosR_SHOULDER + t;
    mPosL_ELBOW = R * mPosL_ELBOW + t;
    mPosR_ELBOW = R * mPosR_ELBOW + t;
    mPosL_HAND = R * mPosL_HAND + t;
    mPosR_HAND = R * mPosR_HAND + t;
    mPosTORSO = R * mPosTORSO + t;
    mPosL_HIP = R * mPosL_HIP + t;
    mPosR_HIP = R * mPosR_HIP + t;
    mPosL_KNEE = R * mPosL_KNEE + t;
    mPosR_KNEE = R * mPosR_KNEE + t;
    mPosL_FOOT = R * mPosL_FOOT + t;
    mPosR_FOOT = R * mPosR_FOOT + t;

    mPosROOT = R * mPosROOT + t;

    //TODO
    //update rotations as well??
}


void Body::rotateRoot(const glm::dmat3x3 &R)
{
    mPosHEAD = R * (mPosHEAD - mPosROOT) + mPosROOT;
    mPosNECK = R * (mPosNECK - mPosROOT) + mPosROOT;
    mPosL_SHOULDER = R * (mPosL_SHOULDER - mPosROOT) + mPosROOT;
    mPosR_SHOULDER = R * (mPosR_SHOULDER - mPosROOT) + mPosROOT;
    mPosL_ELBOW = R * (mPosL_ELBOW - mPosROOT) + mPosROOT;
    mPosR_ELBOW = R * (mPosR_ELBOW - mPosROOT) + mPosROOT;
    mPosL_HAND = R * (mPosL_HAND - mPosROOT) + mPosROOT;
    mPosR_HAND = R * (mPosR_HAND - mPosROOT) + mPosROOT;
    mPosTORSO = R * (mPosTORSO - mPosROOT) + mPosROOT;
    mPosL_HIP = R * (mPosL_HIP - mPosROOT) + mPosROOT;
    mPosR_HIP = R * (mPosR_HIP - mPosROOT) + mPosROOT;
    mPosL_KNEE = R * (mPosL_KNEE - mPosROOT) + mPosROOT;
    mPosR_KNEE = R * (mPosR_KNEE - mPosROOT) + mPosROOT;
    mPosL_FOOT = R * (mPosL_FOOT - mPosROOT) + mPosROOT;
    mPosR_FOOT = R * (mPosR_FOOT - mPosROOT) + mPosROOT;
}


double Body::compareTo(const Body &other)
{
    glm::dvec3 diff = glm::dvec3(0.0, 0.0, 0.0);
    diff += glm::abs(getJointAbsPosition(nite::JOINT_HEAD) - other.getJointAbsPosition(nite::JOINT_HEAD));
    diff += glm::abs(getJointAbsPosition(nite::JOINT_NECK) - other.getJointAbsPosition(nite::JOINT_NECK));
    diff += glm::abs(getJointAbsPosition(nite::JOINT_LEFT_SHOULDER) - other.getJointAbsPosition(nite::JOINT_LEFT_SHOULDER));
    diff += glm::abs(getJointAbsPosition(nite::JOINT_RIGHT_SHOULDER) - other.getJointAbsPosition(nite::JOINT_RIGHT_SHOULDER));
    diff += glm::abs(getJointAbsPosition(nite::JOINT_LEFT_ELBOW) - other.getJointAbsPosition(nite::JOINT_LEFT_ELBOW));
    diff += glm::abs(getJointAbsPosition(nite::JOINT_RIGHT_ELBOW) - other.getJointAbsPosition(nite::JOINT_RIGHT_ELBOW));
    diff += glm::abs(getJointAbsPosition(nite::JOINT_LEFT_HAND) - other.getJointAbsPosition( nite::JOINT_LEFT_HAND));
    diff += glm::abs(getJointAbsPosition(nite::JOINT_RIGHT_HAND) - other.getJointAbsPosition(nite::JOINT_RIGHT_HAND));
    diff += glm::abs(getJointAbsPosition(nite::JOINT_TORSO) - other.getJointAbsPosition(nite::JOINT_TORSO));
    diff += glm::abs(getJointAbsPosition(nite::JOINT_LEFT_HIP) - other.getJointAbsPosition(nite::JOINT_LEFT_HIP));
    diff += glm::abs(getJointAbsPosition(nite::JOINT_RIGHT_HIP) - other.getJointAbsPosition(nite::JOINT_RIGHT_HIP));
    diff += glm::abs(getJointAbsPosition(nite::JOINT_LEFT_KNEE) - other.getJointAbsPosition(nite::JOINT_LEFT_KNEE));
    diff += glm::abs(getJointAbsPosition(nite::JOINT_RIGHT_KNEE) - other.getJointAbsPosition(nite::JOINT_RIGHT_KNEE));
    diff += glm::abs(getJointAbsPosition(nite::JOINT_LEFT_FOOT) - other.getJointAbsPosition(nite::JOINT_LEFT_FOOT));
    diff += glm::abs(getJointAbsPosition(nite::JOINT_RIGHT_FOOT) - other.getJointAbsPosition(nite::JOINT_RIGHT_FOOT));

    return glm::length(diff);
}


void Body::getFloorPlane(nite::Plane &plane)
{
    plane = nite::Plane(nite::Point3f(mFloorPlane.point.x, mFloorPlane.point.y, mFloorPlane.point.z),
                        nite::Point3f(mFloorPlane.normal.x, mFloorPlane.normal.y, mFloorPlane.normal.z));
}


void Body::getFloorPlane(Plane &plane)
{
    plane = mFloorPlane;
}


void Body::setFloorPlane(nite::Plane &plane)
{
    mFloorPlane.point = {plane.point.x, plane.point.y, plane.point.z};
    mFloorPlane.normal = {plane.normal.x, plane.normal.y, plane.normal.z};
}


void Body::setFloorPlane(Plane &plane)
{
    mFloorPlane = plane;
}


glm::dvec3 Body::getJointAbsPosition(const nite::JointType tJointID) const
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
            return glm::dvec3(0.0, 0.0, 0.0);
    }
}

glm::dvec3 Body::getJointRelPosition(const nite::JointType tJointID) const
{
    return getJointAbsPosition(tJointID) - mPosROOT;
}

glm::dvec3 Body::getRootPosition() const
{
    return mPosROOT;
}


glm::dvec4 Body::getJointOrientation(const nite::JointType tJointID) const
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
            return glm::dvec4(0.0, 0.0, 0.0, 0.0);
    }
}


glm::dvec3 Body::getForward() const
{
    return mForward;
}


glm::dvec3 Body::getRight() const
{
    return mRight;
}


glm::dvec3 Body::getUp() const
{
    return mUp;
}


uint64_t Body::getTimeStamp() const
{
    return mTimeStamp;
}


void Body::calcRootOrientations()
{
    mPosROOT = (mPosL_HIP + mPosR_HIP) * 0.5;

    glm::dvec3 tShoulder, tHip;
    tShoulder = mPosL_SHOULDER - mPosR_SHOULDER;
    tHip = mPosL_HIP - mPosR_HIP;
    mRight = tHip + tShoulder;
    mRight = glm::normalize(mRight);
    //up direction is relative to the ground plane
    mUp = mFloorPlane.normal;
    mForward = glm::cross(mRight, mUp);
}


void Body::setTimeStamp(uint64_t time)
{
    mTimeStamp = time;
}


void Body::setJointPositions(const nite::Skeleton &tSkeleton)
{
    mPosL_HIP = toGlmVec3(tSkeleton.getJoint(nite::JOINT_LEFT_HIP).getPosition());
    mPosR_HIP = toGlmVec3(tSkeleton.getJoint(nite::JOINT_RIGHT_HIP).getPosition());

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


void Body::setJointPosition(const glm::dvec3 &pos, const int i)
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


void Body::setJointOrientation(const glm::dvec4 &quat, const int i)
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


