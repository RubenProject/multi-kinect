#pragma once

#include "pch.h" 

enum KINECT_DEVICES : int
{
    KINECT_ID_0 = 0,
    KINECT_ID_1 = 1,
    KINECT_COUNT = 2
};


enum KINECT_STREAM : int
{
    KINECT_COLOR_0 = 0,
    KINECT_COLOR_1 = 1,
    KINECT_DEPTH_0 = 2,
    KINECT_DEPTH_1 = 3,
    KINECT_STREAM_COUNT = 4
};


uint64_t getTimeNow();

bool isRotationMatrix(const cv::Mat &R);

cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat &R);

void composeTransform(const cv::Mat &rvec, const cv::Mat &tvec, cv::Mat &T);
void decomposeTransform(const cv::Mat &T, cv::Mat &rvec, cv::Mat &tvec);


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
            if (!node.IsSequence() || node.size() != 3) {
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
            if (!node.IsSequence() || node.size() != 4) {
                return false;
            }
            rhs.x = node[0].as<double>();
            rhs.y = node[1].as<double>();
            rhs.z = node[2].as<double>();                                                                                           
            rhs.w = node[3].as<double>();
            return true;
        }
    };

    template<>
    struct convert<glm::dmat3> {
        static Node encode (const glm::dmat3& rhs) {
            Node node;
            node.push_back(rhs[0][0]);
            node.push_back(rhs[0][1]);
            node.push_back(rhs[0][2]);

            node.push_back(rhs[1][0]);
            node.push_back(rhs[1][1]);
            node.push_back(rhs[1][2]);

            node.push_back(rhs[2][0]);
            node.push_back(rhs[2][1]);
            node.push_back(rhs[2][2]);
            return node;
        }

        static bool decode(const Node& node, glm::dmat3& rhs) {
            if (!node.IsSequence() || node.size() != 9) {
                return false;
            }
            rhs[0][0] = node[0].as<double>();
            rhs[0][1] = node[1].as<double>();
            rhs[0][2] = node[2].as<double>();

            rhs[1][0] = node[3].as<double>();
            rhs[1][1] = node[4].as<double>();
            rhs[1][2] = node[5].as<double>();

            rhs[2][0] = node[6].as<double>();
            rhs[2][1] = node[7].as<double>();
            rhs[2][2] = node[8].as<double>();
            return true;
        }
    };
}

//CV 2 GLM

void fromCV2GLM(const cv::Mat &cvmat, glm::dmat3 *glmmat);

void fromCV2GLM(const cv::Mat &cvmat, glm::dvec3 *glmvec);

void fromCV2GLM(const cv::Mat &cvmat, glm::dvec4 *glmvec);

//GLM 2 CV

void fromGLM2CV(const glm::dmat3 &glmmat, cv::Mat *cvmat);

void fromGLM2CV(const glm::dvec3 &glmvec, cv::Mat *cvmat);

void fromGLM2CV(const glm::dvec4 &glmvec, cv::Mat *cvmat);
    
// NITE 2 GLM

void fromNITE2GLM(const nite::Point3f &nitevec, glm::dvec3 &glmvec);

void fromNITE2GLM(const nite::Quaternion &nitequat, glm::dvec4 &glmvec);

// printing

std::ostream& operator<<(std::ostream &os, const glm::mat3 &mat);

std::ostream& operator<<(std::ostream &os, const glm::vec3 &vec);

std::ostream& operator<<(std::ostream &os, const glm::vec4 &vec);

std::ostream& operator<<(std::ostream &os, const nite::Point3f &vec);

std::ostream& operator<<(std::ostream &os, const nite::Quaternion &vec);

