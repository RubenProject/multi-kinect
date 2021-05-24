#include "common.h"


uint64_t getTimeNow()
{
    using namespace std::chrono;
    milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    return ms.count();
}


bool isRotationMatrix(const cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());
    std::cout << cv::norm(I, shouldBeIdentity) << std::endl;
    return cv::norm(I, shouldBeIdentity) < 1e-6;
}
 

cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat &R)
{
    isRotationMatrix(R);
    
    float sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                         R.at<double>(1, 0) * R.at<double>(1, 0));
                         
    bool singular = sy < 1e-6;
    
    float x, y, z;
    if (!singular) {
        x = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        x = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y , z);
}
 

void composeTransform(const cv::Mat &rvec, const cv::Mat &tvec, cv::Mat &T)
{
    cv::Mat rmat;
    T = cv::Mat(4, 4, CV_64F);
    cv::Rodrigues(rvec, rmat);

    T.at<double>(0, 0) = rmat.at<double>(0, 0);              
    T.at<double>(1, 0) = rmat.at<double>(1, 0);              
    T.at<double>(2, 0) = rmat.at<double>(2, 0);              
    T.at<double>(0, 1) = rmat.at<double>(0, 1);              
    T.at<double>(1, 1) = rmat.at<double>(1, 1);              
    T.at<double>(2, 1) = rmat.at<double>(2, 1);
    T.at<double>(0, 2) = rmat.at<double>(0, 2);
    T.at<double>(1, 2) = rmat.at<double>(1, 2);
    T.at<double>(2, 2) = rmat.at<double>(2, 2);
 
    T.at<double>(0, 3) = tvec.at<double>(0);
    T.at<double>(1, 3) = tvec.at<double>(1);
    T.at<double>(2, 3) = tvec.at<double>(2);

    T.at<double>(3, 3) = 1;
}


void decomposeTransform(const cv::Mat &T, cv::Mat &rvec, cv::Mat &tvec)
{
    cv::Mat rmat;
    rmat = cv::Mat(3, 3, CV_64F);
    tvec = cv::Mat(3, 1, CV_64F);

    rmat.at<double>(0, 0) = T.at<double>(0, 0);
    rmat.at<double>(1, 0) = T.at<double>(1, 0);              
    rmat.at<double>(2, 0) = T.at<double>(2, 0);              
    rmat.at<double>(0, 1) = T.at<double>(0, 1);              
    rmat.at<double>(1, 1) = T.at<double>(1, 1);              
    rmat.at<double>(2, 1) = T.at<double>(2, 1);
    rmat.at<double>(0, 2) = T.at<double>(0, 2);
    rmat.at<double>(1, 2) = T.at<double>(1, 2);
    rmat.at<double>(2, 2) = T.at<double>(2, 2);
    cv::Rodrigues(rmat, rvec);

    tvec.at<double>(0) = T.at<double>(0, 3);
    tvec.at<double>(1) = T.at<double>(1, 3);
    tvec.at<double>(2) = T.at<double>(2, 3);
}


void invertPose(cv::Mat &rvec, cv::Mat &tvec)
{
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    rmat = rmat.t();
    tvec = -rmat * tvec;
    cv::Rodrigues(rmat, rvec);
}


void transformPoints(std::vector<cv::Point3f> &points, const cv::Mat &rvec, const cv::Mat &tvec)
{
    for (auto &point : points){
        transformPoint(point, rvec, tvec);
    }
}


void transformPoint(cv::Point3f &point, const cv::Mat &rvec, const cv::Mat &tvec)
{
    cv::Mat rmat, p;
    p = cv::Mat(cv::Point3d(point));
    cv::Rodrigues(rvec, rmat);
    p = rmat * p;
    p = p + tvec;
    point = cv::Point3f(p.at<double>(0), p.at<double>(1), p.at<double>(2));
}


void fromCameraToWorldCoordinates(const cv::Mat &rmat, const cv::Mat &tvec, cv::Mat &p)
{
    p = rmat * p;
    p = p + tvec;
}


void fromWorldToCameraCoordinates(const cv::Mat &rmat, const cv::Mat &tvec, cv::Mat &p)
{
    p = p - tvec;
    p = rmat.inv() * p;
}


//CV 2 GLM
 
void fromCV2GLM(const cv::Mat &cvmat, glm::dmat3 *glmmat)
{
    assert(cvmat.cols == 3 && cvmat.rows == 3 && cvmat.type() == CV_64FC1);
    memcpy(glm::value_ptr(*glmmat), cvmat.data, 9 * sizeof(double));
    *glmmat = glm::transpose(*glmmat);
}
 
void fromCV2GLM(const cv::Mat &cvmat, glm::dvec3 *glmvec)
{
    assert(cvmat.cols == 1 && cvmat.rows == 3 && cvmat.type() == CV_64FC1);
    memcpy(glm::value_ptr(*glmvec), cvmat.data, 3 * sizeof(double));
}
 
void fromCV2GLM(const cv::Mat &cvmat, glm::dvec4 *glmvec)
{
    assert(cvmat.cols == 1 && cvmat.rows == 4 && cvmat.type() == CV_64FC1);
    memcpy(glm::value_ptr(*glmvec), cvmat.data, 4 * sizeof(double));
}
 
//GLM 2 CV
 
void fromGLM2CV(const glm::dmat3 &glmmat, cv::Mat *cvmat) 
{
    assert(cvmat->cols == 3 || cvmat->rows == 3);
    memcpy(cvmat->data, glm::value_ptr(glmmat), 9 * sizeof(double));
    *cvmat = cvmat->t();
}
 
void fromGLM2CV(const glm::dvec3 &glmvec, cv::Mat *cvmat) 
{
    assert(cvmat->cols == 1 || cvmat->rows == 3);
    memcpy(cvmat->data, glm::value_ptr(glmvec), 3 * sizeof(double));
    *cvmat = cvmat->t();
}
 
void fromGLM2CV(const glm::dvec4 &glmvec, cv::Mat *cvmat) 
{
    assert(cvmat->cols == 1 || cvmat->rows == 4);
    memcpy(cvmat->data, glm::value_ptr(glmvec), 4 * sizeof(double));
    *cvmat = cvmat->t();
}
     
// NITE 2 GLM
 
void fromNITE2GLM(const nite::Point3f &nitevec, glm::dvec3 &glmvec)
{
   glmvec = glm::dvec3(nitevec.x, nitevec.y, nitevec.z);
}
 
void fromNITE2GLM(const nite::Quaternion &nitequat, glm::dvec4 &glmvec)
{
    glmvec = glm::dvec4(nitequat.w, nitequat.x, nitequat.y, nitequat.z);
}

// printing

std::ostream& operator<<(std::ostream &os, const glm::mat3 &mat)
{
    os << "[ " << mat[0][0] << " " << mat[0][1] << " " << mat[0][2] << " ]" << std::endl;
    os << "[ " << mat[1][0] << " " << mat[1][1] << " " << mat[1][2] << " ]" << std::endl;
    os << "[ " << mat[2][0] << " " << mat[2][1] << " " << mat[2][2] << " ]" << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream &os, const glm::vec3 &vec)
{
    os << "[ " << vec[0] << " " << vec[1] << " " << vec[2] << " ]" << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream &os, const glm::vec4 &vec)
{
    os << "[ " << vec[0] << " " << vec[1] << " " << vec[2] << " " << vec[3] << " ]" << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream &os, const nite::Point3f &vec)
{
    os << "[ " << vec.x << " " << vec.y << " " << vec.z << " ]" << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream &os, const nite::Quaternion &vec)
{
    os << "[ " << vec.w << " " << vec.x << " " << vec.y << " " << vec.z << " ]" << std::endl;
    return os;
}

