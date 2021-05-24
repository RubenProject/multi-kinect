#include "camera.h"

#include "logger.h"



Camera::Camera()
{
    K = cv::Mat::eye(3, 3, CV_64F);
    dist = cv::Mat::zeros(8, 1, CV_64F);

    rmat = cv::Mat::eye(3, 3, CV_64F);
    tvec = cv::Mat::zeros(3, 1, CV_64F);

    imageSize = cv::Size(0, 0);

    state = 0;
}


Camera::~Camera()
{
}


bool Camera::hasIntrinsics()
{
    return state & 0b0001;
}


bool Camera::hasExtrinsics()
{
    return state & 0b0010;
}


void Camera::setIntrinsics(const cv::Mat &K, const cv::Mat &dist)
{
    this->K = K.clone();
    this->dist = dist.clone();
    state |= 0b0001; 
}


void Camera::setExtrinsics(const cv::Mat &rvec, const cv::Mat &tvec)
{
    cv::Rodrigues(rvec, rmat);
    this->tvec = tvec.clone();
    state |= 0b0010; 
}


void Camera::getCameraMatrix(cv::Mat &K)
{
    K = this->K;
}


void Camera::getDistortion(cv::Mat &dist)
{
    K = this->dist;
}


void Camera::setImageSize(int w, int h)
{
    imageSize = cv::Size(w, h);
}


bool Camera::loadCamera(const std::string &name)
{
    YAML::Node node;
    try {
        node = YAML::LoadFile(name);
    } catch (std::exception &e) {
        logger->log(libfreenect2::Logger::Info, "No camera matrix found.");
        return false;
    }

    std::vector<double> distvec;

    double fx = node["fx"].as<double>();
    double fy = node["fy"].as<double>();
    double cx = node["cx"].as<double>();
    double cy = node["cy"].as<double>();
    distvec = node["dist"].as<std::vector<double>>();

    double *data = new double[9]{fx,  0.0, cx,
                                 0.0, fy,  cy,
                                 0.0, 0.0, 1.0};

    K = cv::Mat(3, 3, CV_64F, data);
    dist = cv::Mat(distvec);
}


bool Camera::saveCamera(const std::string &name)
{
    YAML::Node node;
    node["fx"] = K.at<double>(0, 0);
    node["fy"] = K.at<double>(1, 1);
    node["cx"] = K.at<double>(0, 2);
    node["cy"] = K.at<double>(1, 2);
    for (int i = 0; i < dist.rows; i++){
        node["dist"].push_back(dist.at<double>(i, 0));
    }

    std::ofstream fout(name.c_str());
    if (!fout.is_open())
        return false;
    fout << node;
    fout.close();
    return true;
}


void Camera::fromCameraToWorldCoordinates(cv::Mat &p)
{
    p = rmat * p;
    p = p + tvec;
}


void Camera::fromWorldToCameraCoordinates(cv::Mat &p)
{
    p = p - tvec;
    p = rmat.inv() * p;
}

