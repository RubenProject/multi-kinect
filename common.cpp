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
    assert(isRotationMatrix(R));
    
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
 



