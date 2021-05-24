#include "kinectapp.h"
#include "context.h"
#include "common.h"
#include "camera.h"
#include "pch.h"

# define pi           3.14159265358979323846  /* pi */

KinectApplication *app;
Context *context;
Logger *logger;


int main()
{
    size_t bin = 0b0001;
    std::cout << bin << std::endl;
    exit(0x0);


/*


    //cv::Mat rcmat0, rcmat1, rcmat2;
    cv::Mat rc0 = (cv::Mat_<double>(3, 1) << 0, -pi/2, 0);
    cv::Mat tc0 = (cv::Mat_<double>(3, 1) << 2, 0, 1);
    //cv::Rodrigues(rc0, rcmat0);


    cv::Mat rc1 = (cv::Mat_<double>(3, 1) << 0, pi/2, 0);
    cv::Mat tc1 = (cv::Mat_<double>(3, 1) << -10, 0, 1);
    //cv::Rodrigues(rc1, rcmat1);

    cv::Mat pc0 = (cv::Mat_<double>(3, 1) << 0, 0, 2);

    Camera C0, C1;


    C0.setExtrinsics(rc0, tc0);
    C1.setExtrinsics(rc1, tc1);

    std::cout << pc0 << std::endl;

    C0.fromCameraToWorldCoordinates(pc0);

    std::cout << pc0 << std::endl;

    C1.fromWorldToCameraCoordinates(pc0);

    std::cout << pc0 << std::endl;



    std::cout << pc0 << std::endl;

    fromCameraToWorldCoordinates(rcmat0, tc0, pc0);

    std::cout << pc0 << std::endl;

    fromWorldToCameraCoordinates(rcmat1, tc1, pc0);

    std::cout << pc0 << std::endl;

    exit(0x0);
    // C0 to world coordinate
    pc0 = rcmat0 * pc0;
    pc0 = pc0 + tc0;

    std::cout << pc0 << std::endl;

    // world coordinate to C0
    pc0 = pc0 - tc1;
    pc0 = rcmat1.inv() * pc0;

    std::cout << pc0 << std::endl;

    exit(0x0);
*/

    context = new Context();
    logger = new Logger("log.txt");
    app = new KinectApplication();

    app->initialize();

    while (!app->update()){}

    delete context;
    delete logger;
    delete app;

    return 0;
}
