#include <iostream>

#include "viewer.h"
#include "devicemanager.h"
#include <opencv4/opencv2/opencv.hpp>


int main(int argc, char *argv[])
{
    DeviceManager dm;
    dm.initialize();
    while (true) {
        if (dm.update()){
            break;
        }
    }

    return 0;
}
