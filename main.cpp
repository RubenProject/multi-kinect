#include "kinectapp.h"


int main(int argc, char *argv[])
{
    KinectApplication kApp;
    kApp.initialize();

    while (true) {
        if (kApp.update()){
            break;
        }
    }

    return 0;
}
