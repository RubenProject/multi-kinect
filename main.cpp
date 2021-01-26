#include "kinectapp.h"


int main()
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
