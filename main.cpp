#include "kinectapp.h"
#include "context.h"
#include "common.h"


KinectApplication *app;
Context *context;
Logger *logger;


int main()
{
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
