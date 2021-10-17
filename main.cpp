#include <iostream>
#include "headers/unicam/UnicamDevProvider.h"
#include "headers/RealsenseProvider.h"
#include "headers/cameraControl/CamController.h"
#include "headers/SaveFrame.h"

using namespace std;
using namespace cv;

int main()
{
    Mat currentDepthFrame;

    //Initialize all necessary objects
    UnicamDevProvider* realsense = new RealsenseProvider();
    realsense->initializeCameras();

    UnicamCamera* camera = realsense->getCameraByTag("input camera tag here");  //connects to the camera
    CamController* control = new CamController("/dev/ttyACM0", camera, realsense);

    //updates distance and measure values
    control->updateDist();

    //align camera
    control->realignDevice(currentDepthFrame);

    //initialize SaveFrame object
    SaveFrame* save = new SaveFrame(control, realsense, camera);
    save->saveFrames(currentDepthFrame);
}
