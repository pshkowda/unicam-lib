#include <iostream>
#include "headers/unicam/UnicamDeviceProvider.h"
#include "headers/RealsenseProvider.h"
#include "headers/frameSaver/CameraFrameSaver.h"
#include <sys/stat.h>
#include "headers/orientationControl/CameraOrientationController.h"


int main() {
    cv::Mat currentDepthFrameRef;
    int currentDistance = 0;
    bool isAlignedNow = false;
    bool isDistanceEqualToTarget = false;

    //camera type and initialization
    CameraFrameSaver* frameSaver = new CameraFrameSaver();
    UnicamDeviceProvider *realsenseCameraProvider = new RealsenseProvider();
    realsenseCameraProvider->initializeCameras();
    UnicamCamera *camera = realsenseCameraProvider->getCameraByTag("902512070480");  //connects to the camera

    //to define camera controller
    CameraOrientationController* controller = new CameraOrientationController("/dev/ttyACM0", camera, realsenseCameraProvider);
    //to align camera
    controller->realignDevice(currentDepthFrameRef);

    //to create a folder with name od the distance
    std::cout << "Insert the current distance: ";
    std::cin >> currentDistance;
    mkdir(("/home/robot/Documents/unicam-lib-master/"+std::to_string(currentDistance)).c_str(), 0777);   //creates new specific folder

    //updates distance target to controller and frame saver
    frameSaver->updateDistanceTarget(currentDistance);
    controller->updateDistanceTarget(currentDistance);

    isAlignedNow = controller->realignDevice(currentDepthFrameRef);                            //check if device is aligned
    isDistanceEqualToTarget = controller->isAtExpectedDistance(currentDepthFrameRef); //checks if expected distance is right to measured distance

    //to determine number of frames taken
    int fileCount = 0;
    int requestedFileCount = 0;
    std::cout << "Set the requested number of frames (maximum is 50): "; //to determine requested number of frames
    do {        //to prevent too long processes
        std::cin >> requestedFileCount;
    } while (requestedFileCount > 50);

    while (fileCount < requestedFileCount)
    {
        currentDepthFrameRef = camera->getDepthFrame(); //gets new depth frame
        frameSaver->addNewFrameToBuffer(currentDepthFrameRef);
        realsenseCameraProvider->spinOnce(); //to get the next frame

        std::list<frame_data> frameBuffer = frameSaver->getFrameDataList();

        std::cout<<"waiting for 1 seconds before starting to buffer frames"<<std::endl;
        cv::waitKey(1000);

        for (frame_data dataFrame: frameBuffer)
        {
            frameSaver->persistMatrixToFile(dataFrame.depthFrame, fileCount,
                                            "/home/robot/Documents/unicam-lib-master/");
            fileCount++;
        }
        frameBuffer.clear();    //clear the list for next measurement
   }
}