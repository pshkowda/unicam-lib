#include <iostream>
#include "headers/unicam/UnicamDeviceProvider.h"
#include "headers/RealsenseProvider.h"
#include <sys/stat.h>
#include "headers/cameraControl/CameraController.h"

int main() {

    int currentDistance = 0;
    int currentDistanceMeasureCount = 0;
    cv::Mat currentDepthFrameRef;

    //camera type and initialization
    UnicamDeviceProvider *realsenseCameraProvider = new RealsenseProvider();
    realsenseCameraProvider->initializeCameras();
    UnicamCamera *camera = realsenseCameraProvider->getCameraByTag("f1121168");  //connects to the camera
    CameraController* controller = new CameraController("/dev/ttyACM0", camera, realsenseCameraProvider);

    //to create a folder with name of the distance and measure
    std::cout << "Insert the current distance: ";
    std::cin >> currentDistance;
    std::cout << "Insert the current measure: ";
    std::cin >> currentDistanceMeasureCount;
    mkdir(("/home/pavel/unicam-lib/"+std::to_string(currentDistance)+"_"+std::to_string(currentDistanceMeasureCount)).c_str(), 0777);   //creates new specific folder

    //updates distance target to controller and frame saver
    controller->updateCurrentDistanceMeasureCount(currentDistanceMeasureCount);
    controller->updateDistanceTarget(currentDistance);

    //to align camera
    controller->realignDevice(currentDepthFrameRef);
    std::cout << "Camera should be aligned now" << endl;

    //to determine number of frames taken

    int fileCount = 0;
    int requestedFileCount = 0;
    std::cout << "Set the requested number of frames (maximum is 50): "; //to determine requested number of frames
    std::cin >> requestedFileCount;

    while (fileCount < requestedFileCount)
    {
        currentDepthFrameRef = camera->getDepthFrame(); //gets new depth frame
        controller->addNewFrameToBuffer(currentDepthFrameRef);
        realsenseCameraProvider->spinOnce(); //to get the next frame
        std::list<frame_data> frameBuffer = controller->getFrameDataList();

        std::cout<<"Waiting for 1 seconds before starting to buffer frames"<<std::endl;
        cv::waitKey(1000);

        for (frame_data dataFrame: frameBuffer)
        {
            std::cout<<"Waiting for 1 seconds"<<std::endl;
            cv::waitKey(1000);
            controller->persistMatrixToFile(dataFrame.depthFrame, fileCount,
                                                "/home/pavel/unicam-lib/");
            fileCount++;
        }
        frameBuffer.clear();    //clear the list for next measurement
    }


    std::cout<<"Ending"<<endl;
 }
