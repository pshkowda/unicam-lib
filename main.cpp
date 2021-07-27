#include<iostream>
#include "headers/unicam/UnicamDeviceProvider.h"
#include "headers/RealsenseProvider.h"
#include "headers/frameSaver/CameraFrameSaver.h"

int main() {
    cv::Mat currentDepthFrameRef;

    CameraFrameSaver* frameSaver = new CameraFrameSaver();

    UnicamDeviceProvider *realsenseCameraProvider = new RealsenseProvider();
    realsenseCameraProvider->initializeCameras();

    UnicamCamera *camera = realsenseCameraProvider->getCameraByTag("902512070480");  //connects to the camera

    int fileCount = 0;
    int requestedFileCount = 0;
    std::cout << "Write the requested number of frames (maximum is 5): "; //to determine requested number of frames

    do {        //to prevent too long processes
        std::cin >> requestedFileCount;
    } while (requestedFileCount > 5); //just for test

    while (fileCount < requestedFileCount)
    {
        currentDepthFrameRef = camera->getDepthFrame();                       //gets new depth frame
        frameSaver->addNewFrameToBuffer(currentDepthFrameRef);

        //proceedToNextMeasurement is true when the current measurement frames have been persisted

        //if (proceedToNextMeasurement)
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

