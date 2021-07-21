#include<iostream>
#include "headers/unicam/UnicamDeviceProvider.h"
#include "headers/RealsenseProvider.h"
#include "headers/frameSaver/CameraFrameSaver.h"
using namespace cv;
int main() {
    Mat currentDepthFrameRef;

    CameraFrameSaver* frameSaver = new CameraFrameSaver();

    UnicamDeviceProvider *realsenseCameraProvider = new RealsenseProvider();
    realsenseCameraProvider->initializeCameras();

    UnicamCamera *camera = realsenseCameraProvider->getCameraByTag("902512070480");
    currentDepthFrameRef = camera->getDepthFrame();
    frameSaver->addNewFrameToBuffer(currentDepthFrameRef);       //proceedToNextMeasurement is true when the current measurement frames have been persisted
    //realsenseCameraProvider->spinOnce() to get the next frame

    std::list<frame_data> frameBuffer = frameSaver->getFrameDataList();

    //std::cout<<"waiting for .1 seconds before starting to buffer frames"<<std::endl;
    //cv::waitKey(100);

    int fileCount = 0;
    for (frame_data dataFrame: frameBuffer) {
        frameSaver->persistMatrixToFile(dataFrame.depthFrame, fileCount, "/home/robot/Documents/unicam-lib-master/");
        fileCount++;
    }
    frameBuffer.clear();    //clear the list for next measurement
}

//test upload

