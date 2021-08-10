//
// Created by rob-ot on 22.11.19.
//

#ifndef LIBUNICAM_CAMERAORIENTATIONCONTROLLER_H
#define LIBUNICAM_CAMERAORIENTATIONCONTROLLER_H
#define CAMERA_MINIMUM 400
#define DISTANCE_ERROR_THRESHOLD 10

#define ORIENTATION_THRESHOLD 60

#include <string>
#include <opencv2/core/mat.hpp>
#include <fstream>
#include "../unicam/UnicamCamera.h"
#include "../unicam/UnicamDeviceProvider.h"
#include "../frame_data.h"

class CameraOrientationController {
public:

    explicit CameraOrientationController(const char *arduinoPort , UnicamCamera *camera, UnicamDeviceProvider *xtion);

    void updateDistanceTarget(int newTarget);
    bool realignDevice(cv::Mat &alignedDepthFrame);     //returns true if aligned, parameter is the reference to the current depth frame
    bool isAtExpectedDistance(cv::Mat currentMatrix);

    ~CameraOrientationController();

private:

    int nSavedFrames = 50;

    int distanceTarget = 1000;
    std::string arduinoPort;
    FILE *arduinoSerial;
    UnicamCamera* cameraControl;
    UnicamDeviceProvider *realsenseCameraProvider;
    std::list<frame_data> frameDataList;

    bool isAligned(cv::Mat depthFrame);
    double getDistanceFromCenterOfSurface(cv::Mat &depthFrame);
    float computeAverageDistanceInPixelRegion(int centerCol, int centerRow, int sqrDim, cv::Mat depthFrame);
    void computeDisparity(cv::Mat &depthFrame, int *horizontalDisparity, int *verticalDisparity);

};


#endif //LIBUNICAM_CAMERAORIENTATIONCONTROLLER_H
