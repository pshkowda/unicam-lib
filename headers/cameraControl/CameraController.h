//
// Created by rob-ot on 22.11.19.
//

#ifndef LIBUNICAM_CAMERACONTROLLER_H
#define LIBUNICAM_CAMERACONTROLLER_H
#define CAMERA_MINIMUM 400
#define DISTANCE_ERROR_THRESHOLD 10

#define ORIENTATION_THRESHOLD 7

#include <string>
#include <opencv2/core/mat.hpp>
#include <fstream>
#include "../unicam/UnicamCamera.h"
#include "../unicam/UnicamDeviceProvider.h"
#include "../frame_data.h"

class CameraController {
public:

    void updateDistanceTarget(int newTarget);
    explicit CameraController(const char *arduinoPort , UnicamCamera *camera, UnicamDeviceProvider *xtion);
    bool realignDevice(cv::Mat &alignedDepthFrame);     //returns true if aligned, parameter is the reference to the current depth frame
    bool isAtExpectedDistance(cv::Mat currentMatrix);
    //from framesaver
    bool addNewFrameToBuffer(cv::Mat data);
    bool persistMatrixToFile(cv::Mat data, int index, std::string base_path);
    void updateCurrentDistanceMeasureCount(int newCount); //new

    std::list<frame_data> getFrameDataList();

    ~CameraController();

private:

    int nSavedFrames = 50;
    int currentDistanceMeasureCount = 0; //new
    int distanceTarget = 1000;
    std::string arduinoPort;
    FILE *arduinoSerial;
    UnicamCamera* cameraControl;
    UnicamDeviceProvider *zedCameraProvider;
    std::list<frame_data> frameDataList;

    bool isAligned(cv::Mat depthFrame);
    double getDistanceFromCenterOfSurface(cv::Mat &depthFrame);
    static float computeAverageDistanceInPixelRegion(int centerCol, int centerRow, int sqrDim, cv::Mat depthFrame);
    void computeDisparity(cv::Mat &depthFrame, int *horizontalDisparity, int *verticalDisparity);

};


#endif //LIBUNICAM_CAMERACONTROLLER_H
