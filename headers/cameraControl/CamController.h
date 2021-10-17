//
// Created by rob-ot on 22.11.19.
//

#ifndef LIBUNICAM_CAMCONTROLLER_H
#define LIBUNICAM_CAMCONTROLLER_H
#define CAMERA_MINIMUM 400
#define ORIENTATION_THRESHOLD 7

#include <string>
#include <opencv2/core/mat.hpp>
#include <fstream>
#include "../unicam/UnicamCamera.h"
#include "../unicam/UnicamDevProvider.h"
#include "../frame_data.h"

class CamController {
public:

    explicit CamController(const char *arduinoPort, UnicamCamera *camera, UnicamDevProvider *realsense);

    bool realignDevice(cv::Mat& alignedDepthFrame);

    void updateDist();

    bool addNewFrameToBuffer(cv::Mat data);

    bool saveMatrixToFile(cv::Mat data, int index, std::string base_path);

    std::list<frame_data> getFrameDataList();

    ~CamController();

private:

    int currentDistanceMeasureCount = 0; //new
    int distanceTarget = 1000;
    std::string arduinoPort;
    FILE* arduinoSerial;
    UnicamCamera* camControl;
    UnicamDevProvider* realsense;
    std::list<frame_data> frameDataList;

    bool isAligned(cv::Mat depthFrame);

    double getDistanceFromCenterOfSurface(cv::Mat& depthFrame);

    static float computeAverageDistanceInPixelRegion(int centerCol, int centerRow, int sqrDim, cv::Mat depthFrame);

    void computeDisparity(cv::Mat& depthFrame, int* horizontalDisparity, int* verticalDisparity);
};


#endif //LIBUNICAM_CAMCONTROLLER_H
