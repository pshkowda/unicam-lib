//
// Created by rob-ot on 22.11.19.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "../headers/orientationControl/CameraOrientationController.h"
#include "../headers/async_buf.h"

template<int I>
struct CvType {
};
template<>
struct CvType<CV_16U> {
    typedef unsigned short type_t;
};

/*
 * Controls the camera orientation control hardware to orient the camera's normal perpendicular to the wall's surface
 * INPUT: arduinoPort to communicate with, cv::Mat depthFrame to compute orientation against
 * Steps:
 * 1. Detect the plane from center, expand.
 * 2. measure distance from centre, use the fov of the camera to get an approx translation from px to wo
 *
 *
 ***/

CameraOrientationController::CameraOrientationController(const char *arduinoPort, UnicamCamera *camera,
                                                         UnicamDeviceProvider *realsenseCameraProvider) : cameraControl(camera),
                                                                                        realsenseCameraProvider(realsenseCameraProvider) {
    this->arduinoPort = arduinoPort;
    arduinoSerial = fopen(arduinoPort, "w");
    cv::waitKey(500);
    if (arduinoSerial)
        fprintf(arduinoSerial, "%d:%d\n", 90, 150);
    cv::waitKey(2500);
}

//returns true if the frame is aligned with the current aligned depth frame
bool CameraOrientationController::realignDevice(cv::Mat &alignedDepthFrame) {

    for (int baseAngle = 150; baseAngle <= 158; baseAngle++) {
        for (int topAngle = 88; topAngle <= 98; topAngle++) {

            realsenseCameraProvider->spinOnce();
            cv::Mat currentDepthImage = cameraControl->getDepthFrame();
            alignedDepthFrame = currentDepthImage;          //assign the current depth frame

            bool aligned = isAligned(currentDepthImage);
            cv::waitKey(100);
            if (aligned) {
                return true;
            } else {
                std::cout << "aligning... updating angles :: top angle = " << topAngle << " bottom angle " << baseAngle
                          << std::endl;
                cv::waitKey(100);
                if (arduinoSerial)
                    fprintf(arduinoSerial, "%d:%d\n", baseAngle, topAngle);
                cv::waitKey(100);
            }
        }
    }
    return false;
}

//gets distance from center of current frame
bool CameraOrientationController::isAtExpectedDistance(cv::Mat depthFrame) {
    int distance = getDistanceFromCenterOfSurface(depthFrame);
    std::cout<<"current distance = "<<distance<<" target = "<<distanceTarget<<std::endl;
    return abs(distance - distanceTarget) <= DISTANCE_ERROR_THRESHOLD;
}

//provides the vertical and horizontal disparity
void CameraOrientationController::computeDisparity(cv::Mat &depthFrame, int *horizontalDisparity, int *verticalDisparity) {
    int sqrDim = 20;
    int width = depthFrame.cols;
    int height = depthFrame.rows;

    int midRow = height / 2;
    int midCol = width / 2;

    int boxCenterOffset = 20;

    float verticalVect = computeAverageDistanceInPixelRegion(midCol, midRow - boxCenterOffset, sqrDim, depthFrame) -
                         computeAverageDistanceInPixelRegion(midCol, midRow + boxCenterOffset, sqrDim, depthFrame);

    *verticalDisparity = verticalVect;
    float horizontalVect = computeAverageDistanceInPixelRegion(midCol - boxCenterOffset, midRow, sqrDim, depthFrame) -
                           computeAverageDistanceInPixelRegion(midCol + boxCenterOffset, midRow, sqrDim, depthFrame);
    *horizontalDisparity = horizontalVect;
}

//computes the average distance from the wall
double CameraOrientationController::getDistanceFromCenterOfSurface(cv::Mat &depthFrame) {
    int width = depthFrame.cols;
    int height = depthFrame.rows;
    return computeAverageDistanceInPixelRegion(height / 2, width / 2, 80, depthFrame);
}

void CameraOrientationController::updateDistanceTarget(int newTarget) {
    distanceTarget = newTarget;
}

bool CameraOrientationController::isAligned(cv::Mat depthFrame) {
    int verticalness, horizontalness;
    bool aligned;
    computeDisparity(depthFrame, &horizontalness, &verticalness);
    if (horizontalness < ORIENTATION_THRESHOLD && horizontalness > -ORIENTATION_THRESHOLD &&
        verticalness < ORIENTATION_THRESHOLD &&
        verticalness > -ORIENTATION_THRESHOLD) {
        aligned = true;
        std::cout << "aligned hz disparity   " << horizontalness << "  vertical disparity = " << verticalness
                  << std::endl;

    } else {
        std::cout << "horizontal disparity   " << horizontalness << "  vertical disparity = " << verticalness
                  << std::endl;
        aligned = false;
    }
    return aligned;
}

float CameraOrientationController::computeAverageDistanceInPixelRegion(int centerCol, int centerRow, int sqrDim, cv::Mat depthFrame) {
    auto frame_data = depthFrame.data;
    int validPixelCount = 1;
    float regionAverage = 0;
    const int type = CV_16U;

    int width = depthFrame.cols;
    int height = depthFrame.cols;


    for (int row = (centerRow - sqrDim / 2); row <= (centerRow + (sqrDim / 2)); row++) {
        for (int col = (centerCol - sqrDim / 2); col < (centerCol + (sqrDim / 2)); col++) {

            //std::cout<<(depthFrame.at<CvType<type>::type_t>(row2, col))<<" ";
            float val = depthFrame.at<CvType<type>::type_t>(row, col);

            if (val >= CAMERA_MINIMUM) {
                //regionAverage +=frame_data[col*width+row];
                regionAverage += val;
                validPixelCount++;
            }
        }
    }
    //std::cout<<regionAverage/validPixelCount<<std::endl;
    return regionAverage / validPixelCount;

}

CameraOrientationController::~CameraOrientationController() {
    fclose(arduinoSerial);
}
