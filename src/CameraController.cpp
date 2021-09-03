//
// Created by rob-ot on 22.11.19.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "../headers/cameraControl/CameraController.h"
#include "../headers/async_buf.h"

template<int I>
struct CvType {
};
template<>
struct CvType<CV_16U> {
    typedef unsigned short type_t;
};

CameraController::CameraController(const char *arduinoPort, UnicamCamera *camera,
                                                         UnicamDeviceProvider *realsenseCameraProvider) : cameraControl(camera),
                                                                                        realsenseCameraProvider(realsenseCameraProvider) {
    this->arduinoPort = arduinoPort;
    arduinoSerial = fopen(arduinoPort, "w");
    cv::waitKey(500);
    if (arduinoSerial) {
        std::cout << "Setting up default angle." << std::endl;
        fprintf(arduinoSerial, "%d:%d\n", 93, 150);
        cv::waitKey(1000);
    }
    cv::waitKey(2500);
}

//returns true if the frame is aligned with the current aligned depth frame
bool CameraController::realignDevice(cv::Mat &alignedDepthFrame) {

    for (int baseAngle = 88; baseAngle <= 98; baseAngle++) {
        for (int topAngle = 150; topAngle <= 158; topAngle++) {
            realsenseCameraProvider->spinOnce();
            cv::Mat currentDepthImage = cameraControl->getDepthFrame();
            alignedDepthFrame = currentDepthImage;          //assign the current depth frame
            int disp_vert, disp_hz;
            computeDisparity(currentDepthImage, &disp_hz, &disp_vert);


            bool aligned = isAligned(currentDepthImage);
            cv::waitKey(100);
            if (aligned) {
                return true;
            } else {
                std::cout << "aligning... updating angles :: top angle = " << topAngle << " bottom angle " << baseAngle
                          << std::endl;
                cv::waitKey(200);
                if (arduinoSerial) {
                    int data = fprintf(arduinoSerial, "%d:%d\n", baseAngle, topAngle);
                    std::cout << "bytes printed = " << data << std::endl;
                    cv::waitKey(200);
                }
            }
        }
    }
    return false;
}

//gets distance from center of current frame
bool CameraController::isAtExpectedDistance(cv::Mat depthFrame) {
    int distance = getDistanceFromCenterOfSurface(depthFrame);
    std::cout<<"current distance = "<<distance<<" target = "<<distanceTarget<<std::endl;
    return abs(distance - distanceTarget) <= DISTANCE_ERROR_THRESHOLD;
}

//provides the vertical and horizontal disparity
void CameraController::computeDisparity(cv::Mat &depthFrame, int *horizontalDisparity, int *verticalDisparity) {
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
double CameraController::getDistanceFromCenterOfSurface(cv::Mat &depthFrame) {
    int width = depthFrame.cols;
    int height = depthFrame.rows;
    return computeAverageDistanceInPixelRegion(height / 2, width / 2, 80, depthFrame);
}

void CameraController::updateDistanceTarget(int newTarget) {
    distanceTarget = newTarget;
}

bool CameraController::isAligned(cv::Mat depthFrame) {
    int verticalness, horizontalness;
    bool aligned;
    computeDisparity(depthFrame, &horizontalness, &verticalness);
    if (horizontalness < ORIENTATION_THRESHOLD && horizontalness > -ORIENTATION_THRESHOLD &&
        verticalness < ORIENTATION_THRESHOLD && verticalness > -ORIENTATION_THRESHOLD) {
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

float CameraController::computeAverageDistanceInPixelRegion(int centerCol, int centerRow, int sqrDim, cv::Mat depthFrame) {
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

//returns false until all the files have been persisted to storage, true if frame write is complete

bool CameraController::addNewFrameToBuffer(cv::Mat data) {
    frame_data dataM(data);
    frameDataList.emplace_back(dataM);
    return false;
}

bool CameraController::persistMatrixToFile(cv::Mat data, int index, std::string base_path) {
    std::cout << "persisting frame = " << nSavedFrames << std::endl;

    std::string fileName =
            base_path + std::to_string(distanceTarget) + "_" +std::to_string(currentDistanceMeasureCount) + "/D435i_" + std::to_string(index) + ".yaml";
    std::cout << "saving to file: " << fileName << std::endl;
    async_buf sbuf(fileName);
    std::ostream astream(&sbuf);

    std::time_t result = std::time(nullptr);

    astream << "timestamp: " << std::asctime(std::localtime(&result)) << '\n';

    astream << "matrix: " << '\n';
    astream << "rows: " << data.rows << '\n';
    astream << "cols: " << data.cols << '\n';
    astream << "dt: " << "f" << '\n';
    astream << "data: " << data << '\n' << std::flush;
    return true;
}

//new
void CameraController::updateCurrentDistanceMeasureCount(int newCount) {
    currentDistanceMeasureCount = newCount;
}

std::list<frame_data> CameraController::getFrameDataList() {
    return frameDataList;
}

CameraController::~CameraController() {
    fclose(arduinoSerial);
}
