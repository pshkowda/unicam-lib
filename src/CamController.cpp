//
// Created by rob-ot on 22.11.19.
//

#include "../headers/cameraControl/CamController.h"
#include "../headers/async_buf.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>

template<int I> struct CvType {
};
template<> struct CvType<CV_16U> {
    typedef unsigned short type_t;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// CamController constructor. Opens Arduino serial communication.
/// \param arduinoPort USB port address of arduino
/// \param camera
/// \param realsense
CamController::CamController(const char* arduinoPort, UnicamCamera* camera, UnicamDevProvider* realsense) :camControl(camera),
                                                                                                           realsense(realsense)
{
    this->arduinoPort = arduinoPort;
    arduinoSerial = fopen(arduinoPort, "w");
    cv::waitKey(500);
    if (arduinoSerial)
    {
        std::cout << "Setting up default angle." << std::endl;
        fprintf(arduinoSerial, "%d:%d\n", 93, 150);
        cv::waitKey(1000);
    }
    cv::waitKey(2500);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Aligns the camera to the wall (using data from the frame to calculate
/// disparity and through arduino + servo motors physically moves the camera)
/// \param alignedDepthFrame Reference frame for disparity calculation
/// \return True when aligned
bool CamController::realignDevice(cv::Mat& alignedDepthFrame)
{
    // new camera align system
    bool aligned = false;
    int disp_vert, disp_hz, alignIteration = 0, baseAngle = 100, topAngle = 150;

    while (!aligned)
    {
        realsense->spinOnce();
        cv::Mat currentDepthImage = camControl->getDepthFrame();
        alignedDepthFrame = currentDepthImage; // assign the current depth frame
        computeDisparity(currentDepthImage, &disp_hz, &disp_vert);
        aligned = isAligned(currentDepthImage);
        cv::waitKey(100);
        if (aligned)
        {
            aligned = true;
        }
        else
        {
            baseAngle += 2 * disp_vert; // rotates the camera in vertical axis (by adding new
            // values to the absolute rotational angle)
            topAngle += 2 * disp_hz; // rotates the camera in horizontal axis (by adding new
            // values to the absolute rotational angle)
            alignIteration++; // to monitor iterations in the terminal
            std::cout << "Aligning the frame. Iteration:" << alignIteration << std::endl;
            cv::waitKey(200);
            if (arduinoSerial)
            {
                int data = fprintf(arduinoSerial, "%d:%d\n", baseAngle, topAngle);
                std::cout << "Printing iteration: " << alignIteration << " Angles:" << baseAngle << " & " << topAngle
                          << std::endl;
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Computes the vertical and horizontal disparity
/// \param depthFrame
/// \param horizontalDisparity
/// \param verticalDisparity
void CamController::computeDisparity(cv::Mat& depthFrame, int* horizontalDisparity, int* verticalDisparity)
{
    int sqrDim = 20;
    int width = depthFrame.cols;
    int height = depthFrame.rows;

    int midRow = height / 2;
    int midCol = width / 2;

    int boxCenterOffset = 20;

    float verticalVect = computeAverageDistanceInPixelRegion(midCol, midRow - boxCenterOffset, sqrDim,
            depthFrame) - computeAverageDistanceInPixelRegion(midCol, midRow + boxCenterOffset, sqrDim, depthFrame);

    *verticalDisparity = verticalVect;
    float horizontalVect = computeAverageDistanceInPixelRegion(midCol - boxCenterOffset, midRow, sqrDim,
            depthFrame) - computeAverageDistanceInPixelRegion(midCol + boxCenterOffset, midRow, sqrDim, depthFrame);
    *horizontalDisparity = horizontalVect;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Computes the average distance from the wall.
/// \param depthFrame
/// \return Average pixel count in the region.
double CamController::getDistanceFromCenterOfSurface(cv::Mat& depthFrame)
{
    int width = depthFrame.cols;
    int height = depthFrame.rows;
    return computeAverageDistanceInPixelRegion(height / 2, width / 2, 80, depthFrame);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Checks if camera is aligned.
/// \param depthFrame
/// \return true if aligned
bool CamController::isAligned(cv::Mat depthFrame)
{
    int verticalness, horizontalness;
    bool aligned;
    computeDisparity(depthFrame, &horizontalness, &verticalness);
    if (horizontalness < ORIENTATION_THRESHOLD && horizontalness > -ORIENTATION_THRESHOLD && verticalness < ORIENTATION_THRESHOLD && verticalness > -ORIENTATION_THRESHOLD)
    {
        aligned = true;
        std::cout << "aligned hz disparity   " << horizontalness << "  vertical disparity = " << verticalness << std::endl;

    }
    else
    {
        std::cout << "horizontal disparity   " << horizontalness << "  vertical disparity = " << verticalness << std::endl;
        aligned = false;
    }
    return aligned;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Computes average distance in pixel region.
/// \param centerCol
/// \param centerRow
/// \param sqrDim
/// \param depthFrame
/// \return
float CamController::computeAverageDistanceInPixelRegion(int centerCol, int centerRow, int sqrDim, cv::Mat depthFrame)
{
    int validPixelCount = 1;
    float regionAverage = 0;
    const int type = CV_16U;


    for (int row = (centerRow - sqrDim / 2); row <= (centerRow + (sqrDim / 2)); row++)
    {
        for (int col = (centerCol - sqrDim / 2); col < (centerCol + (sqrDim / 2)); col++)
        {
            float val = depthFrame.at<CvType<type>::type_t>(row, col);

            if (val >= CAMERA_MINIMUM)
            {
                regionAverage += val;
                validPixelCount++;
            }
        }
    }
    return regionAverage / validPixelCount;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// returns false until all the files have been persisted to storage, true if
/// frame write is complete \param data \return false
bool CamController::addNewFrameToBuffer(cv::Mat data)
{
    frame_data dataM(data);
    frameDataList.emplace_back(dataM);
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Saves the frame to file. File is a .yaml text format.
/// \param data
/// \param index
/// \param base_path
/// \return  true
bool CamController::saveMatrixToFile(cv::Mat data, int index, std::string base_path)
{
    // creates folder for the measures
    mkdir(("/home/pavel/unicam-lib/" + std::to_string(distanceTarget) + "_" + std::to_string(
            currentDistanceMeasureCount)).c_str(), 0777);

    std::cout << "Creating folder for files." << std::endl;

    std::string fileName = base_path + std::to_string(distanceTarget) + "_" + std::to_string(
            currentDistanceMeasureCount) + "/D435i_" + std::to_string(index) + ".yaml";

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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Asks user to input current measured distance and current measure count. (How many measures are currently done at single distance)
void CamController::updateDist()
{
    std::cout << "Insert the current distance: ";
    std::cin >> distanceTarget;
    std::cout << "Insert the current measure: ";
    std::cin >> currentDistanceMeasureCount;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Gets the frame data list.
/// \return frameDataList
std::list<frame_data> CamController::getFrameDataList()
{
    return frameDataList;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// CamController destructor which closes arduinoSerial communication.
CamController::~CamController()
{
    fclose(arduinoSerial);
}
