//
// Created by rob-ot on 22.11.19.
//

#ifndef LIBUNICAM_CAMERAFRAMESAVER_H
#define LIBUNICAM_CAMERAFRAMESAVER_H
#define CAMERA_MINIMUM 400
#define DISTANCE_ERROR_THRESHOLD 10

#define ORIENTATION_THRESHOLD 60

#include <string>
#include <opencv2/core/mat.hpp>
#include <fstream>
#include "../unicam/UnicamCamera.h"
#include "../unicam/UnicamDeviceProvider.h"
#include "../frame_data.h"

class CameraFrameSaver {
public:

    bool addNewFrameToBuffer(cv::Mat data);
    bool persistMatrixToFile(cv::Mat data, int index, std::string base_path);
    void updateDistanceTarget(int newTarget);

    std::list<frame_data> getFrameDataList();


private:
    int nSavedFrames = 1;
    int distanceTarget = 1500;
    std::list<frame_data> frameDataList;
};


#endif //LIBUNICAM_CAMERAFRAMESAVER_H
