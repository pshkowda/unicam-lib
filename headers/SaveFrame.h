//
// Created by pavel on 17.10.21.
//
#ifndef LIBUNICAM_SAVEFRAME_H
#define LIBUNICAM_SAVEFRAME_H

#include "cameraControl/CamController.h"
#include "RealsenseProvider.h"
#include "unicam/UnicamDevProvider.h"

class SaveFrame {
public:
    explicit SaveFrame(CamController* control, UnicamDevProvider* realsense, UnicamCamera* camera);

    void saveFrames(cv::Mat currentDepthFrame);

    ~SaveFrame();

private:
    int fileCount = 0;
    int requestFileCount;
    CamController* control;
    UnicamDevProvider* realsense;
    UnicamCamera* camera;
};


#endif //LIBUNICAM_SAVEFRAME_H
