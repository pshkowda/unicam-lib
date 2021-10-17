//
// Created by pavel on 17.10.21.
//

#include "SaveFrame.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

SaveFrame::SaveFrame(CamController* _control, UnicamDevProvider* _realsense, UnicamCamera* _camera)
{
    control = _control;
    realsense = _realsense;
    camera = _camera;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Asks user how many frames he want to save. Then buffers the frames and saves them to a desired location.
/// \param currentDepthFrame Current frame in camera.
void SaveFrame::saveFrames(cv::Mat currentDepthFrame)
{
    //to determine number of frames taken
    cout << "Set the requested number of frames: "; //to determine requested number of frames
    cin >> requestFileCount;

    while (fileCount <= requestFileCount)
    {
        currentDepthFrame = camera->getDepthFrame(); //gets new depth frame
        control->addNewFrameToBuffer(currentDepthFrame);
        list<frame_data> frameBuffer = control->getFrameDataList();

        for (frame_data dataFrame: frameBuffer)
        {
            cout << "Waiting for 1 seconds" << endl;
            waitKey(1000);
            control->saveMatrixToFile(dataFrame.depthFrame, fileCount, "/home/pavel/unicam-lib/");
            fileCount++;
        }
        realsense->spinOnce();  //to get the next frame
        frameBuffer.clear();    //clear the list for next measurement
    }
}

/// SaveFrame destructor.
SaveFrame::~SaveFrame()
{
}

