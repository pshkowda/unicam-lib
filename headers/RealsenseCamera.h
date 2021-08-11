//
// Created by rob-ot on 13.11.19.
//

#ifndef LIBUNICAM_REALSENSECAMERA_H
#define LIBUNICAM_REALSENSECAMERA_H


#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include "unicam/UnicamCamera.h"

class RealsenseCamera: public UnicamCamera {
public:
    rs2::frameset current_frameset;  //Current Frameset as from the camera, is refreshed in this structure by calling pollFrames();
    rs2::pipeline pipe;
    rs2::pipeline_profile profile;


    RealsenseCamera(std::string cameraTag, rs2::frameset frameset, rs2::pipeline pipeline,
                    rs2::pipeline_profile profile) {
        current_frameset = frameset;
        this->pipe = pipeline;
        this->profile = profile;
        setSerialTag(cameraTag) ;
    }

    cv::Mat getRGBFrame() {
        rs2::video_frame colorFrame = current_frameset.get_color_frame();
        cv::Mat depth(cv::Size(640, 480), CV_8UC1, (void *) colorFrame.get_data(), cv::Mat::DEPTH_MASK); //todo: fix the encodings
        return depth;

    }
    cv::Mat getDepthFrame() {
        cv::Mat depth;
        rs2::depth_frame depthFrame = this->current_frameset.get_depth_frame();
        try {
            depth = cv::Mat(cv::Size(640, 480), CV_16UC1, (void *) depthFrame.get_data(), cv::Mat::AUTO_STEP);
        } catch (...) {

        }
        return depth;

    }
    cv::Mat getIRFrame(){
        rs2::depth_frame depthFrame = current_frameset.get_depth_frame();
        cv::Mat depth(cv::Size(640, 480), CV_8UC1, (void *) depthFrame.get_data(), cv::Mat::AUTO_STEP);
        return depth;
    }

};


#endif //LIBUNICAM_REALSENSECAMERA_H
