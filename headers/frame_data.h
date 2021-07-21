//
// Created by rob-ot on 2019-12-12.
//

#ifndef LIBUNICAM_FRAME_DATA_H
#define LIBUNICAM_FRAME_DATA_H


#include <opencv2/core/mat.hpp>
#include <string>

struct frame_data {
    cv::Mat depthFrame;

    frame_data(cv::Mat depth) {
        depthFrame = depth;
    }
};


#endif //LIBUNICAM_FRAME_DATA_H
