//
// Created by rob-ot on 13.11.19.
//

#ifndef LIBUNICAM_UNICAMCAMERA_H
#define LIBUNICAM_UNICAMCAMERA_H

#include <opencv2/core/mat.hpp>

class UnicamCamera {
public:

    void setSerialTag(std::string serialTag)
    {
        cameraTag = serialTag;
    }

    virtual cv::Mat getRGBFrame() = 0;

    virtual cv::Mat getIRFrame() = 0;

    virtual cv::Mat getDepthFrame() = 0;

protected:
    std::string cameraTag;
};

#endif //LIBUNICAM_UNICAMCAMERA_H
