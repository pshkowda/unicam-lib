//
// Created by rob-ot on 6.2.19.
//

#ifndef REALCOLLABORATIONCAL_REALSENSEDEVICEPROVIDER_H
#define REALCOLLABORATIONCAL_REALSENSEDEVICEPROVIDER_H

#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <mutex>
#include <map>
#include <iostream>
#include "RealsenseCamera.h"

/*
 * @Class Realsense Viewer is the base layer to communicate with the realsense cameras
 *  it allows basic features like enabling multiple cameras, keep their list, polling of frames and retrieval of camera data.
 * **/
const std::string platform_camera_name = "Platform Camera";
class RealsenseDevice {

public:

    /*
     * Enables the device provided as {@Param dev}, puts it into a list of RealsenseCamera
     * **/
    void enableDevice(rs2::device dev);
    /*
     * Removes the device from the list of enabled devices
     * **/
    void removeDevices(const rs2::event_information& info);
    /*
     * Check all cameras for new frames, update their view port with the new frames
     * **/
    void pollFrames();

    /**
     * Returns the number of devices in the enabled devices list
     * */
    int deviceCount();

    /***
     * returns the list of enabled cameras
     * @return _devices, the list of current devices
     */
    std::map<std::string, RealsenseCamera>& getEnabledDevices();

    rs2::context& getContext();

private:
    std::mutex _mutex;
    std::map<std::string, RealsenseCamera> _devices;  //map with camera's serial number and view port
    rs2::context context;
};


#endif //REALCOLLABORATIONCAL_REALSENSEDEVICEPROVIDER_H
