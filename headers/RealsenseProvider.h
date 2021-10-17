#pragma once
//
// Created by rob-ot on 18.1.19.
//
#define DEVICE_CALIB_CAM 0
#define DEVICE_REGULAR_CAM 1
#include "RealsenseDevice.h"
#include "unicam/UnicamDevProvider.h"

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace rs2;
/*
 * {@Class RealsenseProvider} is a layer built on top of @Class RealsenseDevice and allows a simple interface to
 * initialize the cameras and get relevant data from them.
 * **/
class RealsenseProvider: public UnicamDevProvider {
public:
    void initializeCameras(){
        rs2::context ctx = provider.getContext();
        ctx.set_devices_changed_callback([&](rs2::event_information &info) {
            provider.removeDevices(info);
            for (auto &&dev : info.get_new_devices()) {
                provider.enableDevice(dev);
            }
            std::cout << "number of connected RealsenseDevices = " << getNumberOfDevices() << std::endl;
        });

        // Initial population of the device list
        for (auto &&dev : ctx.query_devices()) // Query the list of connected RealSense devices
        {
            provider.enableDevice(dev);
        }
        std::cout << "number of connected RealsenseDevices = " << getNumberOfDevices() << std::endl;

        for (int i = 0; i < 100; i++) {
            provider.pollFrames(); //exposure stablization
        }
    }

    std::list<string> getConnectedCameraTags();
    void spinOnce() {provider.pollFrames();};    //updates the frames

    RealsenseCamera getCameraStream(const string &cameraSerial);

    UnicamCamera * getCameraByTag(std::string cameraTag) {
        RealsenseCamera camera = provider.getEnabledDevices().at(cameraTag);
        return &provider.getEnabledDevices().at(cameraTag);
    }

    int getNumberOfDevices();
    ~RealsenseProvider() {}

protected:
    RealsenseDevice provider;
};