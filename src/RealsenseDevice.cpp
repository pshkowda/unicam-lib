//
// Created by rob-ot on 13.11.19.
//

#include "../headers/RealsenseDevice.h"


    void RealsenseDevice::enableDevice(rs2::device dev)
    {
        std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        std::lock_guard<std::mutex> lock(_mutex);

        if (_devices.find(serial_number) != _devices.end())
        {
            return; //already in
        }

        // Ignoring platform cameras (webcams, etc..)
        if (platform_camera_name == dev.get_info(RS2_CAMERA_INFO_NAME))
        {
            return;
        }
        // Create a pipeline from the given device
        rs2::pipeline p;
        rs2::config c;                                                      //configuration of the camera
       // c.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);  //choose the resolution and fps
       // c.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30); //Cannot be a random combination, please refer
        c.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);//realsense documentation for available configurations.
        //.enable_all_streams();

        c.enable_device(serial_number);
        std::cout<<serial_number<<std::endl;
        // Start the pipeline with the configuration
        try {
            rs2::pipeline_profile profile = p.start(c);     //start pipe p with camera configutation c
            _devices.emplace(serial_number, RealsenseCamera(serial_number, {}, p, profile));

        } catch (const rs2::error & e){
            std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        }

        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
        // Hold it internally
    }

    void RealsenseDevice::removeDevices(const rs2::event_information& info) {
        std::lock_guard<std::mutex> lock(_mutex);
        // Go over the list of devices and check if it was disconnected
        auto itr = _devices.begin();
        while (itr != _devices.end()) {
            if (info.was_removed(itr->second.profile.get_device())) {
                itr = _devices.erase(itr);
            } else {
                ++itr;
            }
        }
    }

    void RealsenseDevice::pollFrames()
    {
        try {
            std::lock_guard<std::mutex> lock(_mutex);
            // Go over all device
            for (auto &&view : _devices) {
                // Ask each pipeline if there are new frames available
                rs2::frameset frameset;
                if (view.second.pipe.try_wait_for_frames(&frameset)) {
                    rs2::frame inf = frameset.get_infrared_frame();
                    view.second.current_frameset = frameset; //update view port with the new frameset
                }
            }
        } catch (rs2::error &e) {
            std::cout<<"realsense frame capture failed";
        }
    }

    int RealsenseDevice::deviceCount()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _devices.size();
    }

    std::map<std::string, RealsenseCamera>& RealsenseDevice::getEnabledDevices() {
        return _devices;
    }

    rs2::context& RealsenseDevice::getContext() {
        return context;
    }


