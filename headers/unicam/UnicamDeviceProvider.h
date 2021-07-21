//
// Created by rob-ot on 13.11.19.
//

#ifndef LIBUNICAM_UNICAMDEVICEPROVIDER_H
#define LIBUNICAM_UNICAMDEVICEPROVIDER_H

#include <list>
#include <string>
#include "UnicamDevices.h"
#include "UnicamCamera.h"

class UnicamDeviceProvider {
public:

    virtual void initializeCameras() =0;
    virtual std::list<std::string> getConnectedCameraTags() = 0; //returns the serial number or a number representing the camera
    virtual UnicamCamera * getCameraByTag(std::string cameraTag) =0 ;
    UnicamDevices getCameraType() ; //Returns the type of current camera
    bool isMultSupported() ; //Returns true if multicam is supported
    virtual void spinOnce() = 0;

private:
    std::list<std::string> cameraNames;
    std::string cameraType;

};
#endif //LIBUNICAM_UNICAMDEVICEPROVIDER_H

