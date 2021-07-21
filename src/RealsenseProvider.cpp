//
// Created by rob-ot on 13.11.19.
//

#include "../headers/RealsenseProvider.h"
#include "../headers/unicam/UnicamDeviceProvider.h"

bool UnicamDeviceProvider::isMultSupported() {
    return true;
}

int RealsenseProvider::getNumberOfDevices() {
    return provider.deviceCount();
}
RealsenseCamera RealsenseProvider::getCameraStream(const string &cameraSerial) {
    //std::cout<<"serial = "<<cameraSerial<<std::endl;
    return provider.getEnabledDevices().at(cameraSerial);
}

UnicamDevices UnicamDeviceProvider::getCameraType(){
     return UnicamDevices::REALSENSE;
}

std::list<string> RealsenseProvider::getConnectedCameraTags() {
    std::list<string> stringlist;
    stringlist.emplace_back("ere");
    return stringlist; //implement tag provider
}



