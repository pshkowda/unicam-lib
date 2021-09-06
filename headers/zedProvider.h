//
// Created by pavel on 03.09.21.
//

#ifndef LIBUNICAM_ZEDPROVIDER_H
#define LIBUNICAM_ZEDPROVIDER_H

#include "unicam/UnicamDeviceProvider.h"

class zedProvider:

        public UnicamDeviceProvider{

public:
    void turnCameraOn();

private:

};

#endif //LIBUNICAM_ZEDPROVIDER_H
