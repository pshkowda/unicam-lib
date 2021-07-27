//
// Created by rob-ot on 22.11.19.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "../headers/frameSaver//CameraFrameSaver.h"
#include "../headers/async_buf.h"

#include <sys/stat.h>

template<int I>
struct CvType {
};
template<>
struct CvType<CV_16U> {
    typedef unsigned short type_t;
};

//returns false until all the files have been persisted to storage, true if frame write is complete
bool CameraFrameSaver::addNewFrameToBuffer(cv::Mat data) {
        frame_data dataM(data);  //todo: fix the timestamp issue
        frameDataList.emplace_back(dataM);
        return false;
}

bool CameraFrameSaver::persistMatrixToFile(cv::Mat data, int index, std::string base_path) {
    std::cout << "persisting frame = " << nSavedFrames << std::endl;

   // const  char *folderPath = base_path + std::sprintf( distanceTarget);
    mkdir("/home/robot/Documents/unicam-lib-master/1500",S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);   //creates new specific folder

    std::string fileName =
            base_path + std::to_string(distanceTarget) + "/D435i_" + std::to_string(index) + ".yaml";
    std::cout << "saving to file: " << fileName << std::endl;
    async_buf sbuf(fileName);
    std::ostream astream(&sbuf);

    std::time_t result = std::time(nullptr);

    astream << "timestamp: " << std::asctime(std::localtime(&result)) << '\n';

    astream << "matrix: " << '\n';
    astream << "rows: " << data.rows << '\n';
    astream << "cols: " << data.cols << '\n';
    astream << "dt: " << "f" << '\n';
    astream << "data: " << data << '\n' << std::flush;
    return true;
}

std::list<frame_data> CameraFrameSaver::getFrameDataList() {
    return frameDataList;
}
