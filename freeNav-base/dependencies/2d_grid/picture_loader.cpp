//
// Created by yaozhuo on 2021/12/2.
//

#include "picture_loader.h"

namespace freeNav {

    /* load grid map from a picture, keep the raw picture and a writeable copy */
    PictureLoader::PictureLoader(std::string file, bool (*f1)(const cv::Vec3b& color)) : Grid_Loader() {
        raw_map_ = cv::imread(file);
        if(raw_map_.cols==0 || raw_map_.rows==0) {
            std::cout << "-- error in map file!" << std::endl;
            exit(0);
        }
        dimen_[0] = raw_map_.cols; // x
        dimen_[1] = raw_map_.rows; // y
        grid_map_ = GridMap(dimen_[0]*dimen_[1], FREE);
        for(int i=0; i<dimen_[0]; i++) {
            for (int j = 0; j < dimen_[1]; j++) {
                if(f1(raw_map_.at<cv::Vec3b>(j, i))) grid_map_[j * dimen_[0] + i] = OCCUPIED;
            }
        }
    }

    bool PictureLoader::isOccupied(const Pointi<2> & pt) const {
        if(pt[0] < 0 || pt[0] >= dimen_[0] || pt[1] < 0 || pt[1] >= dimen_[1]) { return true; }
        auto index = pt[1]*dimen_[0] + pt[0];
        return grid_map_[index] != FREE;
    }

    void PictureLoader::setOccupied(const Pointi<2> & pt) {
        auto index = pt[1]*dimen_[0] + pt[0];
        if(index >= dimen_[0]*dimen_[1]) return;
        grid_map_[ pt[1]*dimen_[0] + pt[0] ] = OCCUPIED;
    }

}