//
// Created by yaozhuo on 2022/7/10.
//

#ifndef FREENAV_BASE_TEXT_MAP_LOADER_H
#define FREENAV_BASE_TEXT_MAP_LOADER_H

#include "../../basic_elements/point.h"
#include "../environment.h"
#include <iostream>
#include <fstream>

namespace freeNav {

    /*
     * . - passable terrain
       G - passable terrain
       @ - out of bounds
       O - out of bounds
       T - trees (unpassable)
       S - swamp (passable from regular terrain)
       W - water (traversable, but not passable from terrain)
     * */

    // for map from https://www.movingai.com/benchmarks/grids.html
    class TextMapLoader : public Grid_Loader<2> {

    public:
        /* load grid map from a picture, keep the raw picture and a writeable copy */
        explicit TextMapLoader(const std::string& file_path, bool (*f1)(const char& value));

        bool isOccupied(const Pointi<2> & pt) const override;

        void setOccupied(const Pointi<2> & pt) override;

        DimensionLength* getDimensionInfo() override {
            return dimen_;
        }

        GridMap grid_map_;

    };


}

#endif //FREENAV_TEXT_MAP_LOADER_H
