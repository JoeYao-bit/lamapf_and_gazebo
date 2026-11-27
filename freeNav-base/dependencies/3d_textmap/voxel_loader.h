//
// Created by yaozhuo on 2022/11/26.
//

#ifndef FREENAV_BASE_VOXEL_LOADER_H
#define FREENAV_BASE_VOXEL_LOADER_H

#include "../../basic_elements/point.h"
#include "../environment.h"
#include <iostream>
#include <fstream>
#include <limits>

namespace freeNav {

// for map from https://www.movingai.com/benchmarks/voxels.html
    class TextMapLoader_3D : public Grid_Loader<3> {

    public:

        TextMapLoader_3D(const std::string &file_path, int shrink_level = 1);

        virtual bool isOccupied(const Pointi<3> &pt) const;

        virtual void setOccupied(const Pointi<3> &pt);

        virtual DimensionLength *getDimensionInfo() override {
            return dimen_;
        }

        void detectSurface();

        Pointis<3> occ_voxels_;

        IdSet occ_voxel_ids_;

        Pointis<3> surface_voxels_;

        IdSet surface_voxel_ids_;

        Pointis<3> nearby_offsets_;

        //std::vector<uint8_t> example_;
        //std::vector<bool> example_;
        int shrink_level_ = 1;

        std::vector<bool> grid_map_;

        DimensionLength dimen_bound_[3];

        // GridMap grid_map_;

    };
}
#endif //FREENAV_3D_SCENE_LOADER_H
