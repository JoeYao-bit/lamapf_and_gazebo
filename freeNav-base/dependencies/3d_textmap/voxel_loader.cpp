//
// Created by yaozhuo on 2022/11/26.
//

#include "voxel_loader.h"

using namespace freeNav;


TextMapLoader_3D::TextMapLoader_3D(const std::string &file_path, int shrink_level) {

    shrink_level_ = shrink_level;

    std::ifstream text_map(file_path);
    if(!text_map.is_open()) {
        std::cout << " FATAL: open " << file_path << " failed " << std::endl;
    }
    std::string line;
    std::vector<std::string> lines;
    while (getline(text_map, line)) {
        lines.push_back(line);
    }
    if(lines.size() <= 1) {
        std::cout << " FATAL: empty file " << std::endl;
    }
    std::cout << " map_info " << lines[0] << std::endl;
    std::istringstream map_info(lines[0]);
    std::string buffer;
    map_info >> buffer;
    map_info >> buffer;
    int x_size = atoi(buffer.c_str());
    map_info >> buffer;
    int y_size = atoi(buffer.c_str());
    map_info >> buffer;
    int z_size = atoi(buffer.c_str());

    dimen_[0] = x_size / shrink_level;
    dimen_[1] = y_size / shrink_level;
    dimen_[2] = z_size / shrink_level;

    std::cout << " dimen_ = " << dimen_[0] << "/" << dimen_[1] << "/" << dimen_[2] << std::endl;

    dimen_bound_[0] = x_size / shrink_level + 2;
    dimen_bound_[1] = y_size / shrink_level + 2;
    dimen_bound_[2] = z_size / shrink_level + 2;

    //grid_map_.resize(dimen_[0]*dimen_[1]*dimen_[2]/8 + 1, 0x00);
    grid_map_.resize(dimen_bound_[0]*dimen_bound_[1]*dimen_bound_[2], false);

    nearby_offsets_ = GetNeightborOffsetGrids<3>();

    //grid_map_.resize(x_size*y_size*z_size, GridState::FREE);

//    example_.reserve(8);
//    uint8_t level_1 = 0x01;
//    for(int i=0; i<8; i++) {
//        example_.push_back(level_1 << i);
//    }
    Pointi<3> pt_up, pt_down;
    Id id_up, id_down;
    // set boundary to occ
    // x
    pt_down[0] = 0, pt_up[0] = dimen_bound_[0] - 1;
    for(int y=0; y<dimen_bound_[1]; y++) {
        for(int z=0; z<dimen_bound_[2]; z++) {
            pt_down[1] = pt_up[1] = y;
            pt_down[2] = pt_up[2] = z;
            id_up   = PointiToId(pt_up, dimen_bound_);
            id_down = PointiToId(pt_down, dimen_bound_);
            grid_map_[id_up] = true;
            grid_map_[id_down] = true;
        }
    }
    // y
    pt_down[1] = 0, pt_up[1] = dimen_bound_[1] - 1;
    for(int x=0; x<dimen_bound_[0]; x++) {
        for(int z=0; z<dimen_bound_[2]; z++) {
            pt_down[0] = pt_up[0] = x;
            pt_down[2] = pt_up[2] = z;
            id_up   = PointiToId(pt_up, dimen_bound_);
            id_down = PointiToId(pt_down, dimen_bound_);
            grid_map_[id_up] = true;
            grid_map_[id_down] = true;
        }
    }
    // z
    pt_down[2] = 0, pt_up[2] = dimen_bound_[2] - 1;
    for(int x=0; x<dimen_bound_[0]; x++) {
        for(int y=0; y<dimen_bound_[1]; y++) {
            pt_down[0] = pt_up[0] = x;
            pt_down[1] = pt_up[1] = y;
            id_up   = PointiToId(pt_up, dimen_bound_);
            id_down = PointiToId(pt_down, dimen_bound_);
            grid_map_[id_up] = true;
            grid_map_[id_down] = true;
        }
    }
    for(int i=1; i<lines.size(); i++) {
        std::stringstream sin(lines[i]);
        Pointi<3> pt, pt_bound;
        for(int i=0; i<3; i++) {
            sin >> buffer;
            pt[i] = atoi(buffer.c_str()) / shrink_level;
            pt_bound[i] = atoi(buffer.c_str()) / shrink_level + 1;
        }

        occ_voxels_.push_back(pt);
        Id pt_id = PointiToId(pt, dimen_);
        occ_voxel_ids_.insert(pt_id);

//        uint8_t & seg = grid_map_[pt_id / 8];
//        seg = seg | example_[pt_id % 8];
        Id id_bound = PointiToId(pt_bound, dimen_bound_);
        grid_map_[id_bound] = true;

        //std::cout << pt << std::endl;
        //grid_map_[PointiToId(pt, dimen_)] = OCCUPIED;
        //if(i>100) break;

    }
    std::cout << " occ voxel size " << occ_voxels_.size() << std::endl;
    detectSurface();
}

void TextMapLoader_3D::detectSurface() {
    for(const auto& pt : occ_voxels_) {
        bool existing_free = false;
        bool existing_occupied = false;
        for(const auto& offset : nearby_offsets_) {
            if(isOccupied(pt + offset)) { existing_occupied = true; }
            else { existing_free = true; }
            if(existing_free & existing_occupied) {
                //return true;
                surface_voxels_.push_back(pt);
                surface_voxel_ids_.insert(PointiToId(pt, dimen_));
                break;
            }
        }
    }
    std::cout << " surface size " << surface_voxels_.size() << std::endl;
}


bool TextMapLoader_3D::isOccupied(const Pointi<3> &pt) const {
    if(pt[0] < 0 || pt[0] >= dimen_[0] || pt[1] < 0 || pt[1] >= dimen_[1] || pt[2] < 0 || pt[2] >= dimen_[2]) { return true; }

    //Id pt_id = pt[0] + 1 + (pt[1] + 1)*dimen_[0] + (pt[2] + 1)*dimen_[0]*dimen_[1];
    Id pt_id = pt[0] + 1 + (pt[1] + 1)*dimen_bound_[0] + (pt[2] + 1)*dimen_bound_[0]*dimen_bound_[1];

    //return occ_voxel_ids_.find(pt_id) != occ_voxel_ids_.end();
//    const uint8_t & seg = grid_map_[pt_id / 8];
//    return !(( seg & example_[pt_id % 8] ) == 0x00);
    return grid_map_[pt_id];
}

void TextMapLoader_3D::setOccupied(const Pointi<3> &pt) {
    //Id id = pt[0] + pt[1]*dimen_[0] + pt[2]*dimen_[0]*dimen_[1];
    Id id = PointiToId(pt, dimen_);
    if(occ_voxel_ids_.find(id) != occ_voxel_ids_.end()) {
        return;
    }
    occ_voxels_.push_back(pt);
    occ_voxel_ids_.insert(id);
}



































