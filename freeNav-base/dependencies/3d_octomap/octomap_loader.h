//
// Created by yaozhuo on 2022/2/27.
//

#ifndef FREENAV_BASE_OCTOMAP_LOADER_H
#define FREENAV_BASE_OCTOMAP_LOADER_H

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include "../../basic_elements/point.h"
#include "../environment.h"


class OctoMapLoader {

public:
        explicit OctoMapLoader(std::string file_path_, int depth_offset_)
        {
            octo_map = new octomap::OcTree(file_path_);
            std::cout << "tree depth = " << octo_map->getTreeDepth() << std::endl;
            octo_map->getMetricMin(min_x, min_y, min_z);
            octo_map->getMetricMax(max_x, max_y, max_z);
            octo_map->coordToKeyChecked(min_x, min_y, min_z, octo_map->getTreeDepth()-depth_offset_, key_min);
            octo_map->coordToKeyChecked(max_x, max_y, max_z, octo_map->getTreeDepth()-depth_offset_, key_max);
            index_ratio = pow(2, depth_offset_);
            //dimen.resize(3);
            dimen[0] = freeNav::Id((key_max[0] - key_min[0])/index_ratio)+1; // x
            dimen[1] = freeNav::Id((key_max[1] - key_min[1])/index_ratio)+1; // y
            dimen[2] = freeNav::Id((key_max[2] - key_min[2])/index_ratio)+1; // z


            file_path = file_path_;
            depth_offset = depth_offset_;

            octo_map->getMetricMin(min_x, min_y, min_z);
            octo_map->getMetricMax(max_x, max_y, max_z);

            resolution = octo_map->getResolution();
            occ_thresh = octo_map->getOccupancyThres();
            resolution *= pow(2,depth_offset);
            depth_level = octo_map->getTreeDepth();
            octo_map->coordToKeyChecked(min_x, min_y, min_z, depth_level-depth_offset, key_min);
            octo_map->coordToKeyChecked(max_x, max_y, max_z, depth_level-depth_offset, key_max);
            octo_map->coordToKeyChecked(min_x, min_y, min_z, depth_level,              key_zero_min);
            octo_map->coordToKeyChecked(max_x, max_y, max_z, depth_level,              key_zero_max);
            index_ratio = pow(2, depth_offset);

        }

        octomap::OcTree* getOctoMap() {
            return octo_map;
        }

        octomap::point3d PointiToPoint3d(const freeNav::Pointi<3> pt) {
            octomap::point3d pt3;
            pt3.x() = (max_x-min_x)*pt[0]/dimen[0] + min_x;
            pt3.y() = (max_y-min_y)*pt[1]/dimen[1] + min_y;
            pt3.z() = (max_z-min_z)*pt[2]/dimen[2] + min_z;
            return pt3;
        }

// CAN NOT DETECT LINE CROSS
//        bool lineCrossCheck(const freeNav::RimJump::Pointi<3> pt1, const freeNav::RimJump::Pointi<3> pt2) {
//            std::vector<octomap::point3d> retv_pt3d;
//            octomap::OcTreeKey key = PointiToOcTreeKey(pt1);
//            octomap::point3d pt3_1 = PointiToPoint3d(pt1), pt3_2 = PointiToPoint3d(pt2);
//
//            return octo_map->computeRay(pt3_1, pt3_2, retv_pt3d);
//        }

    // check grid state
        bool isOccupied(const freeNav::Id& index) {
            auto it = octo_map->search(IndexToOcTreeKey(index), depth_level-depth_offset);
            if(it) {
                if(it->getOccupancy() > occ_thresh) return true;
            }
            return false;
        }

        bool isOccupied(const freeNav::Pointi<3>& pt) const {
            const auto& it = octo_map->search(PointiToOcTreeKey(pt), depth_level-depth_offset);
            if(it) {
                if(it->getOccupancy() >= occ_thresh) return true;
            }
            return false;
        }

        void setOccupied(const freeNav::Pointi<3>& pt) {
            float x = pt[0]*resolution + min_x;
            float y = pt[1]*resolution + min_y;
            float z = pt[2]*resolution + min_z;
            octo_map->updateNode(x, y, z, true);
        }


        ~OctoMapLoader() {
            std::cout << "OctoMapLoader deconstructor called" << std::endl;
            if(!octo_map) delete octo_map;
        }

        octomap::OcTreeKey IndexToZeroKey(const freeNav::Pointi<3>& pt) {
            octomap::OcTreeKey key;
            key[0] = pt[0]+key_zero_min[0];
            key[1] = pt[1]+key_zero_min[1];
            key[2] = pt[2]+key_zero_min[2];
            return key;
        }

        freeNav::Pointi<3> CoordinateToPointi(double x, double y, double z) {
            octomap::OcTreeKey key;
            octo_map->coordToKeyChecked(x, y, z, depth_level-depth_offset, key);
            freeNav::Pointi<3> pt;
            pt[0] = (key[0]-key_min[0])/index_ratio;
            pt[1] = (key[1]-key_min[1])/index_ratio;
            pt[2] = (key[2]-key_min[2])/index_ratio;
            return pt;
        }

        octomap::OcTreeKey IndexToOcTreeKey(freeNav::Id index) {
            freeNav::Pointi<3> pt = freeNav::IdToPointi<3>(index, dimen);
            return PointiToOcTreeKey(pt);
        }

        octomap::OcTreeKey PointiToOcTreeKey(const freeNav::Pointi<3>& pt) const {
            octomap::OcTreeKey key;
            key[0] = pt[0]*index_ratio+key_min[0];
            key[1] = pt[1]*index_ratio+key_min[1];
            key[2] = pt[2]*index_ratio+key_min[2];
            return key;
        }

        freeNav::DimensionLength* getDimensionInfo() {
            return dimen;
        }

        double min_x, min_y, min_z, max_x, max_y, max_z;
        int depth_offset;


private:

    double resolution;
    octomap::OcTree* octo_map;
    double occ_thresh;
    int depth_level;
    octomap::OcTreeKey key_min, key_max;
    octomap::OcTreeKey key_zero_min, key_zero_max;
    int index_ratio;

    freeNav::DimensionLength dimen[3] = {1};

    std::string file_path;
    std::string map_name;
    std::string vis_name;
};

#endif //FREENAV_OCTOMAP_LOADER_H
