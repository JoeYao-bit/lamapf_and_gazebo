//
// Created by yaozhuo on 2023/7/22.
//

#ifndef FREENAV_BASE_RANDOM_MAP_GENERATOR_H
#define FREENAV_BASE_RANDOM_MAP_GENERATOR_H

#include "../basic_elements/point.h"

namespace freeNav {

    // add random number of cubic into the space
    template <Dimension N>
    struct RandomMapGenerator {
    public:

        // cubic size: half width of cubic
        // file_path to save
        explicit RandomMapGenerator(DimensionLength* dimen,
                                    Id cubic_half_width,
                                    Id cubic_number,
                                    const std::string& file_path = "",
                                    bool force_update = false,
                                    bool no_overlap = false) {
            dimen_ = dimen;
            cubic_number_ = cubic_number;
            cubic_half_width_ = cubic_half_width;
            file_path_ = file_path;
            cubic_dimen_ = new DimensionLength[N];
            for(int dim=0; dim<N; dim++) {
                cubic_dimen_[dim] = 2*cubic_half_width;
            }
            cubic_pts_ = getAllGridInsideASpace<N>(cubic_dimen_);
            Id total_index = getTotalIndexOfSpace<N>(dimen);
            grid_map_ = std::vector<bool>(total_index, false);
            no_overlap_ = no_overlap;
            if(force_update || !deserialize()) {
                std::cout << "load random pt from " << file_path_ << " failed, try to initialize random pts" << std::endl;
                createMapRandomCubicCenter();
                //updateMapFromCubicCenter();
                if(!serialize()) {
                    std::cout << " serialize failed, file_path = " << file_path_ << std::endl;
                }
            } else {
                std::cout << "load random pt from " << file_path_ << " success" << std::endl;
            }
        }

        ~RandomMapGenerator() {
            if(cubic_dimen_) {
                delete[] cubic_dimen_;
            }
        }

        bool serialize() {
            // save 1, dimen_; 2, all random cubic center pt ids
            if(file_path_ == "") return false;
            std::ofstream vis_bin(file_path_, std::ios_base::out|std::ios_base::binary|std::ios_base::trunc);
            if(vis_bin.fail()) return false;
            NodeId end = MAX<NodeId>;
            std::cout << "-- save random map to binary file " << file_path_ << std::endl;
            int count = 0;
            // 1, save dimension info
            for(int dim=0; dim<N; dim++) {
                vis_bin.write((char *) &dimen_[dim], sizeof(Id));
            }
            // 2, save cubic_half_width_
            vis_bin.write((char *) &cubic_half_width_, sizeof(Id));
            // 3, save number of cubic
            vis_bin.write((char *) &cubic_number_, sizeof(Id));
            // 4, all random cubic center pt ids
            for(const auto& pt : cubic_center_pts_) {
                Id id = PointiToId(pt, dimen_);
                vis_bin.write((char *) &id, sizeof(Id));
            }
            vis_bin.close();
            std::cout << "-- save " << cubic_center_pts_.size() << " cubic center pts" << std::endl;
            return true;
        }

        bool deserialize() {
            if(file_path_ == "") return false;
            std::ifstream vis_bin(file_path_, std::ios_base::in | std::ios_base::binary);
            if(vis_bin.fail()) return false;
            std::cout << "-- load random pts from binary file " << file_path_ << std::endl;
            // 1, check whether is the same space, in dimension
            Id dimension_length;
            for(int dim=0; dim<N; dim++) {
                vis_bin.read((char*)&dimension_length, sizeof (Id));
                if(dimen_[dim] != dimension_length) {
                    std::cout << "-- unmatch dimension length in dim " << dimension_length << "!=" << dimen_[dim] << std::endl;
                    return false;
                }
            }
            // 2, check whether is the same cubic_half_width_
            Id cubic_half_width;
            vis_bin.read((char*)&cubic_half_width, sizeof (Id));
            if(cubic_half_width != cubic_half_width_) {
                std::cout << "-- unmatched cubic_half_width " << cubic_half_width << " != " << cubic_half_width_ << std::endl;
                return false;
            }
            // 3, check whether is the same cubic_number_
            Id cubic_number;
            vis_bin.read((char*)&cubic_number, sizeof (Id));
            if(cubic_number != cubic_number_) {
                std::cout << "-- unmatched cubic number " << cubic_number << " != " << cubic_number << std::endl;
                return false;
            }
            // 4, load random points from id
            Id pt_id; Pointi<N> pt;
            cubic_center_pts_.clear();
            while(vis_bin.read((char*)&pt_id, sizeof (Id))) {
                pt = IdToPointi<N>(pt_id, dimen_);
                cubic_center_pts_.push_back(pt);
            }
            vis_bin.close();
            updateMapFromCubicCenter();
            std::cout << "-- load " << cubic_number_ << " random cubic center pts" << std::endl;
            if(cubic_number_ != cubic_center_pts_.size()) {
                std::cerr << "-- unmatch random pt size " << cubic_center_pts_.size() << " != " << cubic_number_ << std::endl;
            }
            return true;
        }

        void createMapRandomCubicCenter() {
            Pointi<N> random_pt;
            Id random_id;
            srand((unsigned)time(NULL));
            cubic_center_pts_.clear();
            Id total_index = getTotalIndexOfSpace<N>(dimen_);
            int count_of_sample = 0;
            while(1) {
                count_of_sample ++;
                if(count_of_sample>= 10*cubic_number_) {
                    std::cout << " run out of " << 10*cubic_number_ <<  " samples but only found " << cubic_center_pts_.size() << "blocks <" << cubic_number_ << std::endl;
                }
                // generate center of cubic
                random_id = (Id)(total_index*rand() / double(RAND_MAX));
                if(grid_map_[random_id]) { continue; }
                random_pt = IdToPointi<N>(random_id, dimen_);
                for(int i=0; i<N; i++) {
                    random_pt[i] = std::min(random_pt[i], (int)(dimen_[i]-5-2*cubic_half_width_));
                    random_pt[i] = std::max((int)(2*cubic_half_width_ + 5), random_pt[i]);
                }
                if(!setFromCubic(random_pt)) {
                    // if do not allow overlap
                    continue;
                } else {
                    cubic_center_pts_.push_back(random_pt);
                    if(cubic_center_pts_.size() >= cubic_number_) break;
                }
            }
        }

        bool setFromCubic(const Pointi<N>& center_pt) {
            Pointi<N> pt; Id id;
            std::vector<Id> oids;
            for(const auto& offset : cubic_pts_) {
                pt = center_pt + offset;
                if(isOutOfBoundary(pt, dimen_)) { continue; }
                id = PointiToId(pt, dimen_);
                if(grid_map_[id]) {
                    if(no_overlap_) return false;
                    continue;
                }
                oids.push_back(id);
            }
            for(const auto& oid : oids) {
                grid_map_[oid] = true;
                pt = IdToPointi<N>(id, dimen_);
                occ_grids_.push_back(pt);
                occ_ids_.insert(oid);
            }
            return true;
        }

        void updateMapFromCubicCenter() {
            Id total_index = getTotalIndexOfSpace<N>(dimen_);
            grid_map_ = std::vector<bool>(total_index, false);
            occ_grids_.clear();
            occ_ids_.clear();
            Pointi<N> pt; Id id;
            for(const auto& random_pt : cubic_center_pts_) {
                // insert surface pt in boundary to occ_grids and occ_ids
                for(const auto& offset : cubic_pts_) {
                    pt = random_pt + offset;
                    if(isOutOfBoundary(pt, dimen_)) { continue; }
                    id = PointiToId(pt, dimen_);
                    if(grid_map_[id]) { continue; }
                    grid_map_[id] = true;
                    occ_grids_.push_back(pt);
                    occ_ids_.insert(id);
                }
            }
        }

    //private:
        Id cubic_number_;

        Id cubic_half_width_;

        DimensionLength* cubic_dimen_;

        DimensionLength* dimen_;

        Pointis<N> cubic_center_pts_;

        Pointis<N> cubic_pts_;

        std::vector<bool> grid_map_;

        Pointis<N> occ_grids_;

        IdSet occ_ids_;

        std::string file_path_;

        bool no_overlap_;

    };

}

#endif //FREENAV_RANDOM_MAP_GENERATOR_H
