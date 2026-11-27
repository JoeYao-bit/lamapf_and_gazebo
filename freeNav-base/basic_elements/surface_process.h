//
// Created by yaozhuo on 2022/1/6.
//

#ifndef FREENAV_BASE_SURFACE_PROCESS_H
#define FREENAV_BASE_SURFACE_PROCESS_H

#include <memory>
#include <map>
#include "sys/time.h"
#include <iomanip>
#include <functional>

#include "../basic_elements/point.h"

#define CROWDED_MAP 1

namespace freeNav {

    template <Dimension N>
    struct Grid;

    template <Dimension N> using GridPtr = std::shared_ptr<Grid<N> >;

    template <Dimension N> using GridPtrs = std::vector<GridPtr<N> >;

    template <Dimension N> using CandidateNodeConstraints = std::vector<bool (*)(const Grid<N>& grid)>;

    template<Dimension N>
    struct Grid {

        Grid(const Id& id, const IdSet& nearby_ids,
             const Pointi<N>& pt, const Pointis<N>& nearby_pts,
             const Pointis<N>& nearby_obsts_, bool is_occupied)
                : id_(id), nearby_surface_ids_(nearby_ids),
                pt_(pt), nearby_surface_pts_(nearby_pts),
                nearby_obst_pts_(nearby_obsts_), is_occupied_(is_occupied) {}

        Grid(const Id& id, const Pointi<N>& pt, bool is_occupied) : id_(id), pt_(pt), is_occupied_(is_occupied) {}

        Grid() {}

        bool is_node_candidate_ = false;

        Id id_; // the id of current grid

        IdSet nearby_surface_ids_ = {}; // the id of surface grid in nearby 3^n-1 grid of current grid

        Pointi<N> pt_;

        Pointis<N> nearby_surface_pts_; // grid on surface grid in nearby 3^n-1 grid of current grid

        Pointis<N> nearby_obst_pts_; // the obstacle grid in nearby 3^n-1 grid of current grid

        Pointis<N> nearby_obst_offsets_; // the obstacle grid in nearby 3^n-1 grid of current grid

        bool is_occupied_ = false;

        NodeId node_id_ = MAX<NodeId>; // id in node vector, MAX<NodeId> mean its not node

    };

    // cross locally means there is both collide and free connection from pt1 to sg2's nearby
    template <Dimension N>
    bool LineCrossObstacleLocal(const Pointi<N>& pt1, const GridPtr<N>& sg2, IS_OCCUPIED_FUNC<N> is_occupied) {
        if(pt1 == sg2->pt_) return true;
        bool collide_flag = false, free_flag = false;
        auto dist_square = (pt1 - sg2->pt_).Square();
        int min_free_dist = MAX<int>;
        int max_occ_dist = 0;

        for(const auto &surface_grid : sg2->nearby_surface_pts_) {
            // filter surface_grid that closer to pt1 than sg2
            for (const auto &obst : sg2->nearby_obst_pts_) {
                if(pointDistToLine(obst, pt1, surface_grid) < 1 - 1e-9) {
                    collide_flag = true;
                    int temp_dist = (pt1 - surface_grid).Square();
                    if(temp_dist > max_occ_dist) max_occ_dist = temp_dist;
                }
                else {
                    free_flag = true;
                    int temp_dist = (pt1 - surface_grid).Square();
                    if(temp_dist < min_free_dist) min_free_dist = temp_dist;
                }
            }
            if(collide_flag && free_flag) return true;
            // TODO: how about get the foot of a perpendicular and check whether it in the cubic of obstacle
            //if(min_free_dist <  max_occ_dist) return true;
        }
        return false;
    }

    template <Dimension N>
    Pointis<N> GetAllGridOfSpace(DimensionLength* dimension_info) {
        Id total_index = getTotalIndexOfSpace<N>(dimension_info);
        Pointis<N> retv;
        for(Id i = 0; i < total_index; i++) {
            retv.push_back(IdToPointi<N>(i, dimension_info));
        }
        retv.shrink_to_fit();
        return retv;
    }

    typedef std::vector<IdVector> ProjectSpace;

    typedef std::vector<IdSet> RawProjectSpace;

    /* the flag used to intercept a 2D plane from N dimensional grid space  */
    typedef std::vector<int> InterceptFlag;
    typedef std::vector<InterceptFlag> InterceptFlags;

    template <Dimension N>
    class SurfaceProcessorInterface {
    public:
        explicit SurfaceProcessorInterface(DimensionLength* dimension_info,
                                           IS_OCCUPIED_FUNC<N> is_occupied,
                                           SET_OCCUPIED_FUNC<N> set_occupied
        ) {

            dimension_info_ = dimension_info;
            std::cout << " dimension_info = ";
            for(int i=0; i<N; i++) {
                std::cout << dimension_info[i] << " ";
            }
            std::cout << std::endl;
            total_index_ = getTotalIndexOfSpace<N>(dimension_info);
            //global_total_index = total_index_;
            is_occupied_ = is_occupied;
            set_occupied_ = set_occupied;
            generateDimensionInterceptFlags();
            nearby_offsets_ = GetNeightborOffsetGrids<N>();
            //minimum_surface_grid_near_obstacle_count_ = pow(3, N - 1) -1;
            max_line_length_ = 0;
            for(Id i = 0; i < N; i++) {
                max_line_length_ += pow(dimension_info[i], 2);
            }
            max_line_length_ = (Id)ceil(sqrt(this->max_line_length_));
            neighbor_ = GetNeightborOffsetGrids<N - 1>();
            floor_and_ceil_ = GetFloorOrCeilFlag<N>();
            //std::cout  << "floor_and_ceil_ " << floor_and_ceil_ << std::endl;
            //std::cout << "neighbor_ " << neighbor_ << std::endl;
        }

        virtual inline GridPtrs<N> getLocalVisibleSurfaceGrids(const Pointi<N>& pt, bool half = false) = 0;

        virtual inline GridPtrs<N> getVisibleTangentCandidates(const Pointi<N>& pt) = 0;


        virtual std::vector<Id> getLocalVisibleSurfaceGridIdsByWave(const Pointi<N>& pt) {
            if(this->is_occupied_(pt)) return {};
            // 2*N plains at any level, each level L(0,1,2,..) plain have (2*L+1)^(N-1) grids
            std::vector<std::vector<bool>> on_plain_grid_stats;
            // when N = 2
            // floor_and_ceil_ (0, 0)->(1, 0)->(0, 1)->(1, 1)
            // neighbor_ (-1)->(1)

            // when N = 3
            // floor_and_ceil_ (0, 0, 0)->(1, 0, 0)->(0, 1, 0)->(1, 1, 0)->(0, 0, 1)->(1, 0, 1)->(0, 1, 1)->(1, 1, 1)
            // neighbor_ (-1, -1)->(0, -1)->(1, -1)->(-1, 0)->(1, 0)->(-1, 1)->(0, 1)->(1, 1)

            // init plains
            on_plain_grid_stats.resize(2*N, {false});

//          if(this->is_occupied_(temp_pt)) on_rib_grid_stats[2*i] = true;

            int level = 1;
            bool need_expand = true;
            Pointi<N-1> all_1_pt;
            all_1_pt.setAll(1);
            //std::cout << " all_1_pt " << all_1_pt << std::endl;

            // get local plain
            DimensionLength local_3_plain_dimension_info[N - 1];
            for(int i=0; i<N-1; i++) {
                local_3_plain_dimension_info[i] = 3;
            }
            Pointis<N-1> id_to_pt_in_local_3_plain;
            id_to_pt_in_local_3_plain.reserve(pow(3, N - 1));
            for(int i=0; i<pow(3, N-1); i++)
            {
                id_to_pt_in_local_3_plain.push_back(IdToPointi<N - 1>(i, local_3_plain_dimension_info) - all_1_pt);
            }
            //std::cout << " id_to_pt_in_local_3_plain " << id_to_pt_in_local_3_plain << std::endl;

            // prev plain
            Pointis<N-1> id_to_pt_in_prev_plain = {Pointi<N-1>()};

            std::vector<Id> retv;
            IdSet insert_pt_ids;
            while(need_expand) {
                need_expand = false;
                DimensionLength next_plain_dimension_info[N - 1];
                for(int i=0; i<N-1; i++) {
                    next_plain_dimension_info[i] = 2 * level + 1;
                }
                // get current plain
                Pointis<N-1> id_to_pt_in_next_plain;
                id_to_pt_in_next_plain.reserve(pow(2 * level + 1, N - 1));
                for(int i=0; i<pow(2*level+1, N-1); i++)
                {
                    id_to_pt_in_next_plain.push_back(IdToPointi<N - 1>(i, next_plain_dimension_info) - level * all_1_pt);
                }
                std::vector<std::vector<bool>> next_on_plain_grid_stats(2*N, std::vector<bool>(pow(2*level+1, N-1), true));
                for(int i=0; i<2*N; i++) {
                    // expand on plain grid
                    for(int j=0; j<on_plain_grid_stats[i].size(); j++) {
                        if(on_plain_grid_stats[i][j]) continue;
                        // check whether on surface
                        Pointi<N> offset;
                        // set 2*N direction
                        if(i % 2 == 0) {
                            offset[i/2] = -level+1;
                        } else {
                            offset[i/2] = level-1;
                        }
                        // set offset inside plain
                        for(int k=0; k<i/2; k++) {
                            offset[k] = id_to_pt_in_prev_plain[j][k];
                        }
                        for(int k=i/2+1; k<N; k++) {
                            offset[k] = id_to_pt_in_prev_plain[j][k-1];
                        }
                        // considering global offset
                        Pointi<N> world_pt = offset + pt;
                        // which out of boundary is occupied
                        if(this->is_occupied_(world_pt)) {
                            continue;
                        } else {
                            need_expand = true;
                            // set project grid in next wave to free
                            for(const auto& local_offset : id_to_pt_in_local_3_plain) {
                                Pointi<N-1> grid_in_next_plain = id_to_pt_in_prev_plain[j] + local_offset + level*all_1_pt;
                                next_on_plain_grid_stats[i][PointiToId(grid_in_next_plain, next_plain_dimension_info)] = false;
                            }
                            //if(this->isOnSurface(world_pt, this->nearby_offsets_))
                            Id id = PointiToId(world_pt, this->dimension_info_);
                            if(this->grid_map_[id] != nullptr && this->grid_map_[id]->node_id_ != MAX<NodeId>)
                            {
                                //avoid replicate caused by overlap of plain
                                if(level != 1 && insert_pt_ids.find(id) == insert_pt_ids.end()
                                   && !LineCrossObstacle(world_pt, pt, this->is_occupied_, this->neighbor_)
                                        ) { // avoid insert start
                                    retv.push_back(id);
                                    insert_pt_ids.insert(id);
                                }
                            }
                        }
                    }
                    //break;
                }
                level ++;
                std::swap(on_plain_grid_stats, next_on_plain_grid_stats);
                std::swap(id_to_pt_in_prev_plain, id_to_pt_in_next_plain);
                //if(level >= 7) break;
            }
            //std::cout << " get " << retv.size() << " surface grid " << std::endl;
            return retv;
        }


        virtual GridPtrs<N> getLocalVisibleSurfaceGridsByWave(const Pointi<N>& pt) {
            if(this->is_occupied_(pt)) return {};
            auto retv_id = this->getLocalVisibleSurfaceGridIdsByWave(pt);
            GridPtrs<N> retv;
            for(const Id & id : retv_id) {
                retv.push_back(this->grid_map_[id]);
            }
            return retv;
        }

        /* use convolution kernel to inflate the surface, and update surface grids and candidates of tangent point  */
        virtual void surfaceInflation(const Pointis<N>& convolution_kernel) {
            for(const auto& pt : this->surface_grids_) {
                for(const auto& offset: convolution_kernel) {
                    this->set_occupied_(pt->pt_ + offset);
                }
            }
            this->surfaceGridsDetection();
            this->tangentCandidateDetect();
        }

        virtual std::vector<GridPtr<N> > getTangentCandidates() {
            return tangent_candidates_;
        }

        virtual std::vector<GridPtr<N> > getSurfaceGrids() {
            return surface_grids_;
        }

        virtual Pointis<N> getSurfacePts() {
            return surface_pts_;
        }

        const Id & getTotalIndex() const {
            return total_index_;
        }

        //  whether the grid has the potential to be a tangent node
        bool isDiagonalOccupied(const Pointi<N>& pt) {
            Pointi<N> current = pt;
            Pointi<N> pre     = pt;
            Pointi<N> next    = pt;
            for(const auto& flag : this->flags_) {
                for (int i=0; i<8; i++) {
                    current = pt;
                    pre = pt;
                    next = pt;
                    current[flag[0]] += this->CNOS[i][0];
                    current[flag[1]] += this->CNOS[i][1];
                    if(this->is_occupied_(current)) {
                        pre[flag[0]]  += this->CNOS[(i+7)%8][0];
                        pre[flag[1]]  += this->CNOS[(i+7)%8][1];
                        next[flag[0]] += this->CNOS[(i+1)%8][0];
                        next[flag[1]] += this->CNOS[(i+1)%8][1];
                        // i % 2 = 0 means only which has diagonal obstacle are tangent point candidate
                        // flag1 means there must be obstacle grid in the diagonal direction
                        // i % 2 = 0 is much safer and looking good but may introduce extra path when search distinctive topology path
                        // i % 2 = 1 is more intense but not very safe
                        if(i%2 == 0 &&
                           !this->is_occupied_(pre) && !this->is_occupied_(next)) {
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        virtual inline bool isTangentPointCandidate(const GridPtr<N>& surface_grid) {
            Pointi<N> current = surface_grid->pt_;
            Pointi<N> pre     = surface_grid->pt_;
            Pointi<N> next    = surface_grid->pt_;
            bool flag1 = false, flag2 = false;
            // a flag corresponding to a plane of current point, if any plane existing a alone occupied grid, the point is a candidate
            for(const auto& flag : this->flags_) {
                for (int i=0; i<8; i++) {
                    current = surface_grid->pt_;
                    pre = surface_grid->pt_;
                    next = surface_grid->pt_;
                    current[flag[0]] += this->CNOS[i][0];
                    current[flag[1]] += this->CNOS[i][1];
                    if(this->is_occupied_(current)) {
                        pre[flag[0]]  += this->CNOS[(i+7)%8][0];
                        pre[flag[1]]  += this->CNOS[(i+7)%8][1];
                        next[flag[0]] += this->CNOS[(i+1)%8][0];
                        next[flag[1]] += this->CNOS[(i+1)%8][1];
                        // i % 2 = 0 means only which has diagonal obstacle are tangent point candidate
                        // flag1 means there must be obstacle grid in the diagonal direction
                        // i % 2 = 0 is much safer and looking good but may introduce extra path when search distinctive topology path
                        // i % 2 = 1 is more intense but not very safe
                        if(i%2 == 0 &&
                           !this->is_occupied_(pre) && !this->is_occupied_(next)) {
                            Pointi<N> pre_o = current + 2*(pre - current);
                            Pointi<N> next_o = current + 2*(next - current);
                            if(this->isDiagonalOccupied(pre_o) && this->isDiagonalOccupied(next_o)) {
                                continue;
                            }
                            flag1 = true;
                        }
                    }
                }
            }
            return flag1;
        }


        DimensionLength* getDimensionInfo() {
            return dimension_info_;
        }

        const Pointis<N-1>& getNeighbor() {
            return neighbor_;
        }

        virtual PathLen distBetween(const Pointi<N>& pt1, const Pointi<N>& pt2) const {
            return (pt1-pt2).Norm();
        }

        virtual void updateNearbyGrids(const GridPtr<N>& surface_grid) {
            Pointi<N> current = surface_grid->pt_;
            Pointi<N> pre     = surface_grid->pt_;
            Pointi<N> next    = surface_grid->pt_;
            surface_grid->nearby_obst_pts_.clear();
            surface_grid->nearby_surface_pts_.clear();
            surface_grid->nearby_surface_ids_.clear();
            // a flag corresponding to a plane of current point, if any plane existing a alone occupied grid, the point is a candidate
            for(const auto& flag : this->flags_) {
                for (int i=0; i<8; i++) {
                    current = surface_grid->pt_;
                    pre = surface_grid->pt_;
                    next = surface_grid->pt_;
                    current[flag[0]] += this->CNOS[i][0];
                    current[flag[1]] += this->CNOS[i][1];
                    if(this->is_occupied_(current)) {
                        pre[flag[0]]  += this->CNOS[(i+7)%8][0];
                        pre[flag[1]]  += this->CNOS[(i+7)%8][1];
                        next[flag[0]] += this->CNOS[(i+1)%8][0];
                        next[flag[1]] += this->CNOS[(i+1)%8][1];
                        surface_grid->nearby_obst_pts_.push_back(current);
                    } else {
                        if(this->isOnSurface(current, this->nearby_offsets_)) {
                            surface_grid->nearby_surface_pts_.push_back(current);
                            surface_grid->nearby_surface_ids_.insert(PointiToId(current, this->dimension_info_));
                        }
                    }
                }
            }
            surface_grid->nearby_obst_pts_.shrink_to_fit();
            surface_grid->nearby_surface_pts_.shrink_to_fit();
            //surface_grid->nearby_surface_ids_.shrink_to_fit();
        }

        /* detect surface */
        // whether considering free grids near boundary as grid on surface
        virtual void surfaceGridsDetection(bool considering_bound = false) = 0;

        /* detect tangent candidate, which fill nearby surface grid */
        void tangentCandidateDetect() {
            //std::cout << " start " << __FUNCTION__ << std::endl;
            std::vector<GridPtr<N>> result;
            for(const auto& pt : this->surface_grids_) {
                if(this->isTangentPointCandidate(pt)) {
                    result.push_back(pt);
                    pt->is_node_candidate_ = true;
                }
            }
            this->tangent_candidates_ = result;
            this->tangent_candidates_.shrink_to_fit();
            //std::cout << " tangent_candidates_.size() " << this->tangent_candidates_.size() << std::endl;
        }

        // whether the grid has the potential to be a tangent node
        bool isCornerNode(const Pointi<N>& pt) {
            Pointi<N> current = pt;
            Pointi<N> pre     = pt;
            Pointi<N> next    = pt;
            for(const auto& flag : this->flags_) {
                for (int i=0; i<8; i++) {
                    current = pt;
                    pre = pt;
                    next = pt;
                    current[flag[0]] += this->CNOS[i][0];
                    current[flag[1]] += this->CNOS[i][1];
                    if(this->is_occupied_(current)) {
                        pre[flag[0]]  += this->CNOS[(i+7)%8][0];
                        pre[flag[1]]  += this->CNOS[(i+7)%8][1];
                        next[flag[0]] += this->CNOS[(i+1)%8][0];
                        next[flag[1]] += this->CNOS[(i+1)%8][1];
                        // i % 2 = 0 means only which has diagonal obstacle are tangent point candidate
                        // flag1 means there must be obstacle grid in the diagonal direction
                        // i % 2 = 0 is much safer and looking good but may introduce extra path when search distinctive topology path
                        // i % 2 = 1 is more intense but not very safe
                        if(i%2 == 0 && this->is_occupied_(pre) && this->is_occupied_(next)) {
                            Pointi<N> pre_o = current + 2*(pre - current);
                            Pointi<N> next_o = current + 2*(next - current);
                            if(this->is_occupied_(pre_o) && this->is_occupied_(next_o)) {
                                return true;
                            }
                        }
                    }
                }
            }
            return false;
        }

        /* detect tangent candidate, which fill nearby surface grid */
        std::vector<GridPtr<N>> cornerNodeCandidateDetect() {
            //std::cout << " start " << __FUNCTION__ << std::endl;
            std::vector<GridPtr<N>> result;
            for(const auto& grid_ptr : this->surface_grids_) {
                if(this->isTangentPointCandidate(grid_ptr) || this->isCornerNode(grid_ptr->pt_)
                ) {
                    result.push_back(grid_ptr);
                }
            }
            //std::cout << " corner grid  size() " << result.size() << std::endl;
            return result;
        }

        void generateDimensionInterceptFlags() {
            InterceptFlags result;
            for(int i=0; i<N; i++) {
                for(int j=i+1; j<N; j++) {
                    InterceptFlag flag = {i,j};
                    result.push_back(flag);
                }
            }
            this->flags_ = result;
            this->flags_.shrink_to_fit();
        }

        virtual bool isOnSurface(const Pointi<N>& pt, const Pointis<N>& nearby_offset) {
            if(this->is_occupied_(pt)) { return false; }
            bool existing_free = false;
            bool existing_occupied = false;
            for(const auto& offset : nearby_offset) {
                if(this->is_occupied_(pt + offset)) { existing_occupied = true; }
                else { existing_free = true; }
                if(existing_free & existing_occupied) {
                    return true;
                }
            }
            return false;
        }

        virtual bool lineCrossObstacle(const Pointi<N>& pt1, const Pointi<N>& pt2) {
            return LineCrossObstacle(pt1, pt2, this->is_occupied_, this->neighbor_);
        }

        // call this after constructor to ensure call override function
        virtual void initialize() {
            this->surfaceGridsDetection();
            this->tangentCandidateDetect();
        }

        /* continues nearby grid offset, in a 2D surface, used to determine whether a surface grid is tangent */
        const int CNOS[8][2] = {{-1,-1},{-1,0},{-1,1},{0,1},{1,1},{1,0},{1,-1},{0,-1}};

        /* the upbound of maximum visible radius */
        Id max_line_length_ = 0;

        Pointis<N-1> neighbor_;

        Pointis<N> floor_and_ceil_; // Quadrant Flag

        DimensionLength* dimension_info_;

        std::vector<GridPtr<N>> tangent_candidates_;

        std::vector<GridPtr<N>> surface_grids_;

        Pointis<N> surface_pts_;

        Pointis<N> nearby_offsets_;

        Id total_index_;

        IS_OCCUPIED_FUNC<N> is_occupied_;

        SET_OCCUPIED_FUNC<N> set_occupied_;

        InterceptFlags flags_;

        GridPtrs<N> grid_map_;

        int free_grid_count_ = 0;

        int on_surface_grid_count_ = 0;

    };

    template <Dimension N>
    class SurfaceProcessor : public SurfaceProcessorInterface<N> {

    public:

        explicit SurfaceProcessor(DimensionLength* dimension_info,
                                  IS_OCCUPIED_FUNC<N> is_occupied,
                                  SET_OCCUPIED_FUNC<N> set_occupied
                                  ) : SurfaceProcessorInterface<N>(dimension_info, is_occupied, set_occupied) {
            is_occupied_map_ = std::vector<int>(this->total_index_, -2);
            this->grid_map_ = std::vector<GridPtr<N>>(this->total_index_, nullptr);
        }

        /* detect surface */
        virtual void surfaceGridsDetection(bool considering_bound = false) {
            if(!this->surface_grids_.empty()) {
                return;
            }
            //std::cout << " start " << __FUNCTION__ << std::endl;
            this->surface_grids_.clear();
            this->free_grid_count_ = this->total_index_;
            this->on_surface_grid_count_ = 0;
            for(Id i=0; i<this->total_index_; i++) {
                if(this->grid_map_[i] != nullptr) continue;
                Pointi<N> pt = IdToPointi<N>(i, this->dimension_info_);
                if(this->isOnSurface(pt, this->nearby_offsets_)) {
                    this->on_surface_grid_count_ ++;
                    const auto& surface_grid_ptr = this->grid_map_[i] == nullptr ?  std::make_shared<Grid<N> >(i, pt, false) : this->grid_map_[i];
                    this->updateNearbyGrids(surface_grid_ptr);
                    // only keep tangent node
//                    if(!this->isTangentPointCandidate(surface_grid_ptr)) {
//                        //std::cout << " continued " << std::endl;
//                        continue;
//                    }
                    this->grid_map_[i] = surface_grid_ptr;
                    this->surface_grids_.push_back(surface_grid_ptr);
                    this->surface_pts_.push_back(surface_grid_ptr->pt_);
                } else if(this->is_occupied_(pt)) {
                    //this->grid_map_[i] = std::make_shared<Grid<N> >(i, pt, true);
                    this->free_grid_count_ --;
                } else {
                    //this->grid_map_[i] = std::make_shared<Grid<N> >(i, pt, false);
                }
            }
//            for(const auto& ptr : this->grid_map_) {
//                if(ptr == nullptr) {
//                    std::cout << " grid_map contain nullptr, exit" << std::endl;
//                    exit(0);
//                }
//            }
            //std::cout << " this->free_grid_count_ = " << this->free_grid_count_ << std::endl;
            //std::cout << " surface_grids_ size = " << this->on_surface_grid_count_ << std::endl;
            //std::cout << " node candidate size = " << this->surface_grids_.size() << std::endl;

        }

        /*
         * check all visible surface grids from pt, but got some extra surface grid, due to the limitation of grid map
         * TODO: need accelerate, slow than ENL-SVG
         *       difficult to accelerate to fast as ENL-SVG
         * */
        virtual GridPtrs<N> getLocalVisibleSurfaceGrids(const Pointi<N>& pt, bool half = false) {
            return this->getLocalVisibleSurfaceGridsByWave(pt);
        }


        virtual GridPtrs<N> getVisibleTangentCandidates(const Pointi<N>& pt) {
            //GridPtrs<N> surface_grids = getLocalVisibleSurfaceGrids(pt);
            GridPtrs<N> surface_grids = this->getLocalVisibleSurfaceGridsByWave(pt);
            GridPtrs<N> retv;
            for(const auto& surface_grid : surface_grids) {
                // use pre-calculated value to avoid online tangent check
                if(this->grid_map_[surface_grid->id_] != nullptr && this->grid_map_[surface_grid->id_]->node_id_ != MAX<NodeId>
                //&& !LineCrossObstacle(surface_grid->pt_, pt, this->is_occupied_, this->neighbor_)
                ) {
//                    if(//this->isTangentPointCandidate(surface_grid) &&
//                    !LineCrossObstacle(surface_grid->pt_, pt, this->is_occupied_, this->neighbor_)
//                    )
                    {
                        retv.push_back(this->grid_map_[surface_grid->id_]);
                    }
                }
            }
            retv.shrink_to_fit();
            return retv;
        }

        const GridPtrs<N> & gridMap() const {
            return this->grid_map_;
        }

    //private:

        // the minimum requirement of tangent candidate check
        //int minimum_surface_grid_near_obstacle_count_;

        // 01.03 TODO: replace project space with finite grid search
        //ProjectSpace project_space_; // step project space

        // temporary used map for fast line of sight check
        std::vector<int> is_occupied_map_; // -2: unvisited, -1: in queue but unvisited, 0: free, 1: occupied

        //GridPtrs<N> grid_map_; // store grid state, and replace IdToPointi

        friend class SurfaceProcess_LineScanner;
        //friend class SurfaceProcess_ENLSVG_LineScanner;

        struct timeval tvafter, tvpre;
        struct timezone tz;

    };

    template <Dimension N>
    using SurfaceProcessorPtr = std::shared_ptr<SurfaceProcessorInterface<N> >;

    /* determine the shortest connection from the start set (point, circle, polygon or multiple version of them, etc) to a tangent point */
    // the shortest connection between to point is the connect between the two points
    template<Dimension N>
    GridPtr<N> findClosestFromSetToPoint(const GridPtr<N>& start_set, const Pointi<N>& point) {
        return start_set;
    }

    //Grid findClosestFromSetToPoint(const Circle& start_set, const Pointi& point);

    //Grid findClosestFromSetToPoint(const Sphere& start_set, const Pointi& point);

    //Grid findClosestFromSetToPoint(const Polygon& start_set, const Pointi& point);

    // for sparse space, we can make it faster
    template <Dimension N>
    class SurfaceProcessorSparse : public SurfaceProcessorInterface<N> {

    public:

        explicit SurfaceProcessorSparse(DimensionLength* dimension_info,
                                  IS_OCCUPIED_FUNC<N> is_occupied,
                                  SET_OCCUPIED_FUNC<N> set_occupied,
                                  const std::vector<Pointi<N> >& occ_grids,
                                  const IdSet& occ_voxel_ids
        ) : SurfaceProcessorInterface<N>(dimension_info, is_occupied, set_occupied) {
            is_occupied_map_ = std::vector<int>(this->total_index_, -2);
            this->grid_map_ = std::vector<GridPtr<N>>(this->total_index_, nullptr);
            occ_voxels_ = occ_grids;
            occ_voxel_ids_ = occ_voxel_ids;
            surfaceGridsDetection();
            this->tangentCandidateDetect();
        }

        /* detect surface */
        void surfaceGridsDetection(bool considering_bound = false) override {
//            if(!this->surface_grids_.empty()) {
//                return;
//            }
            this->grid_map_ = std::vector<GridPtr<N>>(this->total_index_, nullptr);
            //std::cout << " start sparse " << __FUNCTION__ << ", considering_bound = " << considering_bound << std::endl;
            std::vector<GridPtr<N>> result;
            Pointi<N> pt;
            if(considering_bound) {
                result.reserve(occ_voxels_.size() + getTheCountOfBoundaryGrid<N>(this->dimension_info_));
            } else {
                result.reserve(occ_voxels_.size());
            }
            this->surface_pts_.clear();
            for(const auto& occ_grid : occ_voxels_) {
                for(const auto& offset : this->nearby_offsets_) {
                    pt = occ_grid + offset;
                    Id id = PointiToId(pt, this->dimension_info_);
                    if(this->isOnSurface(pt, this->nearby_offsets_)) {
                        if(this->grid_map_[id] != nullptr) continue;
                        const auto& surface_grid_ptr = std::make_shared<Grid<N> >(id, pt, false);
                        this->updateNearbyGrids(surface_grid_ptr);
                        //if(!this->isTangentPointCandidate(surface_grid_ptr)) continue;
                        this->grid_map_[id] = surface_grid_ptr;
                        result.push_back(surface_grid_ptr);
                        this->surface_pts_.push_back(pt);
                    }
                }
            }
            //std::cout << " surface grid size = " << result.size() << std::endl;
            // check point near boundary
            if(considering_bound) {
                // traversal all boundary surface of the space
                for(int dim=0; dim < 2*N; dim++) {
                    // get each plain
                    DimensionLength plain[N-1];
                    for(Dimension i=0; i<dim/2; i++) {
                        plain[i] = this->dimension_info_[i];
                    }
                    for(Dimension i=dim/2+1; i<N; i++) {
                        plain[i-1] = this->dimension_info_[i];
                    }
                    Id total_plain_index = getTotalIndexOfSpace<N-1>(plain);
                    Pointi<N-1> plain_pt;
                    Pointi<N> origin_of_plain;
                    origin_of_plain[dim/2] = (dim%2 == 0 ? 0 : this->dimension_info_[dim/2]-1);
                    Pointi<N> new_plain_pt;
                    Id new_plain_id;
                    for(Id id=0; id<total_plain_index; id++) {
                        plain_pt = IdToPointi<N-1>(id, plain);
                        new_plain_pt = origin_of_plain.addPlainOffset(plain_pt, dim);
                        if(this->isOnSurface(new_plain_pt, this->nearby_offsets_))
                        {
                            new_plain_id = PointiToId(new_plain_pt, this->dimension_info_);
                            if(this->grid_map_[new_plain_id] != nullptr) continue;
                            const auto& surface_grid_ptr = std::make_shared<Grid<N> >(new_plain_id, new_plain_pt, false);
                            this->updateNearbyGrids(surface_grid_ptr);
                            this->grid_map_[new_plain_id] = surface_grid_ptr;
                            result.push_back(surface_grid_ptr);
                            this->surface_pts_.push_back(new_plain_pt);
                        }
                    }
                }
            }
            std::swap(this->surface_grids_, result);
            this->surface_grids_.shrink_to_fit();
            //std::cout << "sparse surface_grids_ size = " << this->surface_grids_.size() << std::endl;
        }

        /*
         * check all visible surface grids from pt, but got some extra surface grid, due to the limitation of grid map
         * TODO: need accelerate, slow than ENL-SVG
         *       difficult to accelerate to fast as ENL-SVG
         * */
        virtual GridPtrs<N> getLocalVisibleSurfaceGrids(const Pointi<N>& pt, bool half = false) {
            //return this->getLocalVisibleSurfaceGridsByWave(pt);
            GridPtrs<N> retv;
            const auto& candidates = this->tangent_candidates_;//this->surface_grids_;
            Id id = PointiToId(pt, this->dimension_info_);
            for(int j=0; j<candidates.size(); j++) {
                if(half && id > candidates[j]->id_) continue;
                if(!LineCrossObstacle(pt, candidates[j]->pt_, this->is_occupied_, this->neighbor_)) {
                    retv.push_back(candidates[j]);
                }
            }
            return retv;
        }

        virtual GridPtrs<N> getVisibleTangentCandidates(const Pointi<N>& pt) {
            GridPtrs<N> retv;
            const auto& candidates = this->tangent_candidates_;//this->surface_grids_;
            for(const auto& surface_grid : candidates) {
                // use pre-calculated value to avoid online tangent check
                if(this->grid_map_[surface_grid->id_] != nullptr
                        //this->grid_map_[surface_grid->id_]->road_map_node_ptr_ != nullptr
                        ) {
                    if(!LineCrossObstacle(surface_grid->pt_, pt, this->is_occupied_, this->neighbor_)
                            ) {
                        retv.push_back(this->grid_map_[surface_grid->id_]);
                    }
                }
            }
            retv.shrink_to_fit();
            return retv;
        }

        const GridPtrs<N> & gridMap() const {
            return this->grid_map_;
        }

        const Pointis<3>& getOccVoxels() {
            return occ_voxels_;
        }

    //private:

        // the minimum requirement of tangent candidate check
        //int minimum_surface_grid_near_obstacle_count_;

        // 01.03 TODO: replace project space with finite grid search
        ProjectSpace project_space_; // step project space

        // temporary used map for fast line of sight check
        std::vector<int> is_occupied_map_; // -2: unvisited, -1: in queue but unvisited, 0: free, 1: occupied

        //GridPtrs<N> grid_map_; // store grid state, and replace IdToPointi

        Pointis<N> occ_voxels_;

        IdSet occ_voxel_ids_;

        friend class SurfaceProcess_LineScanner;
        //friend class SurfaceProcess_ENLSVG_LineScanner;

        struct timeval tvafter, tvpre;
        struct timezone tz;

    };


}
#endif //FREENAV_SURFACE_PROCESS_H

