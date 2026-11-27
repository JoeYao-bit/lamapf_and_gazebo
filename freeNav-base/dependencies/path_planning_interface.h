//
// Created by yaozhuo on 2022/7/5.
//

#ifndef FREENAV_BASE_PATH_PLANNING_INTERFACE_H
#define FREENAV_BASE_PATH_PLANNING_INTERFACE_H

#include "../basic_elements/point.h"
#include <ctime>
namespace freeNav {

    typedef std::vector<double> Statistic; // a method's Statistic

    typedef std::vector<Statistic> StatisticS;  // multiple method's Statistic

    typedef std::vector<StatisticS> StatisticSS;  // multiple experiment 's multiple method's Statistic

    typedef std::string OutputStream;  // a method's output stream

    typedef std::vector<OutputStream> OutputStreamS; // multiple method's output stream

    typedef std::vector<OutputStreamS> OutputStreamSS; // multiple experiment 's multiple method's output stream

    /* p2p_planning = point to point path planning
     * start: start point of path planning
     * target: target point of path planning
     * path: resulted path
     * statistics: the value we need, path length, time cost, etc
     * return false if failed, otherwise success
     * */
    template <Dimension N, typename START_TYPE, typename TARGET_TYPE>
    using Point2PointPathPlanning = std::function<void (const Pointi<N>& start,
                                                        const TARGET_TYPE& target,
                                                        std::vector<Pointi<N> >& path,
                                                        Statistic& statistic,
                                                        OutputStream& outputStream)>;

    template <Dimension N, typename START_TYPE, typename TARGET_TYPE>
    using Point2PointPathPlannings = std::vector<Point2PointPathPlanning<N, START_TYPE, TARGET_TYPE> >;

    // a couple of start and target for multiple methods
    // return true is all pass, return false is one is failed
    template <Dimension N, typename START_TYPE, typename TARGET_TYPE>
    bool SinglePoint2PointPathPlanningTest(const START_TYPE& start,
                                           const TARGET_TYPE& target,
                                           const Point2PointPathPlanning<N, START_TYPE, TARGET_TYPE>& path_planning,
                                           StatisticS& statistics,
                                           OutputStreamS& output_streams) {
        statistics.clear();
        output_streams.clear();
        bool is_failed = false;
        Statistic statistic;
        OutputStream output_stream;
        Pointis<N> path;
        path_planning(start, target, path, statistic, output_stream);
        statistics.push_back(statistic);
        output_streams.push_back(output_stream);
        if(path.empty()) {
            std::cout << "planning from " << start << " to " <<  target << " failed " << std::endl;
            is_failed = true;
        }
        return !is_failed;
    }

    // a couple of start and target for multiple methods
    // return true is all pass, return false is one is failed
    template <Dimension N, typename START_TYPE, typename TARGET_TYPE>
    bool Point2PointPathPlanningTest(const START_TYPE& start,
                                     const TARGET_TYPE& target,
                                     const Point2PointPathPlannings<N, START_TYPE, TARGET_TYPE>& path_plannings,
                                     StatisticS& statistics,
                                     OutputStreamS& output_streams) {
        statistics.clear();
        output_streams.clear();
        bool is_failed = false;
        for(const auto& path_planning : path_plannings) {
            Statistic statistic;
            OutputStream output_stream;
            Pointis<N> path;
            path_planning(start, target, path, statistic, output_stream);
            statistics.push_back(statistic);
            output_streams.push_back(output_stream);
            if(path.empty()) {
                std::cout << "planning from " << start << " to " <<  target << " failed " << std::endl;
                is_failed = true;
            }
        }
        return !is_failed;
    }

    void appendToFile(const std::string& file_name, const std::string& content, bool prune=false);

    void printCurrentTime();

    // input: static occupancy map / current solving problem / previous path as constraints
    // output: what path was found, or empty is failed
    template<Dimension N>
    using MAPF_FUNC_TEST = std::function<void (DimensionLength*, const IS_OCCUPIED_FUNC<N> &, const Instances<N> &,
                                            Paths<N>& path, Statistic& statistic, OutputStream& outputStream)>;

    template <Dimension N>
    using MAPF_FUNC_TESTS = std::vector<MAPF_FUNC_TEST<N> >;

    // a couple of start and target for multiple methods
    // return true is all pass, return false is one is failed
    template <Dimension N>
    bool MAPFPathPlanningsTest(DimensionLength* dim, const IS_OCCUPIED_FUNC<N> & isoc, const Instances<N> & ists,
                                     const MAPF_FUNC_TESTS<N>& mapf_path_plannings,
                                     const std::string& file_path, bool append=false) {
        bool is_failed = false;
        for(int i=0; i<mapf_path_plannings.size(); i++) {
            const auto& path_planning = mapf_path_plannings[i];
            Statistic statistic;
            OutputStream output_stream;
            Paths<N> path;
            //std::cout << " ists " << ists << std::endl;
            path_planning(dim, isoc, ists, path, statistic, output_stream);
            appendToFile(file_path, output_stream, append);
            if(path.empty()) {
                std::cout << "planning uses method " << i << " failed " << std::endl;
                is_failed = true;
            }
            printCurrentTime();
        }
        return !is_failed;
    }

    template <Dimension N>
    bool SingleMapMAPFPathPlanningsTest(DimensionLength* dim,
                                        const IS_OCCUPIED_FUNC<N> & isoc,
                                        const InstancesS<N> & ists,
                                        const MAPF_FUNC_TESTS<N>& mapf_path_plannings,
                                        const std::string& file_path, bool append=false) {
        bool all_success = true;
        for(int i=0; i<ists.size(); i++) {
            const auto& ist = ists[i];
            bool success = MAPFPathPlanningsTest(dim, isoc, ist, mapf_path_plannings, file_path, append);
            std::cout << "-- finish " << i << " th instance " << std::endl;
            if(all_success) all_success = success;
        }
        std::cout << " finish " << ists.size() << " cases" << std::endl;
        return all_success;
    }

}

#endif //FREENAV_PATH_PLANNING_INTERFACE_H
