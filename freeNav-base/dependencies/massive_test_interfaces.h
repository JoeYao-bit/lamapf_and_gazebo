//
// Created by yaozhuo on 2022/11/26.
//

#ifndef FREENAV_BASE_MASSIVE_TEST_INTERFACES_H
#define FREENAV_BASE_MASSIVE_TEST_INTERFACES_H
#include <istream>
#include <fstream>
#include <sys/time.h>
#include <map>
#include "../basic_elements/point.h"

#include "./path_planning_interface.h"
#include "./massive_scene_loader/ScenarioLoader2D.h"
#include "./massive_scene_loader/ScenarioLoader3D.h"
#include "./random_map_generator.h"
#include <iomanip>

#include <boost/algorithm/string.hpp>

/*
 * there is only ONE map in one scene file,
 * store start and target coordinates, and minimum result path length
 * */

namespace freeNav {

    template <Dimension N>
    using SingleMapTestConfig = std::map<std::string, std::string>;

    template <Dimension N>
    using SingleMapTestConfigs = std::vector<SingleMapTestConfig<N> >;


    template <Dimension N>
    struct RandomMapTestConfig {

        std::string name_;

        DimensionLength dim_[N];

        Id cubic_half_width_;

        Id cubic_number_;

        std::string random_file_path_;

        std::string block_file_path_;

        std::string test_data_path_;

        PathLen min_block_width_ = 3;

        int shrink_level_ = 3;

    };

    template <Dimension N>
    using RandomMapTestConfigs = std::vector<RandomMapTestConfig<N> >;


    // for 2d grid map data set from https://www.movingai.com/benchmarks/grids.html
    bool SceneTest2D(const std::string &config_file_path,
                     const Point2PointPathPlannings<2, Pointi<2>, Pointi<2>> &p2p_plan_test,
                     StatisticSS &statisticss,
                     OutputStreamSS &output_streamss,
                     int max_count_of_case = 20000);

    // for 2d grid map data set from https://www.movingai.com/benchmarks/grids.html
    bool SceneTest2DIndividual(const std::string &config_file_path,
                               const Point2PointPathPlannings<2, Pointi<2>, Pointi<2>> &p2p_plan_tests,
                               StatisticSS &statisticss,
                               OutputStreamSS &output_streamss,
                               int max_count_of_case = 20000);

    // for 3d voxel map data set from https://www.movingai.com/benchmarks/voxels.html
    bool SceneTest3D(const std::string &config_file_path,
                     const Point2PointPathPlannings<3, Pointi<3>, Pointi<3> > &p2p_plan_test,
                     StatisticSS &statisticss,
                     OutputStreamSS &output_streamss,
                     int max_count_of_case = 20000);

    // for 3d voxel map data set from https://www.movingai.com/benchmarks/voxels.html
    // random means start/target is random and uniform selected from all free grids
    template<Dimension N>
    bool SceneTest_Random(DimensionLength* dimension_info,
                            IS_OCCUPIED_FUNC<N> isoc,
                            int count_of_cases,
                            const Point2PointPathPlannings<N, Pointi<N>, Pointi<N> > &p2p_plan_test,
                            StatisticSS &statisticss,
                            OutputStreamSS &output_streamss,
                            int max_random_select = 100 // maximum times of selection to find a free grid
                            ) {

        statisticss.clear();
        output_streamss.clear();
        std::cout << "plan to do " << count_of_cases << " random start/target experiments" << std::endl;
        struct timezone tz;
        struct timeval tvpre;
        struct timeval tvafter;
        gettimeofday (&tvpre , &tz);
        // random experiment data
        Id total_index = getTotalIndexOfSpace<N>(dimension_info);
        Id temp_id;
        Pointis<N> pts;
        Pointi<N> pt;
        srand((unsigned)time(NULL));
        for (int i = 0; i < count_of_cases; i++) {
            pts.clear();
            for(int j=0; j < max_random_select; j++) {
                temp_id = (Id)(total_index*rand() / double(RAND_MAX));
                //std::cout << "temp_id " << temp_id << std::endl;
                pt = IdToPointi<N>(temp_id, dimension_info);
                if(isoc(pt)) { continue; }
                else {
                    pts.push_back(pt);
                    if(pts.size() == 2) {
                        if(pts[0] == pts[1]) {
                            pts.pop_back();
                            continue;
                        }
                        break;
                    }
                }
            }
            if(pts.size() != 2) {
                //std::cout << " index " << i << " find no free start/target pair" << std::endl;
                continue;
            }
            // do test
            StatisticS statistics;
            statisticss.reserve(count_of_cases);
            OutputStreamS output_streams;
            output_streams.reserve(count_of_cases);
            //std::cout << "index " << i  << ": " << pts[0] << "->" << pts[1] << std::endl;
            if (!Point2PointPathPlanningTest<N, Pointi<N>, Pointi<N> >(pts[0],
                                                                                                           pts[1],
                                                                                                           p2p_plan_test,
                                                                                                           statistics,
                                                                                                           output_streams)) {
                std::cout << "failed index " << i  << ": " << pts[0] << "->" << pts[1] << std::endl;
            }
//            if (i % 100 == 0) {
//                std::cout << "finish " << i << " cases " << std::endl;
//            }
            gettimeofday (&tvafter , &tz);
            double time_interval = (tvafter.tv_sec-tvpre.tv_sec)+(tvafter.tv_usec-tvpre.tv_usec)/10e6;
            if(time_interval > .1) {
                std::cout << " finish :  " << i << " cases " << std::endl;
                gettimeofday (&tvpre , &tz);
            }
            for (auto &statistic : statistics) {
                statistic.push_back((pts[0]-pts[1]).Norm());
            }
            for (auto &output_stream : output_streams) {
                std::stringstream sst;
                sst << output_stream << (pts[0]-pts[1]).Norm();
                output_stream = sst.str();
            }
            statisticss.push_back(statistics);
            output_streamss.push_back(output_streams);
            if(i%1000 == 0) {
                statisticss.shrink_to_fit();
                output_streamss.shrink_to_fit();
            }
            //break;
        }
        return true;
    }

    template<Dimension N>
    bool SingleMapTestDataAnalysis(const SingleMapTestConfig <N> &test_data, bool consider_path_count = false) {
        const auto &data_path = test_data.at("output_path");
        std::ifstream fin(data_path);
        std::string line;
        std::map<string, StatisticS> data;
        std::vector<std::pair<string, StatisticS> > data_v;
        int expected_size = 2 * N + 7;
        int count_of_line = 0;
        while (getline(fin, line)) {
            //std::cout << line << std::endl;
            std::istringstream sin(line);
            std::vector<string> fields;
            std::string field;
            while (sin.rdbuf()->in_avail() != 0) {
                sin >> field;
                fields.push_back(field);
            }
            if (!fields.empty()) {
                if (fields[0] == std::string("TYPE")) continue;
            }
            if (fields.size() != expected_size) {
                std::cout << " FATAL: field.size() " << field.size() << " != expected_size " << expected_size
                          << std::endl;
            }
            Statistic current_line;
            current_line.reserve(expected_size - 1);
            for (int i = 1; i < fields.size(); i++) {
                current_line.push_back(atof(fields[i].c_str()));
            }
            if (data.find(fields[0]) == data.end()) {
                data.insert({fields[0], {}});
            }
            auto iter = data.find(fields[0]);
            iter->second.push_back(current_line);
            count_of_line ++;
        }
        std::cout << std::setprecision(5)
                  << std::setiosflags(std::ios::left)<<std::setw(15)
                  << " type name\t"
                  << setiosflags(std::ios::left)<<std::setw(15) //设置宽度为7，left对齐方式
                  << "m_init_time\t"
                  << setiosflags(std::ios::left)<<std::setw(15)
                   << "m_search_cost\t"
                  << setiosflags(std::ios::left)<<std::setw(15)
                   << "m_total_cost \t"
                  << setiosflags(std::ios::left)<<std::setw(15)
                   << "m_length \t"
                  << setiosflags(std::ios::left)<<std::setw(15)
                   << "success_rate\t"
                  << setiosflags(std::ios::left)<<std::setw(15)
                   << "std_length\t"
                  << setiosflags(std::ios::left)<<std::setw(15)
                   << "path count\t "
                  //<< setiosflags(ios::left)<<setw(15)
                  << std::endl;
        for (const auto &iter : data) {
            double init_time_cost = 0;
            double search_time_cost = 0;
            PathLen path_length = 0;
            PathLen path_length_standard = 0;
            int path_count = 0;
            std::string type_name = iter.first;
            int success_count = 0;
            std::vector<std::string> strs;
            boost::split(strs, type_name, boost::is_any_of("_"), boost::token_compress_on);

            int required_count = 0;
            if(consider_path_count) {
                //std::cout << " type_name " << type_name << std::endl;
                assert(strs.size() == 2);
                required_count = atoi(strs[1].c_str());
                //std::cout << " required_count " << required_count << std::endl;
            }
            for (const Statistic &case_data : iter.second) {
                if(case_data[2 * N + 0] >= 100000) {
                    //std::cout  << " error path length " << case_data[2 * N + 0] << std::endl;
                    continue;
                }
                if(case_data[2 * N + 4] < required_count) {
                    // used in distinctive topology data analysis
                    continue;
                }
                success_count ++;
                path_length          += case_data[2 * N + 0];
                init_time_cost       += case_data[2 * N + 2];
                search_time_cost     += case_data[2 * N + 3];
                path_length_standard += case_data[2 * N + 5];
                path_count           += case_data[2 * N + 4];
//                std::cout << " path len         " << case_data[2 * N + 0] << std::endl;
//                std::cout << " init_time_cost   " << case_data[2 * N + 2] << std::endl;
//                std::cout << " search_time_cost " << case_data[2 * N + 3] << std::endl;
//                std::cout << " search_time_cost " << case_data[2 * N + 4] << std::endl;
            }
            if(success_count == 0) {
                std::cout << type_name << " NO success instance " << std::endl;
                continue;
            }
            //std::cout << " success_count " << success_count << " iter.second.size() " << iter.second.size() << std::endl;
            std::cout << std::setprecision(5)
                      << setiosflags(std::ios::left)<<std::setw(15) //设置宽度为7，left对齐方式
                      << type_name << " \t"
                      // << ": m_init_time  = \t"
                      << setiosflags(std::ios::left)<<std::setw(15) << init_time_cost / success_count << " \t"
                      // << ", m_search_cost = \t"
                      << setiosflags(std::ios::left)<<std::setw(15) << search_time_cost / success_count << " \t"
                      // << ", m_total_cost  = \t"
                      << setiosflags(std::ios::left)<<std::setw(15) << (init_time_cost+search_time_cost) / success_count << " \t"
                      // << ", m_length = \t"
                      << setiosflags(std::ios::left)<<std::setw(15) << path_length / success_count << " \t"
                      // << ", success_rate = \t"
                      << setiosflags(std::ios::left)<<std::setw(15) << success_count*100. / iter.second.size() << " \t"
                      // << "%" << ", std_length = \t"
                      << setiosflags(std::ios::left)<<std::setw(15) << path_length_standard / success_count << " \t"
                      // << ", path count = \t "
                      << setiosflags(std::ios::left)<<std::setw(15) << path_count / success_count << " \t";
            if(consider_path_count) {
                std::cout
                //<< ", mean_each_path_time_cost = "
                << setiosflags(std::ios::left)<<std::setw(15) << (init_time_cost+search_time_cost) / success_count / required_count;
            }
            std::cout << std::endl;
        }

        // resort and print

        return true;
    }





    template<Dimension N>
    std::vector<double> SingleMapLosTestDataAnalysis(const std::string& data_path) {
        //const auto &data_path = test_data.at("los_output_path");
        //std::cout << " map minimum block width = " << test_data.at("minimum_block_width") << std::endl;
        std::ifstream fin(data_path);
        std::string line;
        std::vector<double> time_costs;
        std::vector<double> visited_pt_counts;
        std::vector<double> visited_block_counts;
        std::vector<int> collision_free_line_counts;
        std::vector<std::pair<Pointi<N>, Pointi<N>> > checked_lines;

        int expected_size = 6 + 2*N;
        int count_of_line = 0;
        bool find_all_type = false;
        std::vector<std::string> type_names;
        while (getline(fin, line)) {
            //std::cout << line << std::endl;
            std::istringstream sin(line);
            std::vector<string> fields;
            std::string field;
            while (sin.rdbuf()->in_avail() != 0) {
                sin >> field;
                fields.push_back(field);
            }
            if (fields.size() != expected_size) {
                std::cout << " FATAL: field.size() " << field.size() << " != expected_size " << expected_size
                          << std::endl;
                return {};
            }
            if(type_names.size() == 0) {
                type_names.push_back(fields[0]);
            } else if(!find_all_type && type_names[0] != fields[0]) {
                type_names.push_back(fields[0]);
            } else if(type_names[0] == fields[0]) {
                find_all_type = true;
            }
            time_costs.push_back(atof(fields[1 + 2*N].c_str()));
            visited_pt_counts.push_back(atof(fields[2 + 2*N].c_str()));
            visited_block_counts.push_back(atof(fields[3 + 2*N].c_str()));
            collision_free_line_counts.push_back(atoi(fields[4 + 2*N].c_str()));
            Pointi<N> start, target;

            start[0] = atoi(fields[3 + 2*N].c_str());
            for(int i=0; i<N; i++) {
                start[i] = atoi(fields[1 + i].c_str());
                target[i] = atoi(fields[1 + N + i].c_str());
            }
            checked_lines.push_back(std::make_pair(start, target));
            //std::cout << "line collied ? " << fields[3 + 2*N] << std::endl;
            //std::cout << " line " << start << "->" << target << std::endl;
            count_of_line ++;
        }
        std::vector<double> each_time_cost(type_names.size(), 0);
        std::vector<double> each_visited_pt(type_names.size(), 0);
        std::vector<double> each_visited_block(type_names.size(), 0);
        std::vector<int> each_collision_free_line(type_names.size(), 0);

        for(int i=0; i<time_costs.size(); i++) {
            each_time_cost[i%type_names.size()] += time_costs[i];
        }
        for(int i=0; i<visited_pt_counts.size(); i++) {
            each_visited_pt[i%type_names.size()] += visited_pt_counts[i];
        }
        for(int i=0; i<visited_block_counts.size(); i++) {
            each_visited_block[i%type_names.size()] += visited_block_counts[i];
        }
        for(int i=0; i<collision_free_line_counts.size(); i++) {
            each_collision_free_line[i%type_names.size()] += collision_free_line_counts[i];
        }
        // check whether have the same result
        for(int i=0; i<collision_free_line_counts.size()/type_names.size(); i++) {
            auto pre_val = collision_free_line_counts[i*type_names.size()];
            for(int j=i*type_names.size()+1; j<(i+1)*type_names.size(); j++) {
                if(collision_free_line_counts[j] != pre_val) {
                    std::cout << type_names[j-i*type_names.size()] << " los check" << checked_lines[j].first << "->" << checked_lines[j].second << " not the same " << std::endl;
                }
            }
        }
        std::vector<double> ratios;
        for(int i=0; i<type_names.size(); i++) {
            std::cout << type_names[i] << " mean time cost " << type_names.size()*each_time_cost[i]/count_of_line << "\t / "
                      << "visited pt " << type_names.size()*each_visited_pt[i]/count_of_line << " / "
                      << "visited block " << type_names.size()*each_visited_block[i]/count_of_line << " / "
                      << " collide ratio " << 100.*type_names.size()*each_collision_free_line[i]/((double)count_of_line) << "%"
                      << std::endl;
            ratios.push_back(each_time_cost[i]/each_time_cost.back());
        }
        return ratios;
    }

    // for 2d grid map data set from https://www.movingai.com/benchmarks/grids.html
    template<Dimension N>
    bool SceneTest2DWithStartAndTargets(const std::vector<std::pair<Pointi<N>, Pointi<N>> >& start_and_targets,
                     const Point2PointPathPlannings<2, Pointi<2>, Pointi<2>> &p2p_plan_test,
                     StatisticSS &statisticss,
                     OutputStreamSS &output_streamss,
                     int num_of_thread = 4,
                     int max_count_of_case = 20000) {

        statisticss.clear();
        output_streamss.clear();

        if (start_and_targets.size() <= 0) return false;

        struct timezone tz;
        struct timeval tvpre;
        struct timeval tvafter;
        gettimeofday (&tvpre , &tz);

        // load experiment data
        // TODO: multiple thread
        for (int i = 0; i < start_and_targets.size(); i++) {
            //const auto &experiment = sl.GetNthExperiment(i);
            //int sx = experiment.GetGoalX(), sy = experiment.GetGoalY(), gx = experiment.GetStartX(), gy = experiment.GetStartY();
            double length = 0;//experiment.GetDistance();
            // do test
            StatisticS statistics;
            OutputStreamS output_streams;
            Pointi<N> start = start_and_targets[i].first, target = start_and_targets[i].second;
            //std::cout << "index " << i << ": " << start << "->" << target <<  ", shortest length = " << length << std::endl;
            if (!Point2PointPathPlanningTest<N, Pointi<N>, Pointi<N> >(start,
                                                                       target,
                                                                       p2p_plan_test,
                                                                       statistics,
                                                                       output_streams)) {
                std::cout << "failed index " << i << ": " << start << "->" << target << ", shortest length = " << length << std::endl;
            }
//            if (i % 100 == 0) {
//                std::cout << "finish " << i << " cases " << std::endl;
//            }
            gettimeofday (&tvafter , &tz);
            double time_interval = (tvafter.tv_sec-tvpre.tv_sec)+(tvafter.tv_usec-tvpre.tv_usec)/10e6;
            if(time_interval > .1) {
                std::cout << " finish :  " << i << " cases " << std::endl;
                gettimeofday (&tvpre , &tz);
            }
            // print after one second
//            if(statistics[0].front() > length + EPS_FR) {
//                std::cout << "index " << i << ": (" << sx << ", " << sy << ") -> (" << gx << ", " << gy << "), shortest length = " << length << std::endl;
//                std::cout << "statistics[0].front() > length : " << statistics[0].front() << " > " << length << std::endl;
//            }
            for (auto &statistic : statistics) {
                statistic.push_back(length);
            }
            for (auto &output_stream : output_streams) {
                std::stringstream sst;
                sst << output_stream << length;
                output_stream = sst.str();
            }
            statisticss.push_back(statistics);
            output_streamss.push_back(output_streams);
        }
        std::cout << " finish " << start_and_targets.size() << " cases" << std::endl;
        return true;
    }

    template<Dimension N>
    using StartAndTarget = std::pair<Pointi<N>, Pointi<N> >;

    template<Dimension N>
    using StartAndTargets = std::vector<StartAndTarget<N> >;

    template<Dimension N>
    StartAndTargets<N> expandSAT(const StartAndTargets<N>& sat, double ratio) {
        StartAndTargets<N> retv;
        for(const auto& start_and_target : sat) {
            Pointi<N> new_start = {floor(start_and_target.first[0]*ratio), floor(start_and_target.first[1]*ratio)},
                      new_target = {floor(start_and_target.second[0]*ratio), floor(start_and_target.second[1]*ratio)};
            retv.push_back({new_start, new_target});
        }
        return retv;
    }

    template <Dimension N>
    StartAndTargets<N> generateRandomPointPair(const IS_OCCUPIED_FUNC<N>& is_occupied,
                                               DimensionLength* dim,
                                               int required_number,
                                               int max_random_select = 100) {
        Pointis<N> pts;
        Pointi<N> pt;
        Id total_index = getTotalIndexOfSpace<N>(dim), temp_id;
        StartAndTargets<N> retv;
        Pointis<N-1> neighbor = GetNeightborOffsetGrids<N-1>();
        for(int i=0; i<required_number; i++) {
            pts.clear();
            for (int j = 0; j < max_random_select; j++) {
                temp_id = (Id) (total_index * rand() / double(RAND_MAX));
                //std::cout << "temp_id " << temp_id << std::endl;
                pt = IdToPointi<N>(temp_id, dim);
                if (is_occupied(pt)) { continue; }
                else {
                    if(pts.size() == 1) {
                        if(!LineCrossObstacle(pts[0], pt, is_occupied, neighbor)) {
                            continue;
                        }
                    }
                    pts.push_back(pt);
                    if (pts.size() == 2) {
                        if (pts[0] == pts[1]) {
                            pts.pop_back();
                            continue;
                        }
                        break;
                    }
                }
            }
            if (pts.size() != 2) {
                //std::cout << " index " << i << " find no free start/target pair" << std::endl;
                continue;
            }
            retv.push_back({pts[0], pts[1]});
        }
        return retv;
    }

}
#endif //FREENAV_MASSIVE_TEST_INTERFACES_H
