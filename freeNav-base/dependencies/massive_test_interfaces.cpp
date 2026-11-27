//
// Created by yaozhuo on 9/12/26.
//

#include "massive_test_interfaces.h"

namespace freeNav {


    // for 2d grid map data set from https://www.movingai.com/benchmarks/grids.html
    bool SceneTest2D(const std::string &config_file_path,
                     const Point2PointPathPlannings<2, Pointi<2>, Pointi<2>> &p2p_plan_test,
                     StatisticSS &statisticss,
                     OutputStreamSS &output_streamss,
                     int max_count_of_case) {

        statisticss.clear();
        output_streamss.clear();
        ScenarioLoader2D sl(config_file_path.c_str());
        int count_of_experiments = sl.GetNumExperiments();
        std::cout << "get " << count_of_experiments << " 2d experiments" << std::endl;
        if (count_of_experiments <= 0) return false;

        struct timezone tz;
        struct timeval tvpre;
        struct timeval tvafter;
        gettimeofday (&tvpre , &tz);

        // load experiment data
        for (int i = 0; i < std::min(max_count_of_case, count_of_experiments); i++) {
            const auto &experiment = sl.GetNthExperiment(i);
            int sx = experiment.GetGoalX(), sy = experiment.GetGoalY(), gx = experiment.GetStartX(), gy = experiment.GetStartY();
            double length = experiment.GetDistance();
            // do test
            StatisticS statistics;
            OutputStreamS output_streams;
            Pointi<2> start, target;
            start[0] = sx, start[1] = sy, target[0] = gx, target[1] = gy;
            std::cout << "index " << i << ": (" << sx << ", " << sy << ") -> (" << gx << ", " << gy << "), shortest length = " << length << std::endl;
            if (!Point2PointPathPlanningTest<2, Pointi<2>, Pointi<2> >(start,
                                                                       target,
                                                                       p2p_plan_test,
                                                                       statistics,
                                                                       output_streams)) {
                std::cout << "failed index " << i << ": (" << sx << ", " << sy << ") -> (" << gx << ", " << gy
                          << "), shortest length = " << length << std::endl;
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
        std::cout << " finish " <<  std::min(max_count_of_case, count_of_experiments) << " cases" << std::endl;
        return true;
    }


    // for 2d grid map data set from https://www.movingai.com/benchmarks/grids.html
    bool SceneTest2DIndividual(const std::string &config_file_path,
                               const Point2PointPathPlannings<2, Pointi<2>, Pointi<2>> &p2p_plan_tests,
                               StatisticSS &statisticss,
                               OutputStreamSS &output_streamss,
                               int max_count_of_case) {

        statisticss.clear();
        output_streamss.clear();
        ScenarioLoader2D sl(config_file_path.c_str());
        int count_of_experiments = sl.GetNumExperiments();
        std::cout << "get " << count_of_experiments << " 2d experiments" << std::endl;
        if (count_of_experiments <= 0) return false;



        for(const auto& method : p2p_plan_tests) {

            // load experiment data
            for (int i = 0; i < std::min(max_count_of_case, count_of_experiments); i++) {
                const auto &experiment = sl.GetNthExperiment(i);
                int sx = experiment.GetGoalX(), sy = experiment.GetGoalY(), gx = experiment.GetStartX(), gy = experiment.GetStartY();
                double length = experiment.GetDistance();
                for(int j=0; j<10; j++) {
                    // do test
                    StatisticS statistics;
                    OutputStreamS output_streams;
                    Pointi<2> start, target;
                    start[0] = sx, start[1] = sy, target[0] = gx, target[1] = gy;
                    //std::cout << "index " << i << ": (" << sx << ", " << sy << ") -> (" << gx << ", " << gy << "), shortest length = " << length << std::endl;
                    if (!SinglePoint2PointPathPlanningTest<2, Pointi<2>, Pointi<2> >(
                            start, target, method,
                            statistics, output_streams)) {
                        std::cout << "failed index " << i << ": (" << sx << ", " << sy << ") -> (" << gx << ", " << gy
                                  << "), shortest length = " << length << std::endl;
                    }
                    if (i % 100 == 0) {
                        std::cout << "finish " << i << " cases " << std::endl;
                    }
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
                    if(i%1000 == 0) {
                        statisticss.shrink_to_fit();
                        output_streamss.shrink_to_fit();
                    }
                }
            }

        }
        return true;
    }



    // for 3d voxel map data set from https://www.movingai.com/benchmarks/voxels.html
    bool SceneTest3D(const std::string &config_file_path,
                     const Point2PointPathPlannings<3, Pointi<3>, Pointi<3> > &p2p_plan_test,
                     StatisticSS &statisticss,
                     OutputStreamSS &output_streamss,
                     int max_count_of_case) {

        statisticss.clear();
        output_streamss.clear();
        ScenarioLoader3D sl(config_file_path.c_str());
        auto all_experiments = sl.getAllTestCases();
        int experiment_size = all_experiments.size();
        std::cout << "get " << all_experiments.size() << " 3d experiments" << std::endl;
        if (all_experiments.size() <= 0) return false;
        struct timezone tz;
        struct timeval tvpre;
        struct timeval tvafter;
        gettimeofday(&tvpre, &tz);
        // load experiment data
        for (int i = 0; i < std::min(max_count_of_case, experiment_size); i++) {
            const auto &experiment = all_experiments[i];
            double length = experiment.path_length_;
            // do test
            StatisticS statistics;
            OutputStreamS output_streams;
            Pointi<3> start = experiment.test_case_.first, target = experiment.test_case_.second;
            //std::cout << "index " << i  << ": " << start << "->" << target << ", shortest length = " << length << std::endl;
            if (!Point2PointPathPlanningTest<3, Pointi<3>, Pointi<3> >(start,
                                                                       target,
                                                                       p2p_plan_test,
                                                                       statistics,
                                                                       output_streams)) {
                std::cout << "failed index " << i << ": " << start << "->" << target << ", shortest length = " << length
                          << std::endl;
            }
//            if (i % 100 == 0) {
//                std::cout << "finish " << i << " cases " << std::endl;
//            }
            gettimeofday(&tvafter, &tz);
            double time_interval = (tvafter.tv_sec - tvpre.tv_sec) + (tvafter.tv_usec - tvpre.tv_usec) / 10e6;
            if (time_interval > .1) {
                std::cout << " finish :  " << i << " cases " << std::endl;
                gettimeofday(&tvpre, &tz);
            }
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
            if (i % 1000 == 0) {
                statisticss.shrink_to_fit();
                output_streamss.shrink_to_fit();
            }
            //break;
        }
        return true;
    }
}