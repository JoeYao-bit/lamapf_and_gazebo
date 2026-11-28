#include <iostream>
#include <thread>
#include <chrono>

#include "controller.h"

std::shared_ptr<ActionDependencyGraph<2> > CenteralControllerNew::ADG_ = nullptr;

std::vector<int> CenteralControllerNew::progress_of_agents_ = {};

std::vector<std::shared_ptr<Pose<int, 2>> > CenteralControllerNew::all_poses_ = {};

std::pair<AgentPtrs<2>, InstanceOrients<2> > CenteralControllerNew::instances_ = {};

DimensionLength* CenteralControllerNew::dim_ = nullptr;

std::vector<bool> CenteralControllerNew::agent_finishes_ = {};


auto map_test_config = MAPFTestConfig_IndustrialWarehouse;//地图名称

auto is_grid_occupied = [](const cv::Vec3b& color) -> bool {
    if (color == cv::Vec3b::all(255)) { return false; }
    return true;
};

// load map
PictureLoader loader(map_test_config.at("map_path"), is_grid_occupied); //加载地图

DimensionLength* dim = loader.getDimensionInfo(); //地图尺寸

auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); }; //栅格是否占据判定函数


int main() {

    std::cout << "start simulation/ of CenteralControllerNew" << std::endl;

    std::string ins_path = map_test_config.at("la_ins_path"); //从文件中加载MAPF问题
    InstanceDeserializer<2> deserializer;
    if (deserializer.loadInstanceFromFile(ins_path, dim)) {
        std::cout << "load from path " << ins_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << ins_path << " failed" << std::endl;
        return 0;
    }
    //MAPF问题，包括单位类型和起末位置
    std::pair<AgentPtrs<2>, InstanceOrients<2> > instances = deserializer.getTestInstance({5}, 1).front(); // get all instances

    for(int i=0; i<instances.second.size(); i++) {
        std::cout << *instances.first[i] << ": " << instances.second[i].first << "->" << instances.second[i].second << std::endl;
    }

    //启动中央控制器，输入地图和MAPF问题
    auto ctl = CenteralControllerNew(dim, is_occupied, instances, true);

    // 定时周期：500ms，等价于 ROS2 create_timer(500ms, callback)
    const std::chrono::milliseconds period{500};
    auto next_tick = std::chrono::steady_clock::now();


    while(true) {
        if(!paused) {
            //std::cout << "in sim loop..." << std::endl;
            for(int i=0; i<instances.second.size(); i++) {
                //std::cout << "flag 1" << std::endl;
                if(!ctl.agent_finishes_[i]) {
                    //std::cout << "flag 2" << std::endl;
                    if(ctl.progress_of_agents_[i] < ctl.ADG_->paths_[i].size()-1) {
                        //std::cout << "flag 3" << std::endl;
                        // 打印当前动作的起始位姿和目标位姿
                        size_t current_pose_id = ctl.ADG_->paths_[i][ctl.progress_of_agents_[i]];
                        size_t next_pose_id    = ctl.ADG_->paths_[i][ctl.progress_of_agents_[i]+1];
                        PosePtr<int, 2> current_pose = ctl.ADG_->all_poses_[current_pose_id];
                        PosePtr<int, 2> next_pose    = ctl.ADG_->all_poses_[next_pose_id];
                        //std::cout << "agent " << i << "'s action " << *current_pose << "->" << *next_pose << std::endl;
                    }
                    ctl.agent_finishes_[i] = true; // 将对应机器人的动作设置为完成
                }
            }
            ctl.updateADG(); // 更新动作依赖图
        }
        // 更新下一次触发时间
        next_tick += period;
        // 休眠到下一个tick点
        std::this_thread::sleep_until(next_tick);
    }

    return 0;
}
