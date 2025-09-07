#ifndef ACTION_DEPENDENCY_GRAPH
#define ACTION_DEPENDENCY_GRAPH
#include "common_interfaces.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/strong_components.hpp>

    template<Dimension N>
    struct ActionDependencyGraph {

        explicit ActionDependencyGraph(const std::vector<LAMAPF_Path>& paths,
                                       const std::vector<AgentPtr<N>>& agents,
                                       const std::vector<PosePtr<int, N> >& all_poses) :
                paths_(paths), agents_(agents), all_poses_(all_poses) {

            assert(paths_.size() == agents_.size());
            // debug, check whether path is collision free
            for(int i=0; i<agents.size(); i++) {
                for(int j=i+1; j<agents.size(); j++) {
                    auto retv_ptr = detectFirstConflictBetweenPaths(paths[i], paths[j], agents[i], agents[j], all_poses);
                    if (retv_ptr != nullptr) {
                        std::cout << "FATAL: detect conflicts between agent "
                                << agents[i] << "path_size(" << paths[i].size() << ")" << " and "
                                << agents[j] << "path_size(" << paths[j].size() << ")"
                                << ", at (t1 = " << retv_ptr->t1 << ", t2 = " << retv_ptr->t2 << ")" << std::endl;
                                assert(0);
                    }
                }
            }            

            // if(isCollide(agents_[1],  *all_poses[paths_[1][37]],  *all_poses[paths_[1][38]],
            //              agents_[11], *all_poses[paths_[11][37]], *all_poses[paths_[11][38]])) {
            //     std::cout << "== detect conflict " << std::endl;
            // }

            // 0, construct path_index_to_node_
            path_index_to_node_.resize(paths_.size());
            int id_count = 0;
            for(int i=0; i<paths_.size(); i++) {
                path_index_to_node_[i].resize(paths_[i].size());
                std::cout << "path " << i <<"(" << paths_[i].size() << ")" << ": ";
                for(int j=0; j<paths_[i].size(); j++) {
                    std::cout << *all_poses_[paths_[i][j]] << " ";
                    path_index_to_node_[i][j] = id_count;
                    node_to_path_index_.push_back({i, j});
                    id_count ++;
                }
                std::cout << std::endl;
            }
            ADG_.resize(id_count, {});
            action_status_.resize(id_count, UNVISITED);
            // 1, insert path to ADG
           for(int i=0; i<paths_.size(); i++) {
               for(int j=0; j<paths_[i].size()-1; j++) {
                   const int& pre_id = path_index_to_node_[i][j];
                   const int& next_id = path_index_to_node_[i][j+1];
                   ADG_[next_id].push_back(pre_id);
               }

           }
            // 2, traversal all path to construct action dependency graph
            for(int i=0; i<paths_.size(); i++) {
                for(int j=0; j<paths_.size(); j++) {
                    if(i != j) { 
                        std::vector<std::pair<int, int> > map_edge;
                        for(int t1=0; t1<paths_[i].size()-1; t1++) {
                            for(int t2=t1; t2<paths_[j].size()-1; t2++) {
                                if(isCollide(agents_[i], *all_poses[paths_[i][t1]], *all_poses[paths_[i][t1+1]],
                                             agents_[j], *all_poses[paths_[j][t2]], *all_poses[paths_[j][t2+1]])) {
                                    const int& pre_id  = path_index_to_node_[i][t1];
                                    const int& next_id = path_index_to_node_[j][t2];
                                    map_edge.push_back({pre_id, next_id});         
                                    ADG_[next_id].push_back(pre_id);
                                }
                            }
                        }
                        // if(!map_edge.empty() && (i == 1 || j == 1)) {
                        //     std::cout << "insert ADG edge next: ";
                        //     for(const auto& edge_pair : map_edge) {
                        //         int i  = node_to_path_index_[edge_pair.first].first;
                        //         int t1 = node_to_path_index_[edge_pair.first].second;
                        //         int j  = node_to_path_index_[edge_pair.second].first;
                        //         int t2 = node_to_path_index_[edge_pair.second].second;
                        //         std::cout << "next a" << j << "t" << t2 << ", pre: a" << i << "t" << t1 << ", ";
                        //     }
                        //     std::cout << std::endl;
                        // }
                    }
                }
            }
            //debug, ADG loop detection
            adgLoopDetection();
        }

        void adgLoopDetection() {
            using namespace boost;
            // 定义一个有向图
            typedef adjacency_list<vecS, vecS, directedS> Graph;
            Graph g(ADG_.size());

            // 添加边 (构成环: 0->1->2->0)
            // add_edge(0, 1, g);
            // add_edge(1, 2, g);
            // add_edge(2, 0, g);
            // add_edge(3, 4, g);

            for(int i=0; i<ADG_.size(); i++) {
                //std::cout << "a " << node_to_path_index_[i].first << ", t = " << node_to_path_index_[i].second << ": ";
                for(const auto& j : ADG_[i]) {
                    add_edge(i, j, g);
                    //if(node_to_path_index_[i].first != node_to_path_index_[j].first) {
                    //    std::cout << "a " << node_to_path_index_[j].first << ", t = " << node_to_path_index_[j].second << ", "; 
                    //}
                }
                //std::cout << std::endl;
            }

            // 3. 准备存储 SCC 结果的容器
            // comp 将存储每个顶点属于哪个 SCC（组件编号）
            std::vector<int> comp(num_vertices(g));
            // 调用 strong_components 函数计算 SCC
            int num_components = strong_components(g, 
                                                &comp[0] 
                                                );

            // 4. 分析结果：将顶点根据它们的组件编号分组
            // 创建一个向量，每个元素是一个向量，用于存储属于同一 SCC 的顶点
            std::vector<std::vector<int>> components(num_components);
            for (size_t i = 0; i < comp.size(); ++i) {
                components[comp[i]].push_back(i);
            }

            // 5. 检查是否有环（即是否存在大小 > 1 的 SCC）并打印环信息
            bool has_cycle = false;
            std::cout << "Found " << num_components << " strong components.\n";

            for (int i = 0; i < num_components; ++i) {
                if (components[i].size() > 1) {
                    has_cycle = true;
                    std::cout << "Cycle found in component " << i << ": ";
                    for (int node : components[i]) {
                        std::cout << "a" << node_to_path_index_[node].first << "t" << node_to_path_index_[node].second << ", ";
                    }
                    std::cout << std::endl;
                }
            }

            assert(!has_cycle);
        }

        void setActionProcessing(int path_id, int t) {
            action_status_[getActionId(path_id, t)] = PROCESSING;
        }

        void setActionLeave(int path_id, int t) {
            action_status_[getActionId(path_id, t)] = LEAVE;
        }

        bool isActionUnvisited(int path_id, int t) const {
            return action_status_[getActionId(path_id, t)] == UNVISITED;
        }

        bool isActionProcessing(int path_id, int t) const {
            return action_status_[getActionId(path_id, t)] == PROCESSING;
        }

        bool isActionLeave(int path_id, int t) const {
            return action_status_[getActionId(path_id, t)] == LEAVE;
        }

        bool isActionValid(int path_id, int t) const {
            return isActionValid(getActionId(path_id, t));
        }

        /*******************************************************/
        void setActionProcessing(int action_id) {
            action_status_[action_id] = PROCESSING;
        }

        void setActionLeave(int action_id) {
            action_status_[action_id] = LEAVE;
        }

        bool isActionUnvisited(int action_id) const {
            return action_status_[action_id] == UNVISITED;
        }

        bool isActionProcessing(int action_id) const {
            return action_status_[action_id] == PROCESSING;
        }

        bool isActionLeave(int action_id) const {
            return action_status_[action_id] == LEAVE;
        }

        // the maximal actions that agent can take since time t
        // rotate and move forward's cost may not the same
        // finish all valid action avoid unnecessary wait
        int getMaximalValidAction(int agent_id, int start_t) const {
            for(int t=start_t; t<paths_[agent_id].size()-1; t++) {
                if(!isActionValid(agent_id, t)) { return t-1; }
            }
            return paths_[agent_id].size() - 2;// if all actions are valid, just finish all actions
        }

        inline int getActionId(int path_id, int t) const {
            return path_index_to_node_[path_id][t];
        }

        // an action is valid after all precedent action are finished
        bool isActionValid(int action_id) const {
            for(const auto& pre_id : ADG_[action_id]) {
                if(!isActionLeave(pre_id)) { 
                    std::cout << "a" << node_to_path_index_[action_id].first << ", t=" << node_to_path_index_[action_id].second << " is invalid because " << "a" << node_to_path_index_[pre_id].first << ", t=" << node_to_path_index_[pre_id].second << " is not leave" << std::endl;
                    return false;
                 }
            }
            return true;
        }

        void clearAllProgress() {
            action_status_.resize(agents_.size(), UNVISITED);
        }

//    private:

        std::vector<LAMAPF_Path> paths_;
        std::vector<AgentPtr<N> > agents_;
        std::vector<PosePtr<int, N> > all_poses_;

        std::vector<std::vector<int> > path_index_to_node_;
        std::vector<std::pair<int,int> > node_to_path_index_;
        std::vector<std::vector<int> > ADG_; // insert all edge of action dependency graph

        enum ACTION_STATE {UNVISITED, PROCESSING, LEAVE};
        std::vector<ACTION_STATE> action_status_;

    };
#endif