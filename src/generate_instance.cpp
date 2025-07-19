#include "lamapf_and_gazebo/common_interfaces.h"
#include "freeNav-base/dependencies/2d_grid/picture_loader.h"

using namespace freeNav::LayeredMAPF;


InstanceOrients<2> generateLargeAgentInstanceForMap(const AgentPtrs<2>& pic_agents,
                                                    const IS_OCCUPIED_FUNC<2> & pic_isoc,
                                                    DimensionLength* pic_dim,  
                                                    int pic_times_of_try,
                                                    int maximum_sample_count = 1e7) {

    for(int i=0; i<pic_times_of_try; i++) {

        LargeAgentMAPF_InstanceGenerator<2> generator(pic_agents, pic_isoc, pic_dim, maximum_sample_count);
        auto instances_and_path = generator.getNewInstance();
        InstanceOrients<2> instances_pic;
        for(int j=0; j<instances_and_path.size(); j++) {
            instances_pic.push_back(instances_and_path[j].first);
        }

        if(!instances_pic.empty()) {
            std::cout << "-- find " << pic_agents.size() << " agents and poses after " << i << " sample" <<  std::endl;
            //InstanceSerializer<2> serializer(pic_agents, instances_pic);
            // if(serializer.saveToFile(map_test_config.at("la_ins_path"))) {
            //     std::cout << "save to path " << map_test_config.at("la_ins_path") << " success" << std::endl;
                 return instances_pic;
            // } else {
            //     std::cout << "save to path " << map_test_config.at("la_ins_path") << " failed" << std::endl;
            //     return {};
            // }
        } else {
            std::cout << "-- find no " << pic_agents.size() << " agents and poses after " << i << " sample" <<  std::endl;
        }
    }
    return {};
}

int main() {
    std::cout << "start generate instance node" << std::endl;

    IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;


    // generate agents
    // int required_agents = 40;
    // AgentPtrs<2> agents = RandomMixedAgentsGenerator(required_agents/2,
    //     1., 2.,
    //     required_agents/2,
    //     -2., -.2,
    //     1., 2.,
    //     .2, 2.,
    //     .1, dim);


    // related agent ptrs of above agents. in grid
    AgentPtrs<2> real_agents = {
        std::make_shared<BlockAgent_2D >(Pointf<2>({-0.3/reso, -0.2/reso}), Pointf<2>({0.32/reso, 0.2/reso}), 0, dim),
        std::make_shared<CircleAgent<2>>(.3/reso, 1, dim),
        std::make_shared<BlockAgent_2D >(Pointf<2>({-0.2/reso, -0.5/reso}), Pointf<2>({1.1/reso, 0.5/reso}), 2, dim),
        std::make_shared<BlockAgent_2D >(Pointf<2>({-0.4/reso, -0.2/reso}), Pointf<2>({0.1/reso, 0.2/reso}), 3, dim),
        std::make_shared<BlockAgent_2D >(Pointf<2>({-0.5/reso, -0.3/reso}), Pointf<2>({0.5/reso, 0.3/reso}), 4, dim),

    };
    AgentPtrs<2> agents;
    for(int i=0; i<REAL_ROBOTS.size(); i++) {
        AgentPtr<2> local_copy = real_agents[REAL_ROBOTS[i]]->copy();
        local_copy->id_ = i;
        agents.push_back(local_copy);
    }

    // generate instances
    InstanceOrients<2> instances =  generateLargeAgentInstanceForMap(agents, is_occupied_func, dim, 100);

    InstanceSerializer<2> serializer(agents, instances);
    if(serializer.saveToFile(map_test_config.at("la_ins_path"))) {
        std::cout << "save to path " << map_test_config.at("la_ins_path") << " success" << std::endl;
    } else {
        std::cout << "save to path " << map_test_config.at("la_ins_path") << " failed" << std::endl;
    }

    std::cout << "find " << agents.size() << " agent's instance" << std::endl;
    Canvas canvas("GenerateInstanceROS2", dim[0], dim[1], 20, 5);
    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim, is_occupied_func);
        for (int i=0; i<instances.size(); i++)
        {
            //const auto &instance = instances[current_subgraph_id]; // zoom_ratio/10
            const auto &instance = instances[i]; // zoom_ratio/10
            agents[i]->drawOnCanvas(instance.first, canvas, COLOR_TABLE[i%30]);

            agents[i]->drawOnCanvas(instance.second, canvas, COLOR_TABLE[i%30]);

            canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], -orientToPi_2D(instance.first.orient_), 1, std::max(1, zoom_ratio/10));
            canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], -orientToPi_2D(instance.second.orient_) , 1, std::max(1, zoom_ratio/10));

        }
        char key = canvas.show();
    }

    return 0;
}