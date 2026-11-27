//
// Created by yaozhuo on 2022/7/10.
//

#include "text_map_loader.h"

namespace freeNav {

    /* load grid map from a picture, keep the raw picture and a writeable copy */
    TextMapLoader::TextMapLoader(const std::string& file_path, bool (*f1)(const char& value)) : Grid_Loader() {
        std::ifstream text_map(file_path);
        std::string line;
        std::vector<std::string> lines;
        while(getline(text_map, line)) {
            // avoid Enter in end of line
            if(line.back() == 13) {
                line.pop_back();
            }
            lines.push_back(line);
        }
        if(lines.size() <= 4) {
            std::cout << "TML::TML: lines.size() <= 4 " << std::endl;
            exit(0);
        }
        std::cout << "map info: " << std::endl;
        for(int i=0; i<4; i++) {
            std::cout << lines[i] << std::endl;
        }
        std::string value;
        std::stringstream height_str(lines[1]), width_str(lines[2]);

        width_str >> value >> value;
        dimen_[0] = atoi(value.c_str());

        height_str >> value >> value;
        dimen_[1] = atoi(value.c_str());

        if(lines.size() != dimen_[1] + 4) {
            std::cout << "TML::TML: lines.size() = " << lines.size() << "  != dimen_[1] + 4 (" << dimen_[1] + 4 << ")" << std::endl;
            exit(0);
        }
        grid_map_ = GridMap(dimen_[0]*dimen_[1], FREE);
        for(int i=4; i<lines.size(); i++) {
            if(lines[i].size() != dimen_[0]) {
                std::cout << " last char is " << static_cast<int>(lines[i].back()) << std::endl;
                std::cout << "TML::TML: line " << i << " size() " << lines[i].size() << " != dimen_[0] " << dimen_[0] << std::endl;
                exit(0);
            }
            for(int j=0; j<dimen_[0]; j++) {
                grid_map_[(i-4)*dimen_[0]+j] = f1(lines[i][j]) ? GridState::OCCUPIED : GridState::FREE;
            }
        }
        int id = 0;
        for(int i=0; i<dimen_[0]; i++) {
            for(int j=0; j<dimen_[1]; j++) {
                id = i*dimen_[0] + j;
                if(grid_map_[id] == GridState::OCCUPIED) {
                    occ_pt_ids_.insert(id);
                    occ_pts_.push_back(Pointi<2>{i, j});
                }
            }
        }
    }

    bool TextMapLoader::isOccupied(const Pointi<2> & pt) const {
        //std::cout << "reach TextMapLoader::isOccupied" << std::endl;
        if(pt[0] < 0 || pt[0] >= dimen_[0] || pt[1] < 0 || pt[1] >= dimen_[1]) { return true; }
        auto index = pt[1]*dimen_[0] + pt[0];
        return grid_map_[index] != FREE;
    }

    void TextMapLoader::setOccupied(const Pointi<2> & pt) {
        auto index = pt[1]*dimen_[0] + pt[0];
        if(index >= dimen_[0]*dimen_[1]) return;
        grid_map_[ pt[1]*dimen_[0] + pt[0] ] = OCCUPIED;
    }

}