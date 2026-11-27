//
// Created by yaozhuo on 2022/1/2.
//

#include "../basic_elements/point.h"

namespace freeNav {

    bool isFourCornerOccupied(const Pointi<2>& start_pt, IS_OCCUPIED_FUNC<2> is_occupied) {
        Pointi<2> buffer = start_pt;
        if(!is_occupied(buffer)) return false;
        buffer[0] --;
        if(!is_occupied(buffer)) return false;
        buffer[1] --;
        if(!is_occupied(buffer)) return false;
        buffer[1] ++;
        if(!is_occupied(buffer)) return false;
        return true;
    }

    bool isOneInFourCornerOccupied(const FractionPair& start_pt, IS_OCCUPIED_FUNC<2> is_occupied) {
        Pointi<2> buffer;
        int occ_count;

        buffer[0] = start_pt.first.ceil(), buffer[1] = start_pt.second.ceil();
        if(is_occupied(buffer)) occ_count ++;

        buffer[0] = start_pt.first.ceil(), buffer[1] = start_pt.second.floor();
        if(is_occupied(buffer)) occ_count ++;

        buffer[0] = start_pt.first.floor(), buffer[1] = start_pt.second.ceil();
        if(is_occupied(buffer)) occ_count ++;

        buffer[0] = start_pt.first.floor(), buffer[1] = start_pt.second.floor();
        if(is_occupied(buffer)) occ_count ++;
        return occ_count == 1;
    }

}
