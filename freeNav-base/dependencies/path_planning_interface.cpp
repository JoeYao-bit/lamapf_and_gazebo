//
// Created by yaozhuo on 2022/7/5.
//

#include "path_planning_interface.h"
#include <fstream>
namespace freeNav {

    void appendToFile(const std::string &file_name, const std::string &content, bool prune) {
        // 3, save result to file
        std::ofstream os(file_name, prune ? std::ios_base::out : std::ios_base::app);
        //os << "TYPE START TARGET TIME_COST MAKE_SPAN TOTAL_LENGTH " << std::endl;
        os << content << std::endl;
        os.close();
    }

    void printCurrentTime() {
        time_t curtime;
        time(&curtime);
//        std::cout << "seconds from 1970:" << time(&curtime) << std::endl;
        std::cout << "current date and time:" << ctime(&curtime);
//        tm *nowtime = localtime(&curtime);
//        // output structured year/month/day
//        std::cout << "year: " << 1900 + nowtime->tm_year << std::endl;
//        std::cout << "month: " << 1 + nowtime->tm_mon << std::endl;
//        std::cout << "day: " << nowtime->tm_mday << std::endl;
//        std::cout << "hour: " << nowtime->tm_hour << ":";
//        std::cout << nowtime->tm_min << ":" << std::endl;
    }

}