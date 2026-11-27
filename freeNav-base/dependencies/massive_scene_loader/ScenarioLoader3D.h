//
// Created by yaozhuo on 2022/11/27.
//

#ifndef FREENAV_BASE_SCENARIOLOADER3D_H
#define FREENAV_BASE_SCENARIOLOADER3D_H

#include "../../basic_elements/point.h"
#include <vector>

namespace freeNav {

    template<Dimension N>
    using TestCase = std::pair<Pointi<N>, Pointi<N> >;

    struct TestCaseMovingAi {

        TestCase<3> test_case_;

        double path_length_;

        double unknown_value_;

    };

    using TestCases3DMovingAi = std::vector<TestCaseMovingAi>;

    class ScenarioLoader3D {
    public:
        ScenarioLoader3D(const std::string& file_path);

        TestCases3DMovingAi getAllTestCases() const {
            return case_sequence_;
        }
    private:

        TestCases3DMovingAi case_sequence_;
    };
}
#endif //FREENAV_SCENARIOLOADER3D_H
