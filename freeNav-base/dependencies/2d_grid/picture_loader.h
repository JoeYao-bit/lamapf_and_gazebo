//
// Created by yaozhuo on 2021/12/2.
//

#ifndef FREENAV_BASE_PICTURE_LOADER_H
#define FREENAV_BASE_PICTURE_LOADER_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/imgproc.hpp"//draw a line
#include "opencv2/opencv.hpp"
#include "../../basic_elements/point.h"
#include "../environment.h"

namespace freeNav {


    class PictureLoader : public Grid_Loader<2> {

    public:
        /* load grid map from a picture, keep the raw picture and a writeable copy */
        explicit PictureLoader(std::string file, bool (*f1)(const cv::Vec3b& color));

        bool isOccupied(const Pointi<2> & pt) const override;

        void setOccupied(const Pointi<2> & pt) override;

        cv::Mat getOriginPicture() const {
            return raw_map_;
        }

    private:
        /* the original map, do not edit it */
        cv::Mat raw_map_;

        GridMap grid_map_;

    };

}

#endif //FREENAV_PICTURE_LOADER_H
