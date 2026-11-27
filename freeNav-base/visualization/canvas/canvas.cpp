//
// Created by yaozhuo on 2021/9/20.
//

#include <eigen3/Eigen/Dense>
#include "canvas.h"
#include "iomanip"
namespace freeNav {

    Canvas::Canvas(std::string name, int size_x, int size_y, double resolution, double zoom_ratio) :
            canvas_(size_y * zoom_ratio, size_x * zoom_ratio, CV_8UC3, cv::Scalar::all(255)), resolution_(resolution) {
        center_[0] = size_x / 2;
        center_[1] = size_y / 2;
        name_ = name;
        zoom_ratio_ = zoom_ratio;
        setColorTable();
        cv::namedWindow(name_, cv::WINDOW_NORMAL);
    }

    void Canvas::setColorTable() {
        /* set gradation color table, from blue -> green -> red */
        for (double i = 0; i < 1.; i += .1) {
            gradation_color_table_.push_back(cv::Scalar(0, 255 * (1. - i), 255 * i));
        }
        for (double i = 0; i < 1.; i += .1) {
            gradation_color_table_.push_back(cv::Scalar(255 * (i), 0, 255 * (1. - i)));
        }
        std::vector<cv::Scalar> reverse_color_table(gradation_color_table_.rbegin(), gradation_color_table_.rend());
        gradation_color_table_.insert(gradation_color_table_.end(), reverse_color_table.begin(),
                                      reverse_color_table.end());
    }

    void Canvas::drawLineInt(int x1, int y1, int x2, int y2, bool center_offset, int line_width, const cv::Scalar &color) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        cv::line(canvas_,
                 cv::Point2i(x1 * zoom_ratio_, y1 * zoom_ratio_) + cv::Point(offset, offset),
                 cv::Point2i(x2 * zoom_ratio_, y2 * zoom_ratio_) + cv::Point(offset, offset),
                 color, 1, cv::LINE_AA);
    }

    void Canvas::drawLineFloat(float x1, float y1, float x2, float y2, bool center_offset, int line_width, const cv::Scalar &color) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        cv::line(canvas_,
                 cv::Point2i(x1 * zoom_ratio_, y1 * zoom_ratio_) + cv::Point(offset, offset),
                 cv::Point2i(x2 * zoom_ratio_, y2 * zoom_ratio_) + cv::Point(offset, offset),
                 color, line_width, cv::LINE_AA);
    }

    void Canvas::drawRectangleFloat(const Pointf<2>& min_pt, const Pointf<2>& max_pt, bool center_offset, int line_width, const cv::Scalar &color, float weight) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        cv::Point2i min_pt_cv(min_pt[0] * zoom_ratio_, min_pt[1] * zoom_ratio_);
        cv::Point2i max_pt_cv(max_pt[0] * zoom_ratio_, max_pt[1] * zoom_ratio_);
        if(weight < 1.0) {
            cv::Mat temp_copy(canvas_.rows, canvas_.cols, CV_8UC3, cv::Scalar::all(0));
            cv::Mat background_copy = canvas_.clone();
            // draw transparent shapes
            cv::rectangle(temp_copy, min_pt_cv + cv::Point(offset, offset), max_pt_cv + cv::Point(offset, offset), color,
                          line_width);
            cv::Mat middle;
            cv::addWeighted(background_copy, 1.-weight, temp_copy, weight, 0, middle);
            // dig a hole in the background
            cv::rectangle(background_copy, min_pt_cv + cv::Point(offset, offset), max_pt_cv + cv::Point(offset, offset), color,
                          line_width);
            // use a background to keep other content no change
            cv::addWeighted(background_copy, weight, middle, 1, 0, canvas_);

        } else {
            cv::rectangle(canvas_, min_pt_cv + cv::Point(offset, offset), max_pt_cv + cv::Point(offset, offset), color,
                          line_width);
        }
    }


    void Canvas::drawLineInt(const Fraction& x1, const Fraction& y1, const Fraction& x2, const Fraction& y2, bool center_offset, int line_width, const cv::Scalar &color) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;

        int round_x1 = round(x1.toFloat()*zoom_ratio_), round_y1 = round(y1.toFloat()*zoom_ratio_);
        int round_x2 = round(x2.toFloat()*zoom_ratio_), round_y2 = round(y2.toFloat()*zoom_ratio_);

        cv::line(canvas_,
                 cv::Point2i(round_x1, round_y1) + cv::Point(offset, offset),
                 cv::Point2i(round_x2, round_y2) + cv::Point(offset, offset),
                 color, 1, cv::LINE_AA);
    }

    void Canvas::drawLine(float x1, float y1, float x2, float y2, int line_width, bool center_offset, const cv::Scalar &color) {
        Pointf<2> pti1 = transformToPixel(x1, y1);
        Pointf<2> pti2 = transformToPixel(x2, y2);
        drawLineFloat(pti1[0], pti1[1], pti2[0], pti2[1], center_offset, line_width, color);
    }

    void Canvas::drawPointInt(int x, int y, const cv::Vec3b &color) {
        canvas_.at<cv::Vec3b>(x * zoom_ratio_, y * zoom_ratio_) = color;
    }

    void Canvas::drawPoint(float x, float y, const cv::Vec3b &color) {
        Pointf<2> pti = transformToPixel(x, y);
        drawPointInt(pti[0], pti[1], color);
    }

    void Canvas::drawCircleInt(int x, int y, float radius, bool center_offset, int line_width, const cv::Scalar &color, float weight) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        if(weight < 1.0) {
            cv::Mat temp_copy(canvas_.rows, canvas_.cols, CV_8UC3, cv::Scalar::all(0));
            cv::Mat background_copy = canvas_.clone();
            // draw transparent shapes
            cv::circle(temp_copy, cv::Point(x * zoom_ratio_, y * zoom_ratio_) + cv::Point(offset, offset), radius * zoom_ratio_,
                       color, line_width, cv::LINE_AA);
            cv::Mat middle;
            cv::addWeighted(background_copy, 1.-weight, temp_copy, weight, 0, middle);
            // dig a hole in the background
            cv::circle(background_copy, cv::Point(x * zoom_ratio_, y * zoom_ratio_) + cv::Point(offset, offset), radius * zoom_ratio_,
                       cv::Vec3b::all(0), line_width, cv::LINE_AA);
            // use a background to keep other content no change
            cv::addWeighted(background_copy, weight, middle, 1, 0, canvas_);

        } else {
            cv::circle(canvas_, cv::Point(x * zoom_ratio_, y * zoom_ratio_) + cv::Point(offset, offset), radius * zoom_ratio_,
                       color, line_width, cv::LINE_AA);
        }
    }

    void Canvas::drawCircleInt(const Fraction& x, const Fraction& y, float radius, bool center_offset, int line_width, const cv::Scalar &color) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        int round_x = round(x.toFloat()*zoom_ratio_), round_y = round(y.toFloat()*zoom_ratio_);
        cv::circle(canvas_, cv::Point(round_x, round_y) + cv::Point(offset, offset), radius, color, line_width, cv::LINE_AA);
    }



    void Canvas::drawCircle(float x, float y, float radius, bool center_offset, int line_width, const cv::Scalar &color) {
        Pointf<2> pti = transformToPixel(x, y);
        float radius_i = radius * resolution_;
        //std::cout << "radius_i = " << radius_i << std::endl;
        drawCircleFloat(pti[0], pti[1], radius_i, center_offset, line_width, color);
    }

    void Canvas::drawEclipseInt(float x1, float y1, float radius, float start_angle, float end_angle,
                                bool center_offset, int line_width,
                                const cv::Scalar &color) {
        //     // 创建白色背景
        //    Mat img = Mat::ones(400, 400, CV_8UC3) * 255;
        //
        //    Point center(200, 200);  // 圆心
        //    Size axes(100, 100);     // 半径（长轴和短轴）
        //    double angle = 0;        // 椭圆旋转角度
        //    double startAngle = 30;  // 起始角度
        //    double endAngle   = 120; // 结束角度
        //    Scalar color(0, 0, 255); // 红色
        //    int thickness = -1;      // -1 表示填充
        //
        //    // 画扇形
        //    ellipse(img, center, axes, angle, startAngle, endAngle, color, thickness);
        //
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        int round_x = round(x1*zoom_ratio_), round_y = round(y1*zoom_ratio_);
        cv::ellipse(canvas_, cv::Point(round_x, round_y) + cv::Point(offset, offset),
                    cv::Size(radius*zoom_ratio_, radius*zoom_ratio_),
                    0, start_angle*180./M_PI, end_angle*180./M_PI,
                    color, line_width, cv::LINE_AA);

    }

    void Canvas::drawEclipse(float x, float y, float radius, float start_angle, float end_angle,
                     bool center_offset, int line_width,
                     const cv::Scalar &color) {
        Pointf<2> pti = transformToPixel(x, y);
        float radius_i = radius * resolution_;
        drawEclipseInt(pti[0], pti[1], radius_i, -end_angle, -start_angle, center_offset, line_width, color);
        Pointf<2> start_ptf, end_ptf;
        start_ptf[0] = x + radius*cos(start_angle);
        start_ptf[1] = y + radius*sin(start_angle);

        end_ptf[0] = x + radius*cos(end_angle);
        end_ptf[1] = y + radius*sin(end_angle);

        drawLine(x, y, start_ptf[0], start_ptf[1], std::max(1, line_width), center_offset, color);
        drawLine(x, y, end_ptf[0],   end_ptf[1],   std::max(1, line_width), center_offset, color);

    }

    void Canvas::drawRectangle(float x1, float y1, float x2, float y2, bool center_offset, int line_width, const cv::Scalar &color) {
        drawLine(x1, y2, x2, y2, center_offset, line_width, color);
        drawLine(x2, y2, x2, y1, center_offset, line_width, color);
        drawLine(x2, y1, x1, y1, center_offset, line_width, color);
        drawLine(x1, y1, x1, y2, center_offset, line_width, color);
    }

    void Canvas::drawCircleFloat(float x, float y, float radius, bool center_offset, int line_width,
                                 const cv::Scalar &color, float weight) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        int round_x = round(x*zoom_ratio_), round_y = round(y*zoom_ratio_);
        cv::circle(canvas_, cv::Point(round_x, round_y) + cv::Point(offset, offset),
                   radius*zoom_ratio_, color, line_width, cv::LINE_AA);
    }

    void Canvas::drawRectangleFloat(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4,
                                    bool center_offset, int line_width, const cv::Scalar &color) {
        if(center_offset) {
            x1 = (x1 + .5) * zoom_ratio_; y1 = (y1 + .5) * zoom_ratio_;
            x2 = (x2 + .5) * zoom_ratio_; y2 = (y2 + .5) * zoom_ratio_;
            x3 = (x3 + .5) * zoom_ratio_; y3 = (y3 + .5) * zoom_ratio_;
            x4 = (x4 + .5) * zoom_ratio_; y4 = (y4 + .5) * zoom_ratio_;
        } else {
            x1 = x1 * zoom_ratio_; y1 = y1 * zoom_ratio_;
            x2 = x2 * zoom_ratio_; y2 = y2 * zoom_ratio_;
            x3 = x3 * zoom_ratio_; y3 = y3 * zoom_ratio_;
            x4 = x4 * zoom_ratio_; y4 = y4 * zoom_ratio_;
        }

        cv::line(canvas_, cv::Point2i(x1, y1), cv::Point2i(x2, y2), color, line_width, cv::LINE_AA);
        cv::line(canvas_, cv::Point2i(x2, y2), cv::Point2i(x3, y3), color, line_width, cv::LINE_AA);
        cv::line(canvas_, cv::Point2i(x3, y3), cv::Point2i(x4, y4), color, line_width, cv::LINE_AA);
        cv::line(canvas_, cv::Point2i(x4, y4), cv::Point2i(x1, y1), color, line_width, cv::LINE_AA);

    }

    void Canvas::resetCanvas(const cv::Scalar &color) {
        canvas_ = cv::Mat(canvas_.rows, canvas_.cols, CV_8UC3, cv::Scalar::all(255));
    }

    Pointf<2> Canvas::transformToWorld(const Pointf<2> &pt) {
        Pointf<2> v;
        v[0] = (pt[0] - center_[0]) / resolution_;
        v[1] = -(pt[1] - center_[1]) / resolution_;
        return v;
    }

    void canvasMouseCallBack(int event, int x, int y, int flags, void *canvas) {
        Canvas *canvas_ptr = reinterpret_cast<Canvas *>(canvas);
        float x_zoomed = x / canvas_ptr->zoom_ratio_;
        float y_zoomed = y / canvas_ptr->zoom_ratio_;
        if (canvas_ptr->mouse_call_back_func_ != nullptr) {
            (*(canvas_ptr->mouse_call_back_func_))(event, x_zoomed, y_zoomed, flags, nullptr);
        } else {
            std::cout << " canvas_ptr->mouse_call_back_func_ = nullptr !" << std::endl;
            exit(0);
        }
    }


    void Canvas::setMouseCallBack(void (*func)(int, float, float, int, void *)) {
        mouse_call_back_func_ = func;
        cv::setMouseCallback(name_, canvasMouseCallBack, this);
    }

    int Canvas::show(int ms) {
        cv::imshow(name_, canvas_);
        return cv::waitKey(ms);
    }

    void Canvas::drawAxis(double x_range, double y_range, double wing_length) {

        drawLine(-x_range, 0., x_range, 0., 1, true, cv::Scalar::all(0));
        drawLine(0., -y_range, 0., y_range, 1, true, cv::Scalar::all(0));

        drawArrow(x_range, 0., 0., .5, 1, true, cv::Scalar::all(0));
        drawArrow(0., y_range, M_PI_2, .5, 1, true, cv::Scalar::all(0));

        for (double i = ceil(-x_range); i <= floor(x_range); i++) {
            if (i == 0) continue;
            drawLine(i, -wing_length, i, wing_length, 1, true, cv::Scalar::all(0));
        }

        for (double i = ceil(-y_range); i <= floor(y_range); i++) {
            if (i == 0) continue;
            drawLine(-wing_length, i, wing_length, i, 1, true, cv::Scalar::all(0));
        }

    }

    void
    Canvas::drawArrow(float x, float y, double theta, double arrow_length, int line_width, bool center_offset, const cv::Scalar &color) {
        Pointf<2> ptf = transformToPixel(x, y);
        drawArrowInt(ptf[0], ptf[1], theta, arrow_length, line_width, center_offset, color);
    }

    void
    Canvas::drawArrowInt(int x, int y, double theta, double arrow_length, int line_width, bool center_offset, const cv::Scalar &color) {
        cv::Point p1(x, y);
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        double arrow_length_i = arrow_length;
        cv::Point2i p2 = p1 + cv::Point2i(arrow_length_i * cos(theta), -arrow_length_i * sin(theta));
        cv::arrowedLine(canvas_, p1 * zoom_ratio_ + cv::Point(offset, offset), p2 * zoom_ratio_ + cv::Point(offset, offset), color, line_width, cv::LINE_AA, 0, .1);
    }

    void
    Canvas::drawArrowInt(int x1, int y1, int x2, int y2, int line_width, bool center_offset, const cv::Scalar &color) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        cv::Point p1(x1, y1);
        cv::Point p2(x2, y2);
        cv::arrowedLine(canvas_, p1 * zoom_ratio_ + cv::Point(offset, offset),
                        p2 * zoom_ratio_ + cv::Point(offset, offset), color, line_width, cv::LINE_AA, 0, .1);
    }

    void
    Canvas::drawArrowFloat(float x1, float y1, float x2, float y2, int line_width, bool center_offset, const cv::Scalar &color) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        cv::Point p1(x1, y1);
        cv::Point p2(x2, y2);
        cv::arrowedLine(canvas_, p1 * zoom_ratio_ + cv::Point(offset, offset),
                        p2 * zoom_ratio_ + cv::Point(offset, offset), color, line_width, cv::LINE_AA, 0, .1);
    }

    void Canvas::drawPathf(const Pointds<2> &pathd, int line_width, const cv::Scalar &color) {
        if (pathd.empty()) return;
        for (int i = 0; i < pathd.size() - 1; i++) {
            drawLine(pathd[i][0], pathd[i][1], pathd[i + 1][0], pathd[i + 1][1], line_width, true, color);
        }
    }

    void Canvas::drawPointfs(const Pointds<2> &pathd, double radius, int line_width, const cv::Scalar &color) {
        if (pathd.empty()) return;
        for (int i = 0; i < pathd.size() - 1; i++) {
            drawCircle(pathd[i][0], pathd[i][1], radius, line_width, true, color);
        }
        drawCircle(pathd.back()[0], pathd.back()[1], radius, line_width, true, color);
    }

    void Canvas::drawPointfs(const std::vector<PoseSE2> &path, double radius, int line_width, const cv::Scalar &color) {
        if (path.empty()) return;
        for (int i = 0; i < path.size() - 1; i++) {
            drawCircle(path[i].x(), path[i].y(), radius, line_width, true, color);
        }
        drawCircle(path.back().x(), path.back().y(), radius, line_width, true, color);
    }

    void Canvas::drawPointfs(const std::vector<PoseSE2> &path, const std::vector<double>& time_diffs, double current_time,
                             const Point2dContainer &polygon, int line_width) {
        if (path.empty()) return;
        int color_count = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            drawPolygon(path[i].x(), path[i].y(), path[i].theta(), polygon, line_width,
                        gradation_color_table_[color_count]);
            color_count++;
            color_count = color_count % gradation_color_table_.size();
        }
        drawPolygon(path.back().x(), path.back().y(), path.back().theta(), polygon, line_width,
                    gradation_color_table_[color_count]);
    }

    void Canvas::drawPolygon(double x, double y, double theta, const Point2dContainer &polygon, int line_width,
                             const cv::Scalar &color) {
        PoseSE2 pose(x, y, theta);
        const auto &rotate_matrix = Eigen::Rotation2Dd(pose.theta()).toRotationMatrix();
//        Point2dContainer global_polygon(polygon.size());
//        for (int i = 0; i < global_polygon.size(); i++) {
//            PoseSE2 offset(polygon[i], 0);
//            offset.rotateGlobal(theta);
//            global_polygon[i] = pose.position() + offset.position();
//        }
        Point2dContainer global_polygon = transformedFrom(polygon, pose);
        for (int i = 0; i < global_polygon.size() - 1; i++) {
            drawLine(global_polygon[i].x(), global_polygon[i].y(),
                     global_polygon[i + 1].x(), global_polygon[i + 1].y(),
                     line_width, true, color);
        }
        drawLine(global_polygon.front().x(), global_polygon.front().y(),
                 global_polygon.back().x(), global_polygon.back().y(),
                 line_width, true, color);
    }

    void Canvas::drawGrid(int x, int y, const cv::Vec3b &color) {
        if (x < 0 || x >= canvas_.cols / zoom_ratio_ || y < 0 || y >= canvas_.rows / zoom_ratio_) return;
        for (int i = x * zoom_ratio_; i < (x + 1) * zoom_ratio_; i++) {
            for (int j = y * zoom_ratio_; j < (y + 1) * zoom_ratio_; j++) {
                if(i < 0 || i >= canvas_.cols || j < 0 || j >= canvas_.rows) { continue; }
                canvas_.at<cv::Vec3b>(j, i) = color;
            }
        }
    }

    void
    Canvas::drawGridLine(int x1, int y1, int x2, int y2, int line_width, bool center_offset, const cv::Scalar &color) {
        double offset = center_offset ? .5 : 0;
        cv::line(canvas_,
                 cv::Point2f((x1 + offset) * zoom_ratio_, (y1 + offset) * zoom_ratio_),
                 cv::Point2f((x2 + offset) * zoom_ratio_, (y2 + offset) * zoom_ratio_),
                 color, line_width, cv::LINE_AA);
        cv::circle(canvas_, cv::Point2f((x1 + offset) * zoom_ratio_, (y1 + offset) * zoom_ratio_), 2, color, -1);
        cv::circle(canvas_, cv::Point2f((x2 + offset) * zoom_ratio_, (y2 + offset) * zoom_ratio_), 2, color, -1);

    }


    void Canvas::drawGridMap(freeNav::DimensionLength *dimension,
                             IS_OCCUPIED_FUNC<2> is_occupied,
                             const cv::Vec3b &color) {
        //if (dimension[0] > canvas_.cols * zoom_ratio_ || dimension[1] > canvas_.rows * zoom_ratio_) { return; }
        for (int i = 0; i < dimension[0]; i++) {
            for (int j = 0; j < dimension[1]; j++) {
                freeNav::Pointi<2> pt;
                pt[0] = i;
                pt[1] = j;
                if (is_occupied(pt)) {
                    //std::cout << pt << " is occ " << std::endl;
                    drawGrid(i, j, color);
                }
                //else { std::cout << pt << " is free " << std::endl; }
            }
        }
    }

    void Canvas::drawGridMap(const freeNav::MapDownSampler<2>& down_sampler, int down_sample_level) {
        const auto& dimension = down_sampler.raw_dimension_info_;
        const auto& is_occupied = down_sampler.raw_is_occupied_;
        if(down_sample_level == 0) {
            drawGridMap(dimension, is_occupied);
        } else {
            for (int i = 0; i < dimension[0]; i++) {
                for (int j = 0; j < dimension[1]; j++) {
                    freeNav::Pointi<2> pt({2, 2});
                    pt[0] = i;
                    pt[1] = j;
                    if (down_sampler.isOccupied(pt, down_sample_level)) { drawGrid(i, j); }
                }
            }
        }
    }

    void Canvas::drawPointiCircles(const freeNav::Pointis<2> &pts, const cv::Vec3b &color, int radius, int line_width) {
        for (const auto &pt : pts) {
            //drawGrid(pt.second->pt_[0], pt.second->pt_[1], color);
            drawCircleInt(pt[0], pt[1], radius, false, line_width, cv::Scalar(color[0], color[1], color[2]));
            //drawTextInt(pt.second->pt_[0], pt.second->pt_[1], std::to_string(pt.second->surface_id_).c_str(), cv::Scalar::all(0));
        }
    }

    void Canvas::drawGrids(const freeNav::Pointis<2> &pts, const cv::Vec3b &color) {
        for (const auto &pt :pts) {
            drawGrid(pt[0], pt[1], color);
        }
    }

//
//    void Canvas::drawWaveTreeNode(const freeNav::RimJump::WaveTreeNodePtr<2> &wave_tree_node_ptr) {
//        if (wave_tree_node_ptr != nullptr) {
//            drawGrid(wave_tree_node_ptr->pt_[0], wave_tree_node_ptr->pt_[1],
//                     COLOR_TABLE[wave_tree_node_ptr->wave_length_ % 30]);
//            for (const auto &nextNode_ptr : wave_tree_node_ptr->next_wave_nodes_) {
//                drawWaveTreeNode(nextNode_ptr);
//            }
//        }
//    }
//
//    void Canvas::drawWaveTree(const freeNav::RimJump::WaveTree<2> &wave_tree) {
//        if (!wave_tree.empty()) {
//            int color_count = 0;
//            for (const auto &circle : wave_tree) {
//                for (const auto &gird : circle) {
//                    drawGrid(gird->pt_[0], gird->pt_[1], COLOR_TABLE[color_count % 30]);
//                }
//                color_count++;
//            }
//        }
//    }

    void Canvas::drawPaths(const freeNav::Paths<2> &paths) {
        int color_count = 0;
        for (const auto &path : paths) {
            drawPath(path, true, COLOR_TABLE[color_count % 30]);
            color_count++;
        }
    }

    void Canvas::drawPath(const freeNav::Path<2> &path, bool center_offset, const cv::Scalar &color) {
        if (path.size() <= 1) return;
        for (int i = 0; i < path.size() - 1; i++) {
            //drawGridLine(path[i][0], path[i][1], path[i + 1][0], path[i + 1][1], 1, center_offset, color);
            drawArrowInt(path[i+1][0], path[i+1][1], path[i][0], path[i][1], 2, center_offset, color);
        }
        auto arrow_tail = path[path.size() - 2];
        auto arrow_tip = path[path.size() - 1];
        //drawArrowInt(arrow_tail[0], arrow_tail[1], arrow_tip[0], arrow_tip[1], 2, center_offset, color);
    }

    void Canvas::draw_DistMap(freeNav::DimensionLength *dimension,
                              const std::vector<PathLen>& dist_map,
                              const Pointi<2>& offset,
                              double max_dist, double min_dist) {

        for(int id=0; id<dist_map.size(); id++) {
            Pointi<2> pt = IdToPointi<2>(id, dimension) + offset;
            if(dist_map[id] == MAX<PathLen>) continue;
            if(dist_map[id] > 0) {
                drawGrid(pt[0], pt[1], cv::Vec3b(255,200*dist_map[id]/max_dist + 50,255));
            } else {
                double color = 200*dist_map[id]/(-max_dist) + 50;
                drawGrid(pt[0], pt[1], cv::Vec3b(color, color, color));
            }
        }

    }

    void Canvas::draw_Block(const freeNav::Pointi<2>& min_pt, const freeNav::Pointi<2>& max_pt) {
        freeNav::Pointi<2> max_pt_ = max_pt + Pointi<2>{1,1};
        drawLineInt(min_pt[0],  min_pt[1],  min_pt[0],  max_pt_[1], false);
        drawLineInt(min_pt[0],  min_pt[1],  max_pt_[0], min_pt[1], false);
        drawLineInt(min_pt[0],  max_pt_[1], max_pt_[0], max_pt_[1], false);
        drawLineInt(max_pt_[0], min_pt[1],  max_pt_[0], max_pt_[1], false);
    }

    void Canvas::drawEmptyGrid() {
        if (zoom_ratio_ > 5) {
            for (int i = 0; i < canvas_.cols; i += zoom_ratio_) {
                cv::line(canvas_, cv::Point2i(i, 0), cv::Point2i(i, canvas_.rows), COLOR_TABLE[10], 1);
            }
            for (int j = 0; j < canvas_.rows; j += zoom_ratio_) {
                cv::line(canvas_, cv::Point2i(0, j), cv::Point2i(canvas_.cols, j), COLOR_TABLE[10], 1);
            }
        }
    }

    void Canvas::drawTextInt(int x, int y, const char *string, const cv::Scalar &color, double scale, bool center_offset) {

        int offset = center_offset ? .5 * zoom_ratio_ : 0;


        cv::putText(canvas_,
                    cv::String(string),
                    cv::Point2i((x) * zoom_ratio_ + offset, (y) * zoom_ratio_ + offset),
                    cv::FONT_HERSHEY_COMPLEX,
                    scale, color, 2);
    }

    void Canvas::drawMultiTextInt(int x, int y, const std::vector<std::string> &strings, const cv::Scalar &color,
                                  double scale, bool center_offset) {

        for(int i=0; i<strings.size(); i++) {
            const auto& string = strings[i].c_str();
            int baseline = 0;
            cv::Size textSize = getTextSize(cv::String(string), cv::FONT_HERSHEY_COMPLEX, scale, 2, &baseline);

            cv::putText(canvas_,
                        cv::String(string),
                        cv::Point2i((x) * zoom_ratio_, (y) * zoom_ratio_ + (textSize.height)*1.2*(i+1)),
                        cv::FONT_HERSHEY_COMPLEX,
                        scale, color, 2);
        }
    }

    void Canvas::draw_ENLSVG_Extent(const std::vector<int> &extents, freeNav::DimensionLength dimen[2],
                                    double scale) {
        freeNav::DimensionLength internal_dimen[2];
        internal_dimen[0] = dimen[0] + 1; // ENL_SVG internal setting
        internal_dimen[1] = dimen[1] + 2; // ENL_SVG internal setting
        for (int i = 0; i < extents.size(); i++) {
            const freeNav::Pointi<2> pt = freeNav::IdToPointi<2>(i, internal_dimen);
            if (isOutOfBoundary(pt, dimen)) continue;
            drawTextInt(pt[0], pt[1], std::to_string(extents[i]).c_str(), {0, 0, 255}, scale);
        }
    }

}







