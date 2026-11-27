//
// Created by yaozhuo on 2022/2/26.
//

#include "./3d_viewer.h"
#include "../../dependencies/color_table.h"

namespace freeNav {

    bool window_running = true;

    bool Viewer3DBase::viewer_set_start = true;

    Pointi<3> Viewer3DBase::start_;

    Pointi<3> Viewer3DBase::target_;

    std::mutex three_dimension_mutex_;

    int Viewer3DBase::current_top_level;

    int Viewer3DBase::current_path_id = 0;

    int Viewer3DBase::max_time_index_ = 0;


    void Viewer3DBase::DrawPoint(double px, double py, double pz) {
        glPointSize(8.0f);
        glBegin(GL_POINTS);
        //float xf = px*2*halfsize+min_x, yf = py*2*halfsize+min_y, zf = pz*2*halfsize+min_z;
        glColor3f(.1, .1, .1);
        glVertex3f(px, py, pz);
        glEnd();
    }

    void Viewer3DBase::DrawLine(const Pointi<3> &p1, const Pointi<3> &p2, float r, float g, float b) {
//    glLineWidth(4);
//    glBegin(GL_LINES);
//    glColor3f(r,g,b);
//    glVertex3f(p1[0], p1[1], p1[2]);
//    glVertex3f(p2[0], p2[1], p2[2]);
//    glEnd();
        DrawLine(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], r, g, b);
    }

    void Viewer3DBase::DrawLine(int x1, int y1, int z1, int x2, int y2, int z2, float r, float g, float b) {
        glLineWidth(4);
        glBegin(GL_LINES);
        glColor3f(r, g, b);
        glVertex3f(x1, y1, z1);
        glVertex3f(x2, y2, z2);
        glEnd();
    }


    void Viewer3DBase::DrawVoxel(const Pointi<3> &pt, const cv::Vec3b &color) {
        DrawPoint(pt[0], pt[1], pt[2]);
    }

    void Viewer3DBase::DrawVoxels(const Pointis<3> &pts, const cv::Vec3b &color) {
        for (const auto &pt : pts) {
            DrawVoxel(pt, color);
        }
    }

    void Viewer3DBase::DrawBlock(const Pointi<3> &p1, const Pointi<3> &p2, float r, float g, float b) {
        DrawLine(p1[0], p1[1], p1[2], p1[0], p2[1], p1[2], r, g, b);
        DrawLine(p1[0], p1[1], p1[2], p2[0], p1[1], p1[2], r, g, b);
        DrawLine(p2[0], p1[1], p1[2], p2[0], p2[1], p1[2], r, g, b);
        DrawLine(p1[0], p2[1], p1[2], p2[0], p2[1], p1[2], r, g, b);

        DrawLine(p1[0], p1[1], p2[2], p1[0], p2[1], p2[2], r, g, b);
        DrawLine(p1[0], p1[1], p2[2], p2[0], p1[1], p2[2], r, g, b);
        DrawLine(p2[0], p1[1], p2[2], p2[0], p2[1], p2[2], r, g, b);
        DrawLine(p1[0], p2[1], p2[2], p2[0], p2[1], p2[2], r, g, b);


        DrawLine(p1[0], p1[1], p1[2], p1[0], p1[1], p2[2], r, g, b);
        DrawLine(p1[0], p2[1], p1[2], p1[0], p2[1], p2[2], r, g, b);
        DrawLine(p2[0], p1[1], p1[2], p2[0], p1[1], p2[2], r, g, b);
        DrawLine(p2[0], p2[1], p1[2], p2[0], p2[1], p2[2], r, g, b);


    }

    void Viewer3DBase::DrawLine(double x1, double y1, double z1, double x2, double y2, double z2, double line_width) {
        glLineWidth(line_width);
        glBegin(GL_LINES);
        glColor3f(1, 0, 0);
        glVertex3f(x1, y1, z1);
        glVertex3f(x2, y2, z2);
        glEnd();
    }

    void Viewer3DBase::heightMapColor(double h, double &r, double &g, double &b) {
        double s = 1.0;
        double v = 1.0;
        h -= floor(h);
        h *= 6;
        int i;
        double m, n, f;
        i = floor(h);
        f = h - i;
        if (!(i & 1)) {
            f = 1 - f;
        }
        m = v * (1 - s);
        n = v * (1 - s * f);
        switch (i) {
            case 6:
            case 0:
                r = v;
                g = n;
                b = m;
                break;
            case 1:
                r = n;
                g = v;
                b = m;
                break;
            case 2:
                r = m;
                g = v;
                b = n;
                break;
            case 3:
                r = m;
                g = n;
                b = v;
                break;
            case 4:
                r = n;
                g = m;
                b = v;
                break;
            case 5:
                r = v;
                g = m;
                b = n;
                break;
            default:
                r = 1;
                g = 0.5;
                b = 0.5;
                break;
        }
    }


    Viewer3DBase::Viewer3DBase() {
        mImageWidth = 640;
        mImageHeight = 480;
        mViewpointX = 0.;
        mViewpointY = 0.;
        mViewpointZ = 20.;
        mViewpointF = 1000.;
        under_planning = false;
    }

    Viewer3DBase::~Viewer3DBase() {}

    void MyHandler3D::Keyboard(pangolin::View &view_, unsigned char key, int x, int y, bool pressed) {
        three_dimension_mutex_.lock();
        if (key == 32) { // 32 = space
            window_running = false;
            std::cout << "** terminated from key board" << std::endl;
            exit(0);
        }
        three_dimension_mutex_.unlock();
    }

    void Viewer3DBase::DrawPath(const Path<3> &path, float r, float g, float b) {
        if (path.size() < 2) return;
        for (int i = 0; i < path.size() - 1; i++) {
            DrawLine(path[i], path[i + 1], r, g, b);
        }
    }

    void Viewer3DBase::DrawOctoMap(OctoMapLoader &octomap) {
        three_dimension_mutex_.lock();
        //octomap::OcTree octo_map_(*(buff_space->octo_map));
        double max_z = octomap.max_z, min_z = octomap.min_z;
        int depth_offset = octomap.depth_offset;
        three_dimension_mutex_.unlock();
        octomap::OcTree::tree_iterator it = octomap.getOctoMap()->begin_tree();
        octomap::OcTree::tree_iterator end = octomap.getOctoMap()->end_tree();
        int counter = 0;
        double occ_thresh = 0.9;
        int level = 16;
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        //// DRAW OCTOMAP BEGIN //////
        double stretch_factor = 128 / (1 - occ_thresh); //occupancy range in which the displayed cubes can be
        /***NOTICE: small block of obstacle is not shown in the pangolin window ***/
        for (; it != end; ++counter, ++it) {
            // it refer to all depth leaf nodes !
            // the higher depth_offset, the lower depth
            if (level - depth_offset != it.getDepth()) {
                continue;
            }
            double occ = it->getOccupancy();
            if (occ < occ_thresh) {
                continue;
            }
            float halfsize = it.getSize() / 2.0;
            float x = it.getX();
            float y = it.getY();
            float z = it.getZ();
            DrawVoxel(x, y, z, halfsize, min_z, max_z);
        }
    }

    void Viewer3DBase::DrawOctoMapGrid(OctoMapLoader &octomap) {
        auto dimension = octomap.getDimensionInfo();
        for (int i = 0; i < dimension[0]; i++) {
            for (int j = 0; j < dimension[1]; j++) {
                for (int k = 0; k < dimension[2]; k++) {
                    Pointi<3> pt;
                    pt[0] = i, pt[1] = j, pt[2] = k;
                    if (octomap.isOccupied(pt)) {
                        DrawVoxel(i, j, k, .5, 0, dimension[2]);
                    }
                }
            }
        }
    }

    void Viewer3DBase::DrawVoxels(const Pointis<3> &voxels, DimensionLength *dim) {
        for (const auto &voxel : voxels) {
            DrawVoxel(voxel[0], voxel[1], voxel[2], .5, 0, dim[2]);
        }
    }

    void Viewer3DBase::KeyBoardCallBack_W() {
        three_dimension_mutex_.lock();
        if (viewer_set_start) start_[1] += 1;
        else target_[1] += 1;
        if (current_top_level < max_time_index_ - 1) {
            current_top_level++;
        } else {
            current_top_level = max_time_index_ - 1;
        }
        three_dimension_mutex_.unlock();
    }

    void Viewer3DBase::KeyBoardCallBack_S() {
        three_dimension_mutex_.lock();
        if (viewer_set_start) start_[1] -= 1;
        else target_[1] -= 1;
        if (current_top_level > 0) { current_top_level--; }
        else { current_top_level = 0; }
        three_dimension_mutex_.unlock();
    }

    void Viewer3DBase::KeyBoardCallBack_A() {
        three_dimension_mutex_.lock();
        if (viewer_set_start) start_[0] -= 1;
        else target_[0] -= 1;
        if (current_path_id > 0) current_path_id--;
        three_dimension_mutex_.unlock();
    }

    void Viewer3DBase::KeyBoardCallBack_D() {
        three_dimension_mutex_.lock();
        if (viewer_set_start) start_[0] += 1;
        else target_[0] += 1;
        current_path_id++;
        three_dimension_mutex_.unlock();
    }

    void Viewer3DBase::KeyBoardCallBack_Q() {
        three_dimension_mutex_.lock();
        if (viewer_set_start) start_[2] -= 1;
        else target_[2] -= 1;
        three_dimension_mutex_.unlock();
    }

    void Viewer3DBase::KeyBoardCallBack_E() {
        three_dimension_mutex_.lock();
        if (viewer_set_start) start_[2] += 1;
        else target_[2] += 1;
        three_dimension_mutex_.unlock();
    }

    void Viewer3DBase::DrawBound(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z) {

        DrawLine(max_x, min_y, min_z, min_x, min_y, min_z);
        DrawLine(min_x, max_y, min_z, min_x, min_y, min_z);
        DrawLine(max_x, min_y, min_z, max_x, max_y, min_z);
        DrawLine(min_x, max_y, min_z, max_x, max_y, min_z);

        DrawLine(max_x, min_y, max_z, min_x, min_y, max_z);
        DrawLine(min_x, max_y, max_z, min_x, min_y, max_z);
        DrawLine(max_x, min_y, max_z, max_x, max_y, max_z);
        DrawLine(min_x, max_y, max_z, max_x, max_y, max_z);

        DrawLine(min_x, min_y, min_z, min_x, min_y, max_z);
        DrawLine(min_x, max_y, min_z, min_x, max_y, max_z);
        DrawLine(max_x, min_y, min_z, max_x, min_y, max_z);
        DrawLine(max_x, max_y, min_z, max_x, max_y, max_z);
    }

    void Viewer3DBase::init() {
        std::cout << "-- start 3D Viewer thread... " << std::endl;
        /*
        (0.0, 1.0) means the panel has the same high as the window
        (0, pangolin::Attach::Pix(175)) set the width and the start of the panel
        */
        pangolin::CreateWindowAndBind("3D Viewer", 1024, 768);
        pangolin::CreatePanel("menu").SetBounds(.0, 1., 0.0, pangolin::Attach::Pix(175));


        pangolin::RegisterKeyPressCallback('w', KeyBoardCallBack_W);
        pangolin::RegisterKeyPressCallback('a', KeyBoardCallBack_A);
        pangolin::RegisterKeyPressCallback('s', KeyBoardCallBack_S);
        pangolin::RegisterKeyPressCallback('d', KeyBoardCallBack_D);
        pangolin::RegisterKeyPressCallback('q', KeyBoardCallBack_Q);
        pangolin::RegisterKeyPressCallback('e', KeyBoardCallBack_E);


    }

    void Viewer3DBase::Run() {
        //
    }

    void
    Viewer3DBase::DrawVoxel(double x, double y, double z, float halfsize, double r, double g, double b, bool filled) {
        if (filled) {
            glBegin(GL_TRIANGLES);
            //Front
            glColor3d(r, g, b);
            glVertex3f(x - halfsize, y - halfsize, z - halfsize); // - - - 1
            glVertex3f(x - halfsize, y + halfsize, z - halfsize); // - + - 2
            glVertex3f(x + halfsize, y + halfsize, z - halfsize); // + + -3

            glVertex3f(x - halfsize, y - halfsize, z - halfsize); // - - -
            glVertex3f(x + halfsize, y + halfsize, z - halfsize); // + + -
            glVertex3f(x + halfsize, y - halfsize, z - halfsize); // + - -4

            //Back
            glVertex3f(x - halfsize, y - halfsize, z + halfsize); // - - + 1
            glVertex3f(x + halfsize, y - halfsize, z + halfsize); // + - + 2
            glVertex3f(x + halfsize, y + halfsize, z + halfsize); // + + + 3

            glVertex3f(x - halfsize, y - halfsize, z + halfsize); // - - +
            glVertex3f(x + halfsize, y + halfsize, z + halfsize); // + + +
            glVertex3f(x - halfsize, y + halfsize, z + halfsize); // - + + 4

            //Left
            glVertex3f(x - halfsize, y - halfsize, z - halfsize); // - - - 1
            glVertex3f(x - halfsize, y - halfsize, z + halfsize); // - - + 2
            glVertex3f(x - halfsize, y + halfsize, z + halfsize); // - + + 3

            glVertex3f(x - halfsize, y - halfsize, z - halfsize); // - - -
            glVertex3f(x - halfsize, y + halfsize, z + halfsize); // - + +
            glVertex3f(x - halfsize, y + halfsize, z - halfsize); // - + - 4

            //Right
            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);

            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);

            //top
            glVertex3f(x - halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);

            glVertex3f(x - halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);
            glVertex3f(x - halfsize, y - halfsize, z + halfsize);

            //bottom
            glVertex3f(x - halfsize, y + halfsize, z - halfsize);
            glVertex3f(x - halfsize, y + halfsize, z + halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);

            glVertex3f(x - halfsize, y + halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);
            glEnd();
        }
#define DRAW_LINE 1
#if DRAW_LINE
        if (show_line_for_voxel) {
            glLineWidth(2);
            halfsize += .01 * halfsize;
            glBegin(GL_LINES);
            glColor3f(0, 0, 0);
            // x
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);
            glVertex3f(x - halfsize, y + halfsize, z + halfsize);

            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x - halfsize, y - halfsize, z - halfsize);

            glVertex3f(x + halfsize, y - halfsize, z + halfsize);
            glVertex3f(x - halfsize, y - halfsize, z + halfsize);

            glVertex3f(x + halfsize, y + halfsize, z - halfsize);
            glVertex3f(x - halfsize, y + halfsize, z - halfsize);

            // y
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);

            glVertex3f(x - halfsize, y - halfsize, z - halfsize);
            glVertex3f(x - halfsize, y + halfsize, z - halfsize);

            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);

            glVertex3f(x - halfsize, y - halfsize, z + halfsize);
            glVertex3f(x - halfsize, y + halfsize, z + halfsize);

            // z
            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);

            glVertex3f(x - halfsize, y - halfsize, z + halfsize);
            glVertex3f(x - halfsize, y - halfsize, z - halfsize);

            glVertex3f(x + halfsize, y + halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);

            glVertex3f(x - halfsize, y + halfsize, z + halfsize);
            glVertex3f(x - halfsize, y + halfsize, z - halfsize);

            glEnd();
        }
#endif
    }

    void Viewer3DBase::DrawVoxel(double x, double y, double z, float halfsize, double min_z, double max_z) {
        double h = (std::min(std::max((max_z - z) / (max_z - min_z), 0.0), 1.0)) * 0.8;
        double r = .8, g = .8, b = .8;
        if (!show_grey_voxel) heightMapColor(h, r, g, b);
        else {
            r -= h;
            g = r;
            b = r;
        }
        DrawVoxel(x, y, z, halfsize, r, g, b);
    }

    void Viewer3DBase::DrawGrid() {
        glLineWidth(5);
        glBegin(GL_LINES);
        glLineWidth(1);
        glColor3f(0.5, 0.5, 0.5); //gray
        int size = 20;
        for (int i = -size; i <= size; i++) {
            glVertex3f(i, size, .0);
            glVertex3f(i, -size, .0);
            glVertex3f(size, i, .0);
            glVertex3f(-size, i, .0);
        }
        glEnd();
    }

    void Viewer3DBase::DrawArrow(double x1, double y1, double x2, double y2, double z, double head_width,
                                 double line_width) {
        // the center pole
        DrawLine(x1, y1, z, x2, y2, z, line_width);
    }

}

