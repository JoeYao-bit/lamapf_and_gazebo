#include "lamapf_and_gazebo/common_interfaces.h"

#include "lamapf_and_gazebo/controller.h"

Pointi<2> pt1, pt2;
auto mpc = std::make_shared<TwoPhaseLineFollowController>(MotionConfig());

Pointf<3> start_ptf = {.2, .5, M_PI/6}, target_ptf{1., 0, 0};
Pointf<3> cur_ptf = start_ptf, cur_vel = {0,0,0};
Pose<int, 2> start_pose = PtfToPoseInt(start_ptf, PtfToGridPicOnly), target_pose = PtfToPoseInt(target_ptf, PtfToGridPicOnly);
std::vector<Pointf<3>> history_ptfs;
float time_interval = 0.1;
bool set_pt1 = true;

int main() {

    // ConstantLineFollowController
    // TwoPhaseLineFollowController
    // MPCLineFollowController
    dim[0] = 30;
    dim[1] = 30;


    mpc->pt1_ = Pointf<2>{start_ptf[0],  start_ptf[1]};
    mpc->pt2_ = Pointf<2>{target_ptf[0], target_ptf[1]}; // update target line

    float zoom_ratio = std::max(1., std::min(1000./dim[0], 1000./dim[1])); 
    Canvas canvas("LA-MAPF visualization", dim[0], dim[1], 1./reso, zoom_ratio);
    canvas.resolution_ = 1./reso;
    bool wait = false;

    auto callback = [](int event, int x, int y, int flags, void *) {
        if(event == cv::EVENT_LBUTTONDOWN) {
            if(set_pt1) {
                pt1[0] = x;
                pt1[1] = y;
                set_pt1 = false;
                std::cout << "get point pt1 " << x << ", " << y << std::endl;
            } else {
                pt2[0] = x;
                pt2[1] = y;
                set_pt1 = true;
                std::cout << "get point pt2 " << x << ", " << y << std::endl;
                auto temp_ptf1 = GridToPtfPicOnly(pt1);
                auto temp_ptf2 = GridToPtfPicOnly(pt2);
                mpc->pt1_[0] = temp_ptf1[0];
                mpc->pt1_[1] = temp_ptf1[1];
                mpc->pt2_[0] = temp_ptf2[0];
                mpc->pt2_[1] = temp_ptf2[1];
                mpc->finish_rotate_ = false;
                mpc->finish_move_ = false;
                cur_ptf = mpc->pt1_;
                start_pose.pt_ = pt1;
                start_pose.pt_ = pt2;
                cur_vel = freeNav::Pointf<3>{0,0,0};
            }
        }
    };

    canvas.setMouseCallBack(callback);

    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();

        canvas.drawCircleInt(start_pose.pt_[0], start_pose.pt_[1], 4);
        canvas.drawCircleInt(target_pose.pt_[0], target_pose.pt_[1], 4);
        
        if(!wait) {
            if(!reachTarget(cur_ptf, target_ptf)) {
                cur_vel = mpc->calculateCMD(cur_ptf, cur_vel, time_interval);
                cur_ptf = updateAgentPose(cur_ptf, cur_vel, time_interval);
                history_ptfs.push_back(cur_ptf);

                Pose<int, 2> cur_pose = PtfToPoseInt(cur_ptf, PtfToGridPicOnly);
                std::cout << "cmd vel = " << cur_vel << ", cur pose = " << cur_ptf << std::endl;
            } else {
                std::cout << "reach target" << std::endl;
            }
            std::cout << std::endl;
        }
        // draw historical pose
        for(int i=0; i<history_ptfs.size(); i++) {
            Pointf<3> ptf = history_ptfs[i];
            auto temp_pose = PtfToPoseInt(ptf, PtfToGridPicOnly);
            //canvas.drawCircleFloat(ptf[0], ptf[1], 1, true, 1);
            canvas.drawArrow(ptf[0], ptf[1], ptf[2], 4);
        }
        // draw current pose
        canvas.drawCircleFloat(cur_ptf[0], cur_ptf[1], 4, true, 1, COLOR_TABLE[0]);
        canvas.drawArrow(cur_ptf[0], cur_ptf[1], cur_ptf[2], 4);
    
        char key = canvas.show(time_interval*1e3);
        if(key == 32) {
            wait = !wait;
        }
    }

    return 0;
}