#include "lamapf_and_gazebo/dwa_local_controller.h"


// ============================ 简单示例 ============================
int main() {
    DWASimple planner;
    DWAParams params;
    planner.initialize(params, MotionConfig());

    Pointf<3> cur{0., 0., 0.};
    Pointf<3> cur_vel{0., 0., 0.};
    Pointf<3> goal{.5, .5, 0.};

    for (int i = 0; i < 100; ++i) {
        Pointf<3> cmd = planner.computeVelocity(cur, cur_vel, goal);
        // apply cmd to simple integrator (for demonstration only)
        float step = params.dt;
        cur[0] += cmd[0] * cos(cur[2]) * step;
        cur[1] += cmd[0] * sin(cur[2]) * step;
        cur[2] += cmd[2] * step;

        cur_vel[0] = cmd[0];
        cur_vel[2] = cmd[2];

        // update cur_vel with simple first-order response (cap by acc)
        // float dv = cmd[0] - cur_vel[0];
        // float dv_max = planner.motion_config_.max_a_x * step;
        // if (dv > dv_max) dv = dv_max;
        // if (dv < -dv_max) dv = -dv_max;
        // cur_vel[0] += dv;

        // float dw = cmd[2] - cur_vel[2];
        // float dw_max = planner.motion_config_.max_a_w * step;
        // if (dw > dw_max) dw = dw_max;
        // if (dw < -dw_max) dw = -dw_max;
        // cur_vel[2] += dw;

        float dist_to_goal = hypot(goal[0] - cur[0], goal[1] - cur[1]);        
        float dist_to_heading = fabs(shortestAngularDistance(cur[2], goal[2]));

        std::cout << "iter " << i
                  << ", cmd_v/w = " << cmd[0] << "/" << cmd[2]
                  << ", pos=(" << cur[0] << "," << cur[1] << "," << cur[2] <<")"
                  << ", dist_to_goal =" << dist_to_goal << ", dist_to_heading = " << dist_to_heading << std::endl;

        if (dist_to_goal <= params.dist_tolerance && dist_to_heading <= params.heading_tolerance) {
            std::cout << "Reached goal\n";
            break;
        }
    }

    return 0;
}
