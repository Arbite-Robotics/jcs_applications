// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef HOPPER_2D_VMC_SIMPLE_XY_H_
#define HOPPER_2D_VMC_SIMPLE_XY_H_

#include "hopper_2d.h"
#include "helpers.h"

class hopper_2d_vmc_simple_xy_ctl : public hopper_2d {
public:
    hopper_2d_vmc_simple_xy_ctl();
    ~hopper_2d_vmc_simple_xy_ctl();

    int render();
    int startup(double sample_rate_hz);
    int step_rt_run();
    int step_rt_init();
    bool can_start();

private:
    helpers::controller_pd cart_pd_x_;
    helpers::controller_pd cart_pd_y_;

    helpers::vec2 x_tip_;
    helpers::vec2 v_tip_;
    helpers::vec2 x_tip_ref_;
    helpers::vec2 f_tip_cmd_;
    helpers::vec2 x_tip_x_coord_limits_;
    helpers::vec2 x_tip_y_coord_limits_;
    helpers::mat22 J;

    // Joint state
    helpers::vec2 th_;
    helpers::vec2 thd_;
    helpers::vec2 tau_cmd_;

    // Output signal vector index map
    enum class osig_map {
        j0_tau = 0,
        j1_tau = 1,
    };

    bool outputs_active_;

    // Input signal vector index map
    enum class isig_map {
        j0_th  = 0,
        j0_thd = 1,
        j1_th  = 2,
        j1_thd = 3,
        m0_w_m = 4,
        m1_w_m = 5
    };

    enum class thd_source {
        joint_s,
        motor_s
    };
    thd_source thd_source_;

};

#endif
