// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef HOPPER_2D_VIRTUAL_MODEL_CTL_H_
#define HOPPER_2D_VIRTUAL_MODEL_CTL_H_

#include "hopper_2d.h"
#include "helpers.h"

class hopper_2d_virtual_model_ctl : public hopper_2d {
public:
    hopper_2d_virtual_model_ctl();
    ~hopper_2d_virtual_model_ctl();

    int render();
    int startup(double sample_rate_hz);
    int step_rt_run();
    int step_rt_init();
    bool can_start();

private:
    enum class hop_state {
        loading_s,
        unloading_s,
        flight_s,
        stop_s
    };
    hop_state hop_state_;
    bool stop_requested_;
    void next_hop_rt(hop_state next);
    void print_state();
    void maybe_next_flight();
    void maybe_next_unloading();
    void maybe_next_loading();

    helpers::controller_pd cart_pd_x_;
    helpers::controller_pd cart_pd_y_;

    // Lengths
    double x_tip_y_loading_;
    double x_tip_y_loading_thresh_;
    double x_tip_y_unloading_;
    // double x_tip_y_unloading_thresh_;
    double x_tip_y_flight_to_loading_thresh_;

    helpers::vec2 x_tip_;
    helpers::vec2 v_tip_;
    helpers::vec2 x_tip_ref_;
    helpers::vec2 f_tip_cmd_;
    helpers::vec2 x_tip_y_coord_limits_;
    helpers::vec2 x_tip_x_coord_limits_;
    helpers::mat22 J;

    // Cartesian PD gains
    helpers::vec2 loading_gain_;
    helpers::vec2 unloading_gain_;

    double ff_mass_kg_;
    bool do_ff_mass_;
    double f_tip_y_unloading_;

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
