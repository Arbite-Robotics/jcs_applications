// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef HOPPER_2D_SIMPLE_KINE_H_
#define HOPPER_2D_SIMPLE_KINE_H_

#include "hopper_2d.h"
#include "helpers.h"
#include <cmath>

class hopper_2d_simple_kine : public hopper_2d {
public:
    hopper_2d_simple_kine();
    ~hopper_2d_simple_kine();

    int render();
    int startup(double sample_rate_hz);
    int step_rt_run();
    int step_rt_init();
    bool can_start();

private:
    // Tip xy
    helpers::vec2 x_tip_;
    helpers::vec2 x_tip_mod_;

    // Sine params so we can oscillate about the tool tip
    struct sine_params {
        double amplitude;
        double frequency;
        double dt;
        unsigned int t;
        sine_params(double _dt) : amplitude(0.0), frequency(0.0), dt(_dt) {
            t = 0;
        }
        double step_rt() {
            t++;
            return amplitude*sin(2.0*M_PI*frequency*(double)t*dt);
        }
    };
    sine_params sine_x_;
    sine_params sine_y_;

    // Joint positions
    helpers::vec2 th_;
    helpers::vec2 kin_th_;
    // Joint limits
    helpers::vec2 j0_limits_;
    helpers::vec2 j1_limits_;
    // Joint controller gains
    helpers::vec2 j0_gain_;
    helpers::vec2 j1_gain_;

    // Output signal vector index map
    enum class osig_map {
        j0_th = 0,
        j1_th = 1,
        j0_gain_p = 2,
        j0_gain_d = 3,
        j1_gain_p = 4,
        j1_gain_d = 5
    };

    bool outputs_active_;

    // Input signal vector index map
    enum class isig_map {
        j0_th = 0,
        j1_th = 1,
    };
};

#endif