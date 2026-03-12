// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_MC_TUNE_H_
#define GUI_MC_TUNE_H_

#include "jcs_host.h"
#include "imgui.h"
#include <array>
#include <string>
#include <vector>
#include "gui_type_base.h"
#include "gui_interface.h"

/////////////////////////////////////////////////////////////////////////////////////////////
class test_resistance {
public:
    // Parameters
    float amplitude;
    int   time_ms;
    int   ramp_ms;
    // Result
    float result;

    test_resistance();
    void render_ui();
    int execute(jcs::jcs_host* host, std::string const& target);
};

/////////////////////////////////////////////////////////////////////////////////////////////
class test_inductance {
public:
    std::string axis;  // "d" or "q"

    // Parameters
    float bias;
    float amplitude;
    float frequency;
    int   time_ms;
    int   ramp_ms;
    int   settle_ms;
    // Result
    float result;

    test_inductance(std::string const& axis);
    void render_ui();
    int execute(jcs::jcs_host* host, std::string const& target);
    void copy_result_from(test_inductance const& other);
};

/////////////////////////////////////////////////////////////////////////////////////////////
struct controller_gains {
    float kp;
    float ki;
    controller_gains() : kp(0.0f), ki(0.0f) {}
};
controller_gains compute_controller_gains(float bw_rads, float resistance, float inductance);
void render_controller_gains(std::string const& axis, controller_gains const& gains);

/////////////////////////////////////////////////////////////////////////////////////////////
class test_step_response {
public:
    std::string axis;
    float amplitude;
    int   time_ms;

    std::vector<float> data;
    std::vector<float> x;

    bool   plot_cursors;
    double cursor_tag[4];

    test_step_response(std::string const& axis);
    void render_ui();
    void render_plot();
    int execute(jcs::jcs_host* host, std::string const& target);
};

/////////////////////////////////////////////////////////////////////////////////////////////
class gui_mc_tune : public gui_type_base {
public:
    gui_mc_tune(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device);
    ~gui_mc_tune() {}

    int startup();
    int step_rt();
    int render();

private:
    bool is_ready_;
    uint32_t pole_pairs_;

    // Tests
    test_resistance test_r_;
    // [0] = Ld, [1] = Lq
    std::array<test_inductance, 2> test_l_;

    bool lq_equals_ld_;

    // Current controller
    float i_ctl_bw_hz_;
    float i_ctl_bw_rads_;
    std::array<controller_gains, 2> controller_gains_;

    // Step response tests
    std::array<test_step_response, 2> test_step_;

    // Device state helpers
    int ready_test();
    int standby_test();
};

#endif