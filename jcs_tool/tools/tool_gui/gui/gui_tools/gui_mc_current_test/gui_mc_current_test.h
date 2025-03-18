// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_MC_CURRENT_TEST_H_
#define GUI_MC_CURRENT_TEST_H_

#include <string>
#include "jcs_host.h"
#include "gui_type_base.h"
#include "gui_interface.h"
#include "sampler.h"
#include "helpers.h"

#include "ramp.h"
#include "rotate.h"

class gui_mc_current_test : public gui_type_base {
public:
    gui_mc_current_test(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device);
    ~gui_mc_current_test() {}

    int startup();
    int step_rt();
    int step_rt_always();
    int render();    

private:
    enum class state {
        off_s,
        ramp_to_current_s,
        rotate_s,
        finish_s
    };
    state state_;

    int ready_test();
    void get_test_parameters();

    bool is_ready_;
    bool can_start_;

    float test_current_;
    float ramp_time_s_;
    float dwell_time_s_;
    float rotate_speed_rads_;

    sampler sampler_;
    ramp i_ramp_;
    rotate i_rotate_;

    std::vector<std::string> required_input_signal_names_;
    std::vector<std::string> required_output_signal_names_;

    std::vector<float> f32_input_signal_store_;
    std::vector<float> f32_output_signal_store_;

    helpers::combo_source signal_in_source_i_d_;
    helpers::combo_source signal_in_source_th_m_;
    helpers::combo_source signal_in_source_w_m_;

    int signal_out_th_m_0_idx_;
    int signal_out_w_m_0_idx_;
    int signal_out_i_d_idx_;
};

#endif
