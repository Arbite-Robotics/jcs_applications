// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef TOOL_MC_CURRENT_TEST_H_
#define TOOL_MC_CURRENT_TEST_H_

#include "jcs_tool_if.h"
#include <vector>
#include <string>
#include "ramp.h"
#include "rotate.h"
#include "recorder.h"


class tool_mc_current_test : public jcs_tool_if {
public:
    tool_mc_current_test(std::string name, jcs::jcs_host* host);
    ~tool_mc_current_test();

    int load_config(std::string tool_config);

    int step_startup_rt();
    int step_rt();
    int step_shutdown_rt();
    int step_parameter_startup();
    int step_parameter();
    int step_parameter_shutdown();

private:
    enum class behaviour_state {
        ramp_to_current_s,
        rotate_axis_s
    };
    behaviour_state state_;

    ramp* i_ramp_;
    rotate* i_rotate_;

    float ramp_i_energise_;
    float ramp_i_ramp_time_;
    float ramp_i_dwell_time_;

    float rotate_start_;
    float rotate_n_rotations_;
    float rotate_time_;

    // Per tick signal storage
    std::vector<float> signals_out_f_;
    std::vector<float> signals_in_f_;
    unsigned int host_sigs_out_sz_;
    unsigned int host_sigs_in_sz_;
    unsigned int sig_index_i_d_;
    unsigned int sig_index_th_m_;
    unsigned int sig_index_w_m_;
    // Signal recorders
    recorder* rec_fixed_;
    recorder* rec_rotate_;
    std::vector<std::string> signals_record_header_;
    std::vector<float> signals_record_f_;

    int64_t t_0_data_;
};

#endif