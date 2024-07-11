// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_MC_ENCODER_H_
#define GUI_MC_ENCODER_H_

#include "jcs_host.h"
#include "imgui.h"
#include <array>
#include <string>
#include "helpers.h"
#include "gui_type_base.h"

//////////////////////////////////////////////////////////////////////
class gui_mc_encoder : public gui_type_base {
public:
    gui_mc_encoder(jcs::jcs_host* host, std::string const& target_device);
    ~gui_mc_encoder() {}

    int startup();
    int step_rt();
    int render();

private:
    bool is_ready_;
    float i_d_alignment_;
    float i_d_alignment_ramp_time_ms_;
    float encoder_0_position_offset_;


    int render_zero_encoder();
    int ready_test();
};

#endif