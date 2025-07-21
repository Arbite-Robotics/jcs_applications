// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_HOST_INPUT_STIM_H_
#define GUI_HOST_INPUT_STIM_H_

#include "jcs_host.h"
#include "gui_type_base.h"
#include "gui_interface.h"
#include "gui_device_host_base.h"
#include <vector>
#include <string>
#include "imgui.h"

#include "gui_stimulus.h"

class gui_host_input_stimulus : public gui_type_base, public gui_device_host_base {
public:
    gui_host_input_stimulus(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device);
    ~gui_host_input_stimulus() {}

    int startup();
    int step_rt();
    int step_rt_always();
    int render();

private:
    enum class state {
        off_s,
        running_s,
    };
    state state_;

    std::vector<float> f32_isignal_store_;

    struct channel {
        gui_stimulus* input_stimulus_;
        int input_combo_idx_;
        bool is_active_;
        channel(double base_freq_hz);
    };
    std::vector<channel*> channels_;

};

#endif
