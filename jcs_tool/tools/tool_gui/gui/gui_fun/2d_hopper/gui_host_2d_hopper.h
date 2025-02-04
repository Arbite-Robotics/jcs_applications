// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_HOST_2D_HOPPER_H_
#define GUI_HOST_2D_HOPPER_H_

#include "jcs_host.h"
#include "gui_type_base.h"
#include "gui_device_host_base.h"
#include <vector>
#include <string>
#include "hopper_2d.h"
#include "imgui.h"
#include "helpers.h"

class gui_host_2d_hopper : public gui_type_base, public gui_device_host_base {
public:
    gui_host_2d_hopper(jcs::jcs_host* host, std::string const& target_device);
    ~gui_host_2d_hopper() {}

    int startup();
    int step_rt();
    int step_rt_always();
    int render();

private:
    std::vector<hopper_2d*> hopper_control_source_;
    int active_idx_;
    std::vector<std::string> source_names_;

    std::vector<float> f32_osignal_store_;
    std::vector<float> f32_isignal_store_;
    std::vector<std::string> f32_output_signal_names_;
    std::vector<std::string> f32_input_signal_names_;

    enum class controller_state {
        stopped_s,
        init_s,
        running_s,
    };
    controller_state controller_state_;
};

#endif
