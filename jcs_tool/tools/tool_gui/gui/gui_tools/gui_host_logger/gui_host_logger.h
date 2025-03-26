// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_HOST_LOGGER_H_
#define GUI_HOST_LOGGER_H_

#include "jcs_host.h"
#include "gui_type_base.h"
#include "gui_interface.h"
#include "gui_device_host_base.h"
#include <vector>
#include <string>
#include "imgui.h"

#include "sampler.h"

class gui_host_logger : public gui_type_base, public gui_device_host_base {
public:
    gui_host_logger(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device);
    ~gui_host_logger() {}

    int startup();
    int step_rt();
    int step_rt_always();
    int render();

private:
    sampler sampler_;
    std::vector<float> f32_output_signal_store_;
};

#endif
