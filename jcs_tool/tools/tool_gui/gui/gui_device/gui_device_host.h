// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_DEVICE_HOST_H_
#define GUI_DEVICE_HOST_H_

#include <string>
#include "gui_device_base.h"
#include "gui_interface.h"
#include "gui_device_host_base.h"
#include "jcs_host.h"

#include "gui_plot.h"
#include "gui_host_statistics.h"
#include "gui_host_logger.h"
#include "gui_host_oscilloscope.h"
#include "gui_host_input_stimulus.h"
#include "gui_host_analysis.h"
#include "gui_host_network_firmware.h"
#include "gui_host_2d_hopper.h"

class gui_device_host : public gui_device_base, public gui_device_host_base {
public:
    gui_device_host(jcs::jcs_host* host, gui_interface* gui_if, std::string const& name);
    ~gui_device_host() {}

    int startup();
    int step_rt();
    int render(); 

    int step_rt_always();

private:
    std::vector<gui_device_host_base*> gui_element_host_ptr_;

    gui_host_logger* gui_host_logger_;
    gui_plot* gui_plot_;
    gui_host_statistics* gui_host_statistics_;
    gui_host_oscilloscope* gui_host_oscilloscope_;
    gui_host_input_stimulus* gui_host_input_stimulus_;
    gui_host_analysis* gui_host_analysis_;
    gui_host_network_firmware_update* gui_host_network_firmware_update_;
    gui_host_2d_hopper* gui_host_2d_hopper_;
};
#endif