// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_HOST_STATS_H_
#define GUI_HOST_STATS_H_

#include "jcs_host.h"
#include "gui_type_base.h"
#include "gui_interface.h"
#include "gui_device_host_base.h"
#include "helpers.h"

class gui_host_statistics : public gui_type_base, public gui_device_host_base {
public:
    gui_host_statistics(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device);
    ~gui_host_statistics() {}

    int startup();
    int step_rt();
    int step_rt_always();
    int render();

private:
    // Some nice statistics plots
    helpers::scrolling_buffer to_mean_buffer_;

    helpers::scrolling_buffer cycle_buffer_;
    helpers::scrolling_buffer data_exchange_buffer_;
    float t_s_;
    double t_start_ns_;

    int max_history_s_;
    bool fit_y_;
    bool fit_x_;

    jcs::statistics_timing timing_;
    jcs::statistics_health health_;
};

#endif