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
#include <array>
#include "gui_stimulus.h"
#include "imgui.h"
#include "helpers.h"

class gui_host_logger : public gui_type_base, public gui_device_host_base {
public:
    gui_host_logger(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device);
    ~gui_host_logger() {}

    int startup();
    int step_rt();
    // int step_rt_special();
    int step_rt_always();
    int render();

private:
    // Sampler
    enum class sampler_state {
        off_s,
        sampling_s,
    };
    sampler_state sampler_state_;

    int sample_rate_;
    int sample_time_;
    int display_points_;

    int storage_count_;
    int storage_length_;

    struct channel {
        std::string name_;
        std::string source_;
        int source_combo_idx_;
        bool fit_y_;
        bool fit_x_;

        helpers::scrolling_buffer buffer_;
        int sample_time_s_;
        int sample_rate_;
        int downsample_points_;

        bool plot_cursors_;
        double cursor_tag_[4];

        channel(std::string const& name, std::string const& source, int const sample_time_s, int const sample_rate_hz);

        void update_sample_rate(int sample_rate_hz);
        void update_storage_length(int sample_time_s);
        void update_display_downsample_points(int downsample_points);
        void plot();
        void clear();
    };
    std::array<channel*, 8> channels_;

    std::vector<float> f32_osignal_store_;

    double t_start_ns_;
    float t_s_;

    int render_plot();
    int render_interface();
    void render_status();
    int write_channels_to_file();
    int emit_data(std::string const& path_and_file);
};

#endif
