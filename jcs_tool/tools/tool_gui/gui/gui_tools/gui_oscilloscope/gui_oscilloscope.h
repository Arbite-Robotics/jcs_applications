// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_OSCILLOSCOPE_H_
#define GUI_OSCILLOSCOPE_H_

#include "jcs_host.h"
#include "gui_type_base.h"
#include "gui_interface.h"
#include "tool_gui_settings.h"
#include <vector>
#include <array>
#include <string>
#include "imgui.h"

class gui_oscilloscope : public gui_type_base {
public:
    gui_oscilloscope(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device, 
        std::vector<std::string> const* oscilloscope_sources, std::vector<std::string> const* oscilloscope_trigger_config,
        int const default_sample_rate_hz, int const sample_length, int const n_channels);
    ~gui_oscilloscope() {}

    int startup();
    int step_rt();
    int render();

private:
    std::vector<std::string> const* oscilloscope_sources_;
    std::vector<std::string> const* oscilloscope_trigger_config_;
    int n_channels_;
    int sample_length_;

    std::string trigger_source_;
    int         trigger_source_combo_idx_;
    std::string trigger_config_;
    int         trigger_config_combo_idx_;
    float       trigger_level_;
    int         sample_rate_;
    int         trigger_buffer_position_;
    bool        is_done_sampling_;

    struct channel {
        std::string name_;
        std::string source_;
        int source_combo_idx_;
        std::vector<float> data_;
        std::vector<float> x_;

        bool plot_cursors_;
        double cursor_tag_[4];

        channel(std::string const& name, std::string const& source, int const sample_length, int const initial_sample_rate);

        void update_sample_rate(int sample_rate);
        void plot();
    };
    // channel channels_[4];
    std::array<channel*, 4> channels_;

    int render_plot();
    int render_interface();
    int write_channels_to_file();
    int emit_data(std::string const& path_and_file);
};

#endif