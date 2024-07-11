// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_HOST_ANALYSIS_H_
#define GUI_HOST_ANALYSIS_H_

#include "jcs_host.h"
#include "gui_type_base.h"
#include <vector>
#include <string>
#include "gui_stimulus.h"
#include "imgui.h"
#include "helpers.h"

class gui_host_analysis : public gui_type_base {
public:
    gui_host_analysis(jcs::jcs_host* host, std::string const& target_device);
    ~gui_host_analysis() {}

    int startup();
    int step_rt();
    int render();

private:
    // Sampler
    enum class sampler_state {
        off_s,
        init_s,
        sampling_s,
        done_s
    };
    sampler_state sampler_state_;

    int         sample_rate_;
    int         sample_time_;

    int         storage_count_;
    int         storage_length_;

    struct plotter {
        int size_;
        std::vector<double> x_;
        std::vector<double> y0_;
        std::vector<double> y1_;
        bool plot_cursors_;
        double cursor_tag_[4];

        std::string source_y0_;
        int source_combo_idx_y0_;
        std::string source_y1_;
        int source_combo_idx_y1_;
        plotter(int const sample_time_s, int const sample_rate_hz);
        void update_sample_rate(int sample_rate_hz);
        void update_storage_length(int sample_time_s, int sample_rate_hz);
        void plot();
    };
    plotter plotter_;

    std::vector<float> f32_osignal_store_;
    std::vector<float> f32_isignal_store_;
    std::vector<std::string> f32_output_signal_names_;
    std::vector<std::string> f32_input_signal_names_;

    // Trigger
    enum class control_type {
        output_signal_trigger_s,
        // input_signal_trigger_s,
        input_stimulus_s,
        start_manual_s
    };

    // Input stimulus
    gui_stimulus* input_stimulus_;

    int render_plot();
    int render_interface();
    void render_status();
    void render_analysis();
    int write_channels_to_file();
    int emit_data(std::string const& path_and_file);
};

#endif
