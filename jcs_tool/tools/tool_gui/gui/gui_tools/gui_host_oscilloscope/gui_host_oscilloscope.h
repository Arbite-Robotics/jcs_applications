// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_HOST_OSCILLOSCOPE_H_
#define GUI_HOST_OSCILLOSCOPE_H_

#include "jcs_host.h"
#include "gui_type_base.h"
#include "gui_interface.h"
#include "gui_device_host_base.h"
#include <vector>
#include <string>
#include <array>
#include "gui_stimulus.h"
#include "imgui.h"

class gui_host_oscilloscope : public gui_type_base, public gui_device_host_base {
public:
    gui_host_oscilloscope(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device);
    ~gui_host_oscilloscope() {}

    int startup();
    int step_rt();
    int step_rt_always();
    int render();

private:
    // Sampler
    enum class sampler_state {
        off_s,
        waiting_trigger_s,
        sampling_s,
        done_s
    };
    sampler_state sampler_state_;

    int         sample_rate_;
    int         sample_time_;

    int         storage_count_;
    int         storage_length_;

    struct channel {
        std::string name_;
        std::string source_;
        int source_combo_idx_;
        std::vector<float> data_;
        std::vector<float> x_;

        bool plot_cursors_;
        double cursor_tag_[4];

        channel(std::string const& name, std::string const& source, int const sample_time_s, int const sample_rate_hz);

        void update_sample_rate(int sample_time_s, int sample_rate_hz);
        void update_storage_length(int sample_time_s, int sample_rate_hz);
        void plot();
    };
    std::array<channel*, 8> channels_;

    std::vector<float> f32_osignal_store_;
    std::vector<float> f32_isignal_store_;

    // Trigger
    enum class control_type {
        output_signal_trigger_s,
        // input_signal_trigger_s,
        input_stimulus_s,
        start_manual_s
    };

    struct trigger {
        control_type type;
        int          osig_source_combo_idx;
        int          isig_source_combo_idx;
        float        level;

        trigger();
        void reset();
    };
    trigger trigger_;
    bool step_rt_has_trigger();
    void step_rt_sources();

    // Input stimulus
    gui_stimulus* input_stimulus_;
    int input_stim_combo_idx_;

    int render_plot();
    int render_interface();
    void render_status();
    int write_channels_to_file();
    int emit_data(std::string const& path_and_file);
};

#endif
