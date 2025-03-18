// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_MC_COGGING_H_
#define GUI_MC_COGGING_H_

#include "jcs_host.h"
#include "imgui.h"
#include <array>
#include <string>
#include "helpers.h"
#include "gui_type_base.h"
#include "gui_interface.h"

//////////////////////////////////////////////////////////////////////
class gui_mc_cogging : public gui_type_base {
public:
    gui_mc_cogging(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device);
    ~gui_mc_cogging() {}

    int startup();
    int step_rt();
    int render();

private:
    // Configure parameters
    const static int rotation_steps_ = 1024;
    
    float theta_threshold_ = 0.0005f;
    float omega_threshold_ = 0.05f;
    
    void initialise();

    enum class behaviour {
        standby_s,
        wait_position_s,
        rotate_s,
        finish_s
    };
    behaviour state_;

    std::vector<std::string> required_input_signal_names_;
    std::vector<std::string> required_output_signal_names_;

    bool is_ready_;
    bool can_start_;
    int ready_test();

    // Storage
    std::vector<float> signals_out_;
    std::vector<float> signals_in_;
    // Indices into storage
    int fb_th_m_0_idx_;
    int fb_w_m_0_idx_;
    int fb_i_q_idx_;
    int cmd_th_m_0_idx_;

    float computed_mean_;

    // helpers
    float theta_error_;
    float theta_command_;
    int rotation_tick_;
    void compute_outputs();

    int write_coeffs_to_file();
    int emit_config(std::string const& file_path);

    // Results
    helpers::plot_measurement plot_result_;
    helpers::plot_measurement plot_final_;
    helpers::plot_measurement plot_vis_;
    float compensated_at_zero_pos_;
    float cogging_compensator_upper_w_m_low_;
    float cogging_compensator_upper_w_m_high_;
};

#endif