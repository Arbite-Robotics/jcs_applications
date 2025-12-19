// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_MC_ENCODER_CALIB_H_
#define GUI_MC_ENCODER_CALIB_H_

#include <string>
#include "jcs_host.h"
#include "gui_type_base.h"
#include "gui_interface.h"
#include "sampler.h"
#include "helpers.h"
#include "mc_encoder_corrector.h"
#include "ramp.h"
#include "rotate.h"
#include "plot_measurement_multi.h"

class gui_mc_encoder_calib : public gui_type_base {
public:
    gui_mc_encoder_calib(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device);
    ~gui_mc_encoder_calib() {}

    int startup();
    int step_rt();
    int step_rt_always();
    int render();    

private:
    static const int calib_points_ = 64;
    static const int test_time_s_ = 10;

    helpers::combo_source active_encoder_;
    std::vector<std::string> const encoders_;
    std::string configured_encoder_;
    std::string configured_estimator_;

    enum class state {
        off_s,
        initialise_s,
        ramp_to_current_s,
        rotate_s,
        finish_ramp_s,
        finish_s
    };
    state state_;

    int ready_test();
    void get_test_parameters();

    enum class vi_mode {
        current_s,
        voltage_s
    };
    vi_mode vi_mode_;

    bool is_ready_;
    bool can_start_;

    float test_current_;
    float ramp_time_s_;
    float dwell_time_s_;
    float rotate_speed_rads_;

    ramp i_ramp_;
    rotate i_rotate_;
    int rotation_tick_;

    std::vector<float> f32_input_signal_store_;
    std::vector<float> f32_output_signal_store_;

    helpers::combo_source signal_in_source_d_;
    helpers::combo_source signal_in_source_th_m_;

    int signal_out_th_m_0_idx_;
    int signal_out_d_idx_;

    // Plotters and storage
    plot_measurement_multi th_m_orig_;
    plot_measurement_multi::channel* th_m_orig_reference_;
    plot_measurement_multi::channel* th_m_orig_recorded_;

    plot_measurement_multi th_m_error_;
    plot_measurement_multi::channel* th_m_error_corrected_;
    plot_measurement_multi::channel* th_m_error_recorded_;

    plot_measurement_multi th_m_corrected_;
    plot_measurement_multi::channel* th_m_corrected_corrected_;
    plot_measurement_multi::channel* th_m_corrected_recorded_;
    plot_measurement_multi::channel* th_m_corrected_reference_;

    // Calibrator
    mc_encoder_corrector::correction_table correction_table_;

    // Original settings storage
    bool conf_enc_theta_bypass_;
    bool conf_enc_theta_bypass_direction_;
    bool conf_est_theta_passthrough_;
};

#endif
