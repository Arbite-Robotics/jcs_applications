// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_MC_THERMAL_CALIB_H_
#define GUI_MC_THERMAL_CALIB_H_

#include <string>
#include <vector>
#include "jcs_host.h"
#include "gui_type_base.h"
#include "gui_interface.h"
#include "sampler.h"
#include "helpers.h"
#include "ramp.h"
#include "mc_thermal_fitter.h"
#include "plot_measurement_multi.h"

class gui_mc_thermal_calib : public gui_type_base {
public:
    gui_mc_thermal_calib(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device);
    ~gui_mc_thermal_calib() {}

    int startup();
    int step_rt();
    int step_rt_always();
    int render();    

private:
    // ---------------------------------------------------------------
    // State machine
    // ---------------------------------------------------------------
    enum class state {
        off_s,
        initialise_s,
        ramp_to_current_s,
        hold_step_s,
        ramp_to_next_s,
        cooldown_s,
        finish_s
    };
    state state_;

    int ready_test();
    void get_test_parameters();
    void render_profile_editor();
    void render_prerequisites();
    void render_fit_results();
    void render_fit_plots();

    bool is_ready_;
    bool can_start_;

    // ---------------------------------------------------------------
    // Current profile
    // ---------------------------------------------------------------
    struct profile_step {
        float current_a;
        float duration_s;
    };
    std::vector<profile_step> profile_;
    int current_profile_step_;
    float cooldown_time_s_;

    static std::vector<profile_step> default_profile();

    // ---------------------------------------------------------------
    // Ramp and timing
    // ---------------------------------------------------------------
    float ramp_time_s_;

    ramp i_ramp_;

    int hold_tick_;
    int hold_tick_max_;

    // ---------------------------------------------------------------
    // Sampler — 3 channels: v_d, i_d, t_housing
    // ---------------------------------------------------------------
    sampler sampler_;
    std::vector<std::string> sampler_labels_;

    // ---------------------------------------------------------------
    // Signal storage and routing
    // ---------------------------------------------------------------
    std::vector<float> f32_input_signal_store_;
    std::vector<float> f32_output_signal_store_;

    // Input signal: d-axis current command
    helpers::combo_source signal_in_source_i_d_;

    // Output signal indices (for live display, not strictly required)
    int signal_out_v_d_idx_;
    int signal_out_i_d_idx_;

    std::vector<std::string> required_input_signal_names_;
    std::vector<std::string> required_output_signal_names_;

    // ---------------------------------------------------------------
    // Thermal model prerequisites (from prior tests)
    // ---------------------------------------------------------------
    float rs_ref_;          // Phase resistance at reference temperature [Ohm]
    float rs_t_ref_;        // Temperature at which rs_ref was measured [degC]
    float rs_alpha_;        // Temperature coefficient [1/degC] (default: 0.00393 copper)
    float i_d_min_;         // Minimum i_d for valid R(t) computation [A]

    // ---------------------------------------------------------------
    // Fit configuration and results
    // ---------------------------------------------------------------
    enum class model_order {
        first_order_s,
        second_order_s
    };
    model_order model_order_;

    mc_thermal_fitter::derived_data_config derive_config_;
    mc_thermal_fitter::recorded_data       recorded_data_;
    mc_thermal_fitter::initial_guess       initial_guess_;
    mc_thermal_fitter::fit_result          fit_result_;
    bool has_fit_result_;
    bool has_derived_data_;

    // User-adjustable initial guess (populated from auto-estimate, editable)
    float guess_r1_;
    float guess_c1_;
    float guess_r2_;
    float guess_c2_;

    // ---------------------------------------------------------------
    // Validation plots
    // ---------------------------------------------------------------
    plot_measurement_multi plot_t1_;
    plot_measurement_multi::channel* plot_t1_measured_;
    plot_measurement_multi::channel* plot_t1_predicted_;

    plot_measurement_multi plot_t2_;
    plot_measurement_multi::channel* plot_t2_measured_;
    plot_measurement_multi::channel* plot_t2_predicted_;

    plot_measurement_multi plot_power_;
    plot_measurement_multi::channel* plot_power_ch_;
};

#endif