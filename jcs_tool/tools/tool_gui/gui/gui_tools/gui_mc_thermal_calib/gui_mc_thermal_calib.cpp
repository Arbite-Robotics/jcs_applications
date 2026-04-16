// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_mc_thermal_calib.h"
#include <iostream>
#include <cmath>
#include "jcs_user_external.h"
#include "jcs_dev_motor_controller.h"
#include "imgui_helpers.h"

//////////////////////////////////////////////////////////////////////
gui_mc_thermal_calib::gui_mc_thermal_calib(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) :
    gui_type_base("Thermal Model Calibrator", host, gui_if, target_device),
    sampler_labels_({"v_d", "i_d", "t_housing"}),
    sampler_(host->base_frequency_get(), gui_if->get_f32_output_signal_names(), 3, 10, 300, &sampler_labels_),
    i_ramp_(1.0 / static_cast<double>(host_->base_frequency_get())),
    state_(state::off_s),
    is_ready_(false),
    can_start_(true),
    signal_in_source_i_d_("None", 0),
    signal_out_v_d_idx_(0),
    signal_out_i_d_idx_(0),
    current_profile_step_(0),
    hold_tick_(0),
    hold_tick_max_(0),
    model_order_(model_order::first_order_s),
    has_fit_result_(false),
    has_derived_data_(false),
    plot_t1_("T1 Winding Temperature", "Time (s)", "Temperature (degC)", 300, host->base_frequency_get()),
    plot_t2_("T2 Housing Temperature", "Time (s)", "Temperature (degC)", 300, host->base_frequency_get()),
    plot_power_("Power Dissipated", "Time (s)", "Power (W)", 300, host->base_frequency_get())
{
    // Default test parameters
    rs_ref_   = 1.0f;
    rs_t_ref_ = 25.0f;
    rs_alpha_ = 0.00393f;
    i_d_min_  = 0.1f;

    ramp_time_s_    = 2.0f;
    cooldown_time_s_ = 60.0f;

    // Default profile
    profile_ = default_profile();

    // Default initial guess
    guess_r1_ = 1.0f;
    guess_c1_ = 10.0f;
    guess_r2_ = 5.0f;
    guess_c2_ = 50.0f;

    // Validation plot channels
    plot_t1_.add_channel("Measured",  ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
    plot_t1_.add_channel("Predicted", ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
    plot_t1_measured_  = plot_t1_.get_channel("Measured");
    plot_t1_predicted_ = plot_t1_.get_channel("Predicted");

    plot_t2_.add_channel("Measured",  ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
    plot_t2_.add_channel("Predicted", ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
    plot_t2_measured_  = plot_t2_.get_channel("Measured");
    plot_t2_predicted_ = plot_t2_.get_channel("Predicted");

    plot_power_.add_channel("Power", ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
    plot_power_ch_ = plot_power_.get_channel("Power");
}


int gui_mc_thermal_calib::startup() {
    if (sampler_.startup((double)jcs::external::time_now_ns()) != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }

    // Configure signal storage
    f32_output_signal_store_.resize(host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0));
    f32_input_signal_store_.resize(host_->sig_input_sz_unsafe_rt(jcs::signal_type::float32_s, 0));

    can_start_ = true;

    // required_input_signal_names_ = { "HOST::i_d" };
    required_output_signal_names_ = { target_device_+"::v_d",
                                       target_device_+"::i_d" };

    // Check that required output signals exist
    if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::v_d", &signal_out_v_d_idx_) != jcs::RET_OK) { can_start_ = false; }
    if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::i_d", &signal_out_i_d_idx_) != jcs::RET_OK) { can_start_ = false; }

    return jcs::RET_OK;
}


int gui_mc_thermal_calib::step_rt() {
    host_->sig_output_get_rt(0, &f32_output_signal_store_);

    // Feed sampler on every tick — it handles its own decimation
    sampler_.step_rt((double)jcs::external::time_now_ns(), &f32_output_signal_store_);

    switch (state_) {
        default:
        case state::finish_s:
        case state::off_s:
            break;

        case state::initialise_s:
            break;

        case state::ramp_to_current_s:
            f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = i_ramp_.step();
            host_->sig_input_set_rt(0, f32_input_signal_store_);
            if (i_ramp_.is_done()) {
                // Start holding at current level
                hold_tick_ = 0;
                hold_tick_max_ = static_cast<int>(
                    profile_[current_profile_step_].duration_s * 
                    static_cast<float>(host_->base_frequency_get()));
                state_ = state::hold_step_s;
            }
            break;

        case state::hold_step_s:
            // Hold the current level, sampler is recording
            host_->sig_input_set_rt(0, f32_input_signal_store_);
            hold_tick_++;
            if (hold_tick_ >= hold_tick_max_) {
                // Move to next step or cooldown
                current_profile_step_++;
                if (current_profile_step_ >= static_cast<int>(profile_.size())) {
                    // All steps done — ramp down to zero for cooldown
                    float current_level = profile_.back().current_a;
                    i_ramp_.start(current_level, 0.0f, ramp_time_s_, 0.5f, 0.0f);
                    state_ = state::ramp_to_next_s;  // Ramp to zero, then cooldown
                } else {
                    // Ramp to next step's current level
                    float from = profile_[current_profile_step_ - 1].current_a;
                    float to   = profile_[current_profile_step_].current_a;
                    i_ramp_.start(from, to, ramp_time_s_, 0.5f, 0.0f);
                    state_ = state::ramp_to_current_s;
                }
            }
            break;

        case state::ramp_to_next_s:
            f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = i_ramp_.step();
            host_->sig_input_set_rt(0, f32_input_signal_store_);
            if (i_ramp_.is_done()) {
                if (current_profile_step_ >= static_cast<int>(profile_.size())) {
                    // Entering cooldown phase
                    hold_tick_ = 0;
                    hold_tick_max_ = static_cast<int>(
                        cooldown_time_s_ * static_cast<float>(host_->base_frequency_get()));
                    f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = 0.0f;
                    state_ = state::cooldown_s;
                } else {
                    // Continue with next hold
                    hold_tick_ = 0;
                    hold_tick_max_ = static_cast<int>(
                        profile_[current_profile_step_].duration_s * 
                        static_cast<float>(host_->base_frequency_get()));
                    state_ = state::hold_step_s;
                }
            }
            break;

        case state::cooldown_s:
            // Zero current, just recording the cooldown
            f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = 0.0f;
            host_->sig_input_set_rt(0, f32_input_signal_store_);
            hold_tick_++;
            if (hold_tick_ >= hold_tick_max_) {
                state_ = state::finish_s;
            }
            break;
    }

    return jcs::RET_OK;
}


int gui_mc_thermal_calib::step_rt_always() {
    return jcs::RET_OK;
}


int gui_mc_thermal_calib::render() {
    // ---------------------------------------------------------------
    // State transitions that happen in render (non-RT)
    // ---------------------------------------------------------------
    switch (state_) {
        default:
        case state::off_s:
            break;

        case state::initialise_s:
        {
            // Zero the input
            f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = 0.0f;

            // Adjust sampler time to test time
            int total_test_time_s = static_cast<int>(std::ceil(cooldown_time_s_));
            for (int i = 0; i < static_cast<int>(profile_.size()); ++i) {
                total_test_time_s += static_cast<int>(std::ceil(profile_[i].duration_s + ramp_time_s_));
            }
            // Add some margin
            total_test_time_s += 10;
            if (total_test_time_s > sampler_.get_sample_time_s()) {
                sampler_.set_sample_time_s(total_test_time_s);
            }

            // Start JCS host
            if (gui_if_->start() != jcs::RET_OK) {
                state_ = state::off_s;
                break;
            }
            helpers::sleep_ms(500);

            // Start the controller in current DQ mode (d-axis injection, no rotation)
            PARAM_NOTIFY_ACTION( host_->write_enum(target_device_, "controller_mode", "current_dq"),
                "Parameter failed: controller_mode", state_ = state::off_s; break; )
            PARAM_NOTIFY_ACTION( host_->write_command(target_device_, "controller_start"),
                "Parameter failed: controller_start", state_ = state::off_s; break; )

            // Begin sampling
            sampler_.start();

            // Start ramping to first profile step current
            current_profile_step_ = 0;
            i_ramp_.start(0.0f, profile_[0].current_a, ramp_time_s_, 0.5f, 0.0f);
            state_ = state::ramp_to_current_s;
            break;
        }

        case state::ramp_to_current_s:
        case state::hold_step_s:
        case state::ramp_to_next_s:
        case state::cooldown_s:
            break;

        case state::finish_s:
            f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = 0.0f;
            sampler_.stop();
            gui_if_->stop();
            state_ = state::off_s;
            break;
    }

    // ---------------------------------------------------------------
    // UI Layout
    // ---------------------------------------------------------------
    ImGui::Text("Thermal Model Calibrator");
    ImGui::Separator();
    ImGui::Text("This tool identifies the thermal model parameters (R1, C1, R2, C2) for the motor controller.");
    ImGui::Text("It applies a multi-level d-axis current profile and records v_d, i_d, and housing temperature.");
    ImGui::Text("From this data, the winding temperature is inferred via the resistance-temperature relationship");
    ImGui::Text("and the thermal model parameters are fitted using Levenberg-Marquardt optimisation.");
    ImGui::Text(" ");
    ImGui::Text("Prerequisites:");
    ImGui::Text("- test_dq_r completed (need Rs_ref and the temperature at which it was measured).");
    ImGui::Text("- Motor at thermal equilibrium (thermistor reading stable) before starting.");
    ImGui::Text("- Thermistor on housing required for full 2nd order identification.");
    ImGui::Text("- Motor does NOT need to spin. Shaft can be fixed.");
    ImGui::Text("- Current controllers tuned. Encoder zeroed.");
    ImGui::Text("- Device is not temperature clamped.");
    ImGui::Text(" ");
    ImGui::Text("The temperature input should be a housing/case temperature measurement (e.g. thermistor,");
    ImGui::Text("NTC, or board temperature sensor). Map it to sampler channel 2 (t_housing).");
    ImGui::Text("For 1st order mode, this is used directly as T2. For 2nd order mode, it is fitted.");
    ImGui::Text(" ");

    ImGui::Separator();
    // helpers::input_signals_check(gui_if_->get_f32_input_signal_names(), &required_input_signal_names_);
    helpers::output_signals_check(gui_if_->get_f32_output_signal_names(), &required_output_signal_names_);

    ImGui::Separator();
    if (ImGui::Button("Click to make motor controller ready for tests!")) {
        if (ready_test() != jcs::RET_OK) {
            return jcs::RET_OK;
        }
    }

    if (!can_start_) {
        ImGui::Text("Can't start! Most likely missing signal.");
    }

    {
        ImGuiDisabled ui_disabled(!is_ready_);

        // ---------------------------------------------------------------
        // Model order selection
        // ---------------------------------------------------------------
        ImGui::Separator();
        ImGui::Text("Model order");
        if (ImGui::RadioButton("1st Order (R1, C1) — requires housing temperature measurement",
                               model_order_ == model_order::first_order_s)) {
            model_order_ = model_order::first_order_s;
        }
        if (ImGui::RadioButton("2nd Order (R1, C1, R2, C2) — estimates both winding and housing",
                               model_order_ == model_order::second_order_s)) {
            model_order_ = model_order::second_order_s;
        }

        // ---------------------------------------------------------------
        // Prerequisites: Rs reference, temperature coefficient
        // ---------------------------------------------------------------
        render_prerequisites();

        // ---------------------------------------------------------------
        // Test parameters and profile
        // ---------------------------------------------------------------
        get_test_parameters();

        ImGui::Separator();
        render_profile_editor();

        // ---------------------------------------------------------------
        // Sampler interface (channel → signal mapping)
        // ---------------------------------------------------------------
        ImGui::Separator();
        sampler_.render_interface();

        // ---------------------------------------------------------------
        // Controls
        // ---------------------------------------------------------------
        ImGui::Separator();
        ImGui::Text("Starting this test will start JCS system. Ensure it is safe to do so.");
        if (ImGui::Button("Start test")) {
            if (state_ == state::off_s && !profile_.empty()) {
                has_fit_result_ = false;
                has_derived_data_ = false;
                state_ = state::initialise_s;
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Stop test")) {
            if (state_ != state::off_s) {
                state_ = state::finish_s;
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Force sampler start")) {
            sampler_.start();
        }

        // ---------------------------------------------------------------
        // State display
        // ---------------------------------------------------------------
        ImGui::Separator();
        ImGui::Text("Test State: ");
        ImGui::SameLine();
        switch (state_) {
            default:
            case state::off_s:             ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.0f, 1.0f), "Off");          break;
            case state::initialise_s:      ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Initialising"); break;
            case state::ramp_to_current_s: ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Ramping");      break;
            case state::hold_step_s:
                ImGui::TextColored(ImVec4(0.0f, 0.8f, 0.0f, 1.0f), "Holding step %d/%d (%.1f A)",
                    current_profile_step_ + 1, (int)profile_.size(),
                    profile_[current_profile_step_].current_a);
                break;
            case state::ramp_to_next_s:    ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Ramping to next"); break;
            case state::cooldown_s:        ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.8f, 1.0f), "Cooldown");      break;
            case state::finish_s:          ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Finishing");     break;
        }

        // ---------------------------------------------------------------
        // Live signal display
        // ---------------------------------------------------------------
        ImGui::Separator();
        sampler_.render_status();

        static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg |
                                             ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable |
                                             ImGuiTableFlags_NoSavedSettings;
        if (ImGui::BeginTable("LiveSignals", 3, table_flags)) {
            ImGui::TableSetupColumn("Signal",         ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("Commanded value", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("Measured value",  ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::Text("i_d");
            ImGui::TableSetColumnIndex(1); ImGui::Text("%.6f", f32_input_signal_store_[ signal_in_source_i_d_.index_ ]);
            ImGui::TableSetColumnIndex(2); ImGui::Text("%.6f", f32_output_signal_store_[signal_out_i_d_idx_]);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::Text("v_d");
            ImGui::TableSetColumnIndex(1); ImGui::Text("-");
            ImGui::TableSetColumnIndex(2); ImGui::Text("%.6f", f32_output_signal_store_[signal_out_v_d_idx_]);

            ImGui::EndTable();
        }

        // ---------------------------------------------------------------
        // Sampler plots (live recording)
        // ---------------------------------------------------------------
        ImGui::Separator();
        sampler_.render_plots();

        // ---------------------------------------------------------------
        // Post-processing: derive signals, fit model
        // ---------------------------------------------------------------
        ImGui::Separator();
        ImGui::Text("Fitting");

        if (ImGui::Button("Derive signals from sampler data")) {
            // TODO: Extract time, v_d, i_d, t_housing arrays from sampler channels.
            // The sampler stores data in its scrolling_buffer per channel.
            // You will need to expose a method on the sampler or its channels
            // to retrieve the recorded data as a contiguous vector, e.g.:
            //   sampler_.get_channel_data(0, &time_vec, &data_vec);
            //
            // For now, this shows the structure. Wire up to your sampler's
            // data retrieval once available.
            //
            // Channel mapping (user configures in sampler UI):
            //   Channel 0: v_d
            //   Channel 1: i_d
            //   Channel 2: t_housing

            std::vector<float> time_s, v_d_data, i_d_data, t_housing_data;
            sampler_.get_channel_data(0, &time_s, &v_d_data);       // Channel 0: v_d
            sampler_.get_channel_data(1, nullptr,  &i_d_data);      // Channel 1: i_d
            sampler_.get_channel_data(2, nullptr,  &t_housing_data); // Channel 2: t_housing

            derive_config_.rs_ref   = static_cast<double>(rs_ref_);
            derive_config_.rs_t_ref = static_cast<double>(rs_t_ref_);
            derive_config_.rs_alpha = static_cast<double>(rs_alpha_);
            derive_config_.i_d_min_threshold = static_cast<double>(i_d_min_);
            derive_config_.skip_time_s = static_cast<double>(ramp_time_s_) + 2.0;

            if (mc_thermal_fitter::derive_signals(time_s, v_d_data, i_d_data, t_housing_data,
                                                   derive_config_, &recorded_data_) == 0) {
                has_derived_data_ = true;
                // Auto-estimate initial guess
                bool is_2nd = (model_order_ == model_order::second_order_s);
                auto guess = mc_thermal_fitter::estimate_initial_params(recorded_data_, is_2nd);
                guess_r1_ = static_cast<float>(guess.r1);
                guess_c1_ = static_cast<float>(guess.c1);
                guess_r2_ = static_cast<float>(guess.r2);
                guess_c2_ = static_cast<float>(guess.c2);

                // Reconfigure power plot to match sampler timebase
                plot_power_.update_storage_length(sampler_.get_sample_time_s(), sampler_.get_sample_rate_hz());
                plot_power_ch_ = plot_power_.get_channel("Power");

                // Populate power plot
                size_t n = recorded_data_.time_s.size();
                for (size_t i = 0; i < n && i < plot_power_ch_->y_.size(); ++i) {
                    plot_power_ch_->y_[i] = static_cast<float>(recorded_data_.power_w[i]);
                }

                std::cout << "Derived " << n << " samples. "
                          << "Auto-guess: R1=" << guess_r1_ << " C1=" << guess_c1_
                          << " R2=" << guess_r2_ << " C2=" << guess_c2_ << "\n";
            } else {
                std::cout << "ERROR: Failed to derive signals from sampler data.\n";
            }
            
        }

        if (has_derived_data_) {
            ImGui::Text("Derived data: %d samples", (int)recorded_data_.time_s.size());
            ImGui::Text("T_initial: %.2f degC", recorded_data_.t_initial);
        }

        // ---------------------------------------------------------------
        // Initial guess (auto-estimated, user-editable)
        // ---------------------------------------------------------------
        ImGui::Separator();
        ImGui::Text("Initial guess (auto-estimated, adjust if needed)");
        ImGui::InputFloat("R1 (K/W)", &guess_r1_, 0.1f, 1.0f, "%.4f");
        ImGui::InputFloat("C1 (J/K)", &guess_c1_, 0.1f, 1.0f, "%.4f");
        if (model_order_ == model_order::second_order_s) {
            ImGui::InputFloat("R2 (K/W)", &guess_r2_, 0.1f, 1.0f, "%.4f");
            ImGui::InputFloat("C2 (J/K)", &guess_c2_, 0.1f, 1.0f, "%.4f");
        }

        // ---------------------------------------------------------------
        // Run the fit
        // ---------------------------------------------------------------
        ImGui::Separator();
        {
            ImGuiDisabled fit_disabled(!has_derived_data_);

            if (ImGui::Button("Compute thermal model fit")) {
                mc_thermal_fitter::initial_guess guess;
                guess.r1 = static_cast<double>(guess_r1_);
                guess.c1 = static_cast<double>(guess_c1_);
                guess.r2 = static_cast<double>(guess_r2_);
                guess.c2 = static_cast<double>(guess_c2_);

                if (model_order_ == model_order::first_order_s) {
                    fit_result_ = mc_thermal_fitter::fit_1st_order(recorded_data_, guess);
                } else {
                    fit_result_ = mc_thermal_fitter::fit_2nd_order(recorded_data_, guess);
                }
                has_fit_result_ = true;

                // Reconfigure validation plots to match sampler timebase
                plot_t1_.update_storage_length(sampler_.get_sample_time_s(), sampler_.get_sample_rate_hz());
                plot_t2_.update_storage_length(sampler_.get_sample_time_s(), sampler_.get_sample_rate_hz());
                plot_power_.update_storage_length(sampler_.get_sample_time_s(), sampler_.get_sample_rate_hz());

                // Re-grab channel pointers after resize
                plot_t1_measured_  = plot_t1_.get_channel("Measured");
                plot_t1_predicted_ = plot_t1_.get_channel("Predicted");
                plot_t2_measured_  = plot_t2_.get_channel("Measured");
                plot_t2_predicted_ = plot_t2_.get_channel("Predicted");
                plot_power_ch_     = plot_power_.get_channel("Power");

                // Populate validation plots
                size_t n = fit_result_.t1_predicted.size();
                for (size_t i = 0; i < n && i < plot_t1_measured_->y_.size(); ++i) {
                    plot_t1_measured_->y_[i]  = static_cast<float>(recorded_data_.t1_measured[i]);
                    plot_t1_predicted_->y_[i] = static_cast<float>(fit_result_.t1_predicted[i]);
                }
                for (size_t i = 0; i < n && i < plot_t2_measured_->y_.size(); ++i) {
                    plot_t2_measured_->y_[i]  = static_cast<float>(recorded_data_.t2_measured[i]);
                    plot_t2_predicted_->y_[i] = static_cast<float>(fit_result_.t2_predicted[i]);
                }

                std::cout << "Fit result: " << fit_result_.info << "\n";
            }
        }

        // ---------------------------------------------------------------
        // Fit results display
        // ---------------------------------------------------------------
        if (has_fit_result_) {
            render_fit_results();
        }

        // ---------------------------------------------------------------
        // Write to device
        // ---------------------------------------------------------------
        ImGui::Separator();
        ImGui::Text("Write results to device");
        {
            ImGuiDisabled write_disabled(!has_fit_result_ || !fit_result_.converged);

            if (ImGui::Button("Write thermal model to device")) {
                PARAM_NOTIFY( host_->write_float(target_device_, "thermal_model_r1",
                    static_cast<float>(fit_result_.r1)), "Parameter failed: thermal_model_r1" )
                PARAM_NOTIFY( host_->write_float(target_device_, "thermal_model_c1",
                    static_cast<float>(fit_result_.c1)), "Parameter failed: thermal_model_c1" )

                if (model_order_ == model_order::second_order_s) {
                    PARAM_NOTIFY( host_->write_float(target_device_, "thermal_model_r2",
                        static_cast<float>(fit_result_.r2)), "Parameter failed: thermal_model_r2" )
                    PARAM_NOTIFY( host_->write_float(target_device_, "thermal_model_c2",
                        static_cast<float>(fit_result_.c2)), "Parameter failed: thermal_model_c2" )
                }

                // Also write the Rs reference data used for the fit
                PARAM_NOTIFY( host_->write_float(target_device_, "thermal_model_rs_t_ref",
                    rs_t_ref_), "Parameter failed: thermal_model_rs_t_ref" )
                PARAM_NOTIFY( host_->write_float(target_device_, "thermal_model_rs_alpha",
                    rs_alpha_), "Parameter failed: thermal_model_rs_alpha" )

                std::cout << "Thermal model parameters written to device.\n";
            }
        }

        // ---------------------------------------------------------------
        // Validation plots
        // ---------------------------------------------------------------
        if (has_fit_result_) {
            render_fit_plots();
        }

        // ---------------------------------------------------------------
        // Sampler file export
        // ---------------------------------------------------------------
        ImGui::Separator();
        sampler_.channels_write_to_file();
    }

    return jcs::RET_OK;
}


void gui_mc_thermal_calib::render_prerequisites() {
    ImGui::Separator();
    ImGui::Text("Reference resistance (from test_dq_r)");
    {
        float value = rs_ref_;
        if (ImGui::InputFloat("Rs_ref (Ohm)", &value, 0.01f, 0.1f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            rs_ref_ = value;
        }
    }
    {
        float value = rs_t_ref_;
        if (ImGui::InputFloat("Rs_t_ref (degC)", &value, 1.0f, 5.0f, "%.2f", ImGuiInputTextFlags_EscapeClearsAll)) {
            rs_t_ref_ = value;
        }
    }
    {
        float value = rs_alpha_;
        if (ImGui::InputFloat("Rs_alpha (1/degC)", &value, 0.0001f, 0.001f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            rs_alpha_ = value;
        }
    }
    ImGui::Text("(Default alpha = 0.00393 for copper. Set to 0 to disable temperature compensation.)");
    // Offer to read Rs from device
    if (ImGui::Button("Read Rs from device")) {
        float rs = 0.0f;
        if (host_->read_float(target_device_, "motor_Rs", &rs) == jcs::RET_OK) {
            rs_ref_ = rs;
            std::cout << "Read Rs = " << rs << " Ohm from device.\n";
        } else {
            std::cout << "ERROR: Failed to read Rs from device.\n";
        }
    }
    ImGui::Separator();
    ImGui::Text("i_d interpolation threshold");
    ImGui::Text("(Samples where |i_d| < threshold are interpolated. Avoids R=V/I blowup at very low current.)");
    {
        float value = i_d_min_;
        if (ImGui::InputFloat("Min |i_d| threshold (A)", &value, 0.01f, 0.1f, "%.4f", ImGuiInputTextFlags_EscapeClearsAll)) {
            i_d_min_ = value;
        }
    }
}


void gui_mc_thermal_calib::get_test_parameters() {
    ImGui::Separator();
    ImGui::Text("Test parameters");
    {
        float value = ramp_time_s_;
        if (ImGui::InputFloat("Ramp time between steps (s)", &value, 0.1f, 1.0f, "%.2f", ImGuiInputTextFlags_EscapeClearsAll)) {
            ramp_time_s_ = value;
        }
    }
    {
        float value = cooldown_time_s_;
        if (ImGui::InputFloat("Cooldown time at zero current (s)", &value, 5.0f, 30.0f, "%.1f", ImGuiInputTextFlags_EscapeClearsAll)) {
            cooldown_time_s_ = value;
        }
    }

    // Input signal selection
    helpers::combo_select("D-Axis current command (i_d)", gui_if_->get_f32_input_signal_names(), &signal_in_source_i_d_);
}


void gui_mc_thermal_calib::render_profile_editor() {
    ImGui::Text("Current profile");
    ImGui::Text("Each step applies a constant d-axis current for the specified duration.");
    ImGui::Text("A cooldown phase at zero current follows automatically after all steps.");

    static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg |
                                         ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable |
                                         ImGuiTableFlags_NoSavedSettings;
    if (ImGui::BeginTable("ProfileSteps", 3, table_flags)) {
        ImGui::TableSetupColumn("Step",       ImGuiTableColumnFlags_WidthFixed, 50.0f);
        ImGui::TableSetupColumn("Current (A)", ImGuiTableColumnFlags_WidthFixed, 150.0f);
        ImGui::TableSetupColumn("Duration (s)", ImGuiTableColumnFlags_WidthFixed, 150.0f);
        ImGui::TableHeadersRow();

        int remove_idx = -1;
        for (int i = 0; i < static_cast<int>(profile_.size()); ++i) {
            ImGui::TableNextRow();
            ImGui::PushID(i);

            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%d", i + 1);

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(120.0f);
            ImGui::InputFloat("##current", &profile_[i].current_a, 0.1f, 1.0f, "%.2f");

            ImGui::TableSetColumnIndex(2);
            ImGui::SetNextItemWidth(120.0f);
            ImGui::InputFloat("##duration", &profile_[i].duration_s, 1.0f, 10.0f, "%.1f");

            ImGui::PopID();
        }
        ImGui::EndTable();
    }

    if (ImGui::Button("Add step")) {
        float last_current = profile_.empty() ? 1.0f : profile_.back().current_a;
        profile_.push_back({ last_current, 30.0f });
    }
    ImGui::SameLine();
    if (ImGui::Button("Remove last step") && !profile_.empty()) {
        profile_.pop_back();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset to default")) {
        profile_ = default_profile();
    }

    // Compute and display total test time
    float total_s = cooldown_time_s_;
    for (auto& step : profile_) {
        total_s += step.duration_s + ramp_time_s_;
    }
    ImGui::Text("Total estimated test time: %.0f s (%.1f min)", total_s, total_s / 60.0f);
}


void gui_mc_thermal_calib::render_fit_results() {
    ImGui::Separator();
    ImGui::Text("Fit results");
    ImGui::Text("Status: %s", fit_result_.info.c_str());

    static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg |
                                         ImGuiTableFlags_Borders | ImGuiTableFlags_NoSavedSettings;
    if (ImGui::BeginTable("FitResults", 3, table_flags)) {
        ImGui::TableSetupColumn("Parameter",  ImGuiTableColumnFlags_WidthFixed, 120.0f);
        ImGui::TableSetupColumn("Value",      ImGuiTableColumnFlags_WidthFixed, 150.0f);
        ImGui::TableSetupColumn("Unit",       ImGuiTableColumnFlags_WidthFixed, 80.0f);
        ImGui::TableHeadersRow();

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("R1");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%.6f", fit_result_.r1);
        ImGui::TableSetColumnIndex(2); ImGui::Text("K/W");

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("C1");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%.4f", fit_result_.c1);
        ImGui::TableSetColumnIndex(2); ImGui::Text("J/K");

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("tau1 (R1*C1)");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%.2f", fit_result_.r1 * fit_result_.c1);
        ImGui::TableSetColumnIndex(2); ImGui::Text("s");

        if (model_order_ == model_order::second_order_s) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::Text("R2");
            ImGui::TableSetColumnIndex(1); ImGui::Text("%.6f", fit_result_.r2);
            ImGui::TableSetColumnIndex(2); ImGui::Text("K/W");

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::Text("C2");
            ImGui::TableSetColumnIndex(1); ImGui::Text("%.4f", fit_result_.c2);
            ImGui::TableSetColumnIndex(2); ImGui::Text("J/K");

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::Text("tau2 (R2*C2)");
            ImGui::TableSetColumnIndex(1); ImGui::Text("%.2f", fit_result_.r2 * fit_result_.c2);
            ImGui::TableSetColumnIndex(2); ImGui::Text("s");
        }

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("RMS error T1");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%.4f", fit_result_.rms_error_t1);
        ImGui::TableSetColumnIndex(2); ImGui::Text("K");

        if (model_order_ == model_order::second_order_s) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::Text("RMS error T2");
            ImGui::TableSetColumnIndex(1); ImGui::Text("%.4f", fit_result_.rms_error_t2);
            ImGui::TableSetColumnIndex(2); ImGui::Text("K");
        }

        ImGui::EndTable();
    }

    // Copyable result text
    ImGui::Separator();
    std::string result_str = "";

    result_str += "######################################################################## \n";
    result_str += "# Thermal model parameters\n";
    if (model_order_ == model_order::first_order_s) {
        result_str += "thermal_model_mode: thermal_model_mode_1st_order\n";
        result_str += "thermal_model_r1:   " + std::to_string(fit_result_.r1) + "\n";
        result_str += "thermal_model_c1:   " + std::to_string(fit_result_.c1) + "\n";
    } else {
        result_str += "thermal_model_mode: thermal_model_mode_2nd_order\n";
        result_str += "thermal_model_r1:   " + std::to_string(fit_result_.r1) + "\n";
        result_str += "thermal_model_c1:   " + std::to_string(fit_result_.c1) + "\n";
        result_str += "thermal_model_r2:   " + std::to_string(fit_result_.r2) + "\n";
        result_str += "thermal_model_c2:   " + std::to_string(fit_result_.c2) + "\n";
    }

    result_str += "# Thermal model boundry source\n";
    result_str += "thermal_model_boundary_source: PLEASE CONFIGURE\n";
    result_str += "thermal_model_t_ambient:       PLEASE CONFIGURE if thermal_model_boundary_source = thermal_model_boundary_ambient_const\n";

    result_str += "# Thermal model reference resistance\n";
    result_str += "thermal_model_rs_t_ref: " + std::to_string(rs_t_ref_) + "\n";
    result_str += "thermal_model_rs_alpha: " + std::to_string(rs_alpha_) + "\n";

    helpers::result_text_copyable(result_str);
}


void gui_mc_thermal_calib::render_fit_plots() {
    ImGui::Separator();
    ImGui::Text("Validation plots");
    ImGui::Text("Green = measured, Red = predicted from fitted model");

    plot_t1_.plot();
    plot_t2_.plot();
    plot_power_.plot();
}


int gui_mc_thermal_calib::ready_test() {
    {
        bool ctrl_is_temperature_clamped = false;
        PARAM_NOTIFY_ERROR( host_->read_bool(target_device_, "temperature_penalty_ctrl_is_clamped",
            &ctrl_is_temperature_clamped), "Parameter failed: temperature_penalty_ctrl_is_clamped" )

        if (ctrl_is_temperature_clamped == true) {
            std::cout << "ERROR: Device control is temperature clamped. Cannot continue with test.\n";
            return jcs::RET_ERROR;
        }
    }
    if (can_start_) {
        is_ready_ = true;
    }
    helpers::sleep_ms(500);
    return jcs::RET_OK;
}

// ===================================================================
// Default current profile
// A multi-level profile to excite both fast (C1) and slow (C2) dynamics.
// The user can edit this in the UI.
// ===================================================================
std::vector<gui_mc_thermal_calib::profile_step> gui_mc_thermal_calib::default_profile() {
    return {
        { 3.0f,  30.0f },  // I_high for 30s — fast winding heat up
        { 1.0f,  30.0f },  // I_low  for 30s — partial cooldown, different power level
        { 2.0f,  60.0f },  // I_mid  for 60s — intermediate, let housing start moving
        { 3.0f,  30.0f },  // I_high for 30s — another burst
    };
    // Cooldown at zero current follows automatically (configurable duration)
}


// // Copyright (c) 2024 Arbite Robotics Pty Ltd
// // https://arbite.io
// //
// #include "gui_mc_thermal_calib.h"
// #include <iostream>
// #include <cmath>
// #include "jcs_user_external.h"
// #include "jcs_dev_motor_controller.h"
// #include "imgui_helpers.h"

// //////////////////////////////////////////////////////////////////////
// gui_mc_thermal_calib::gui_mc_thermal_calib(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) :
//     gui_type_base("Thermal Model Calibrator", host, gui_if, target_device),
//     sampler_labels_({"v_d", "i_d", "t_housing"}),
//     sampler_(host->base_frequency_get(), gui_if->get_f32_output_signal_names(), 3, 10, 300, &sampler_labels_),
//     i_ramp_(1.0 / static_cast<double>(host_->base_frequency_get())),
//     state_(state::off_s),
//     is_ready_(false),
//     can_start_(true),
//     signal_in_source_i_d_("None", 0),
//     signal_out_v_d_idx_(0),
//     signal_out_i_d_idx_(0),
//     current_profile_step_(0),
//     hold_tick_(0),
//     hold_tick_max_(0),
//     model_order_(model_order::first_order_s),
//     has_fit_result_(false),
//     has_derived_data_(false),
//     plot_t1_("T1 Winding Temperature", "Time (s)", "Temperature (degC)", 300, host->base_frequency_get()),
//     plot_t2_("T2 Housing Temperature", "Time (s)", "Temperature (degC)", 300, host->base_frequency_get()),
//     plot_power_("Power Dissipated", "Time (s)", "Power (W)", 300, host->base_frequency_get())
// {
//     // Default test parameters
//     rs_ref_   = 1.0f;
//     rs_t_ref_ = 25.0f;
//     rs_alpha_ = 0.00393f;
//     i_d_min_  = 0.1f;

//     ramp_time_s_    = 2.0f;
//     cooldown_time_s_ = 60.0f;

//     // Default profile
//     profile_ = default_profile();

//     // Default initial guess
//     guess_r1_ = 1.0f;
//     guess_c1_ = 10.0f;
//     guess_r2_ = 5.0f;
//     guess_c2_ = 50.0f;

//     // Validation plot channels
//     plot_t1_.add_channel("Measured",  ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
//     plot_t1_.add_channel("Predicted", ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
//     plot_t1_measured_  = plot_t1_.get_channel("Measured");
//     plot_t1_predicted_ = plot_t1_.get_channel("Predicted");

//     plot_t2_.add_channel("Measured",  ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
//     plot_t2_.add_channel("Predicted", ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
//     plot_t2_measured_  = plot_t2_.get_channel("Measured");
//     plot_t2_predicted_ = plot_t2_.get_channel("Predicted");

//     plot_power_.add_channel("Power", ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
//     plot_power_ch_ = plot_power_.get_channel("Power");
// }


// int gui_mc_thermal_calib::startup() {
//     if (sampler_.startup((double)jcs::external::time_now_ns()) != jcs::RET_OK) {
//         return jcs::RET_ERROR;
//     }

//     // Configure signal storage
//     f32_output_signal_store_.resize(host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0));
//     f32_input_signal_store_.resize(host_->sig_input_sz_unsafe_rt(jcs::signal_type::float32_s, 0));

//     can_start_ = true;

//     // required_input_signal_names_ = { "HOST::i_d" };
//     required_output_signal_names_ = { target_device_+"::v_d",
//                                        target_device_+"::i_d" };

//     // Check that required output signals exist
//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::v_d", &signal_out_v_d_idx_) != jcs::RET_OK) { can_start_ = false; }
//     if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::i_d", &signal_out_i_d_idx_) != jcs::RET_OK) { can_start_ = false; }

//     return jcs::RET_OK;
// }


// int gui_mc_thermal_calib::step_rt() {
//     host_->sig_output_get_rt(0, &f32_output_signal_store_);

//     // Feed sampler on every tick — it handles its own decimation
//     sampler_.step_rt((double)jcs::external::time_now_ns(), &f32_output_signal_store_);

//     switch (state_) {
//         default:
//         case state::finish_s:
//         case state::off_s:
//             break;

//         case state::initialise_s:
//             break;

//         case state::ramp_to_current_s:
//             f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = i_ramp_.step();
//             host_->sig_input_set_rt(0, f32_input_signal_store_);
//             if (i_ramp_.is_done()) {
//                 // Start holding at current level
//                 hold_tick_ = 0;
//                 hold_tick_max_ = static_cast<int>(
//                     profile_[current_profile_step_].duration_s * 
//                     static_cast<float>(host_->base_frequency_get()));
//                 state_ = state::hold_step_s;
//             }
//             break;

//         case state::hold_step_s:
//             // Hold the current level, sampler is recording
//             host_->sig_input_set_rt(0, f32_input_signal_store_);
//             hold_tick_++;
//             if (hold_tick_ >= hold_tick_max_) {
//                 // Move to next step or cooldown
//                 current_profile_step_++;
//                 if (current_profile_step_ >= static_cast<int>(profile_.size())) {
//                     // All steps done — ramp down to zero for cooldown
//                     float current_level = profile_.back().current_a;
//                     i_ramp_.start(current_level, 0.0f, ramp_time_s_, 0.5f, 0.0f);
//                     state_ = state::ramp_to_next_s;  // Ramp to zero, then cooldown
//                 } else {
//                     // Ramp to next step's current level
//                     float from = profile_[current_profile_step_ - 1].current_a;
//                     float to   = profile_[current_profile_step_].current_a;
//                     i_ramp_.start(from, to, ramp_time_s_, 0.5f, 0.0f);
//                     state_ = state::ramp_to_current_s;
//                 }
//             }
//             break;

//         case state::ramp_to_next_s:
//             f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = i_ramp_.step();
//             host_->sig_input_set_rt(0, f32_input_signal_store_);
//             if (i_ramp_.is_done()) {
//                 if (current_profile_step_ >= static_cast<int>(profile_.size())) {
//                     // Entering cooldown phase
//                     hold_tick_ = 0;
//                     hold_tick_max_ = static_cast<int>(
//                         cooldown_time_s_ * static_cast<float>(host_->base_frequency_get()));
//                     f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = 0.0f;
//                     state_ = state::cooldown_s;
//                 } else {
//                     // Continue with next hold
//                     hold_tick_ = 0;
//                     hold_tick_max_ = static_cast<int>(
//                         profile_[current_profile_step_].duration_s * 
//                         static_cast<float>(host_->base_frequency_get()));
//                     state_ = state::hold_step_s;
//                 }
//             }
//             break;

//         case state::cooldown_s:
//             // Zero current, just recording the cooldown
//             f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = 0.0f;
//             host_->sig_input_set_rt(0, f32_input_signal_store_);
//             hold_tick_++;
//             if (hold_tick_ >= hold_tick_max_) {
//                 state_ = state::finish_s;
//             }
//             break;
//     }

//     return jcs::RET_OK;
// }


// int gui_mc_thermal_calib::step_rt_always() {
//     return jcs::RET_OK;
// }


// int gui_mc_thermal_calib::render() {
//     // ---------------------------------------------------------------
//     // State transitions that happen in render (non-RT)
//     // ---------------------------------------------------------------
//     switch (state_) {
//         default:
//         case state::off_s:
//             break;

//         case state::initialise_s:
//         {
//             // Zero the input
//             f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = 0.0f;

//             // Adjust sampler time to test time
//             int total_test_time_s = static_cast<int>(std::ceil(cooldown_time_s_));
//             for (int i = 0; i < static_cast<int>(profile_.size()); ++i) {
//                 total_test_time_s += static_cast<int>(std::ceil(profile_[i].duration_s + ramp_time_s_));
//             }
//             // Add some margin
//             total_test_time_s += 10;
//             if (total_test_time_s > sampler_.get_sample_time_s()) {
//                 sampler_.set_sample_time_s(total_test_time_s);
//             }

//             // Start JCS host
//             if (gui_if_->start() != jcs::RET_OK) {
//                 state_ = state::off_s;
//                 break;
//             }
//             helpers::sleep_ms(500);

//             // Start the controller in current DQ mode (d-axis injection, no rotation)
//             PARAM_NOTIFY_ACTION( host_->write_enum(target_device_, "controller_mode", "current_dq"),
//                 "Parameter failed: controller_mode", state_ = state::off_s; break; )
//             PARAM_NOTIFY_ACTION( host_->write_command(target_device_, "controller_start"),
//                 "Parameter failed: controller_start", state_ = state::off_s; break; )

//             // Begin sampling
//             sampler_.start();

//             // Start ramping to first profile step current
//             current_profile_step_ = 0;
//             i_ramp_.start(0.0f, profile_[0].current_a, ramp_time_s_, 0.5f, 0.0f);
//             state_ = state::ramp_to_current_s;
//             break;
//         }

//         case state::ramp_to_current_s:
//         case state::hold_step_s:
//         case state::ramp_to_next_s:
//         case state::cooldown_s:
//             break;

//         case state::finish_s:
//             f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = 0.0f;
//             sampler_.stop();
//             gui_if_->stop();
//             state_ = state::off_s;
//             break;
//     }

//     // ---------------------------------------------------------------
//     // UI Layout
//     // ---------------------------------------------------------------
//     ImGui::Text("Thermal Model Calibrator");
//     ImGui::Separator();
//     ImGui::Text("This tool identifies the thermal model parameters (R1, C1, R2, C2) for the motor controller.");
//     ImGui::Text("It applies a multi-level d-axis current profile and records v_d, i_d, and housing temperature.");
//     ImGui::Text("From this data, the winding temperature is inferred via the resistance-temperature relationship");
//     ImGui::Text("and the thermal model parameters are fitted using Levenberg-Marquardt optimisation.");
//     ImGui::Text(" ");
//     ImGui::Text("Prerequisites:");
//     ImGui::Text("- test_dq_r completed (need Rs_ref and the temperature at which it was measured).");
//     ImGui::Text("- Motor at thermal equilibrium (thermistor reading stable) before starting.");
//     ImGui::Text("- Thermistor on housing required for full 2nd order identification.");
//     ImGui::Text("- Motor does NOT need to spin. Shaft can be fixed.");
//     ImGui::Text("- Current controllers tuned. Encoder zeroed.");
//     ImGui::Text("- Device is not temperature clamped.");
//     ImGui::Text(" ");
//     ImGui::Text("The temperature input should be a housing/case temperature measurement (e.g. thermistor,");
//     ImGui::Text("NTC, or board temperature sensor). Map it to sampler channel 2 (t_housing).");
//     ImGui::Text("For 1st order mode, this is used directly as T2. For 2nd order mode, it is fitted.");
//     ImGui::Text(" ");

//     ImGui::Separator();
//     // helpers::input_signals_check(gui_if_->get_f32_input_signal_names(), &required_input_signal_names_);
//     helpers::output_signals_check(gui_if_->get_f32_output_signal_names(), &required_output_signal_names_);

//     ImGui::Separator();
//     if (ImGui::Button("Click to make motor controller ready for tests!")) {
//         if (ready_test() != jcs::RET_OK) {
//             return jcs::RET_OK;
//         }
//     }

//     if (!can_start_) {
//         ImGui::Text("Can't start! Most likely missing signal.");
//     }

//     {
//         ImGuiDisabled ui_disabled(!is_ready_);

//         // ---------------------------------------------------------------
//         // Model order selection
//         // ---------------------------------------------------------------
//         ImGui::Separator();
//         ImGui::Text("Model order");
//         if (ImGui::RadioButton("1st Order (R1, C1) — requires housing temperature measurement",
//                                model_order_ == model_order::first_order_s)) {
//             model_order_ = model_order::first_order_s;
//         }
//         if (ImGui::RadioButton("2nd Order (R1, C1, R2, C2) — estimates both winding and housing",
//                                model_order_ == model_order::second_order_s)) {
//             model_order_ = model_order::second_order_s;
//         }

//         // ---------------------------------------------------------------
//         // Prerequisites: Rs reference, temperature coefficient
//         // ---------------------------------------------------------------
//         render_prerequisites();

//         // ---------------------------------------------------------------
//         // Test parameters and profile
//         // ---------------------------------------------------------------
//         get_test_parameters();

//         ImGui::Separator();
//         render_profile_editor();

//         // ---------------------------------------------------------------
//         // Sampler interface (channel → signal mapping)
//         // ---------------------------------------------------------------
//         ImGui::Separator();
//         sampler_.render_interface();

//         // ---------------------------------------------------------------
//         // Controls
//         // ---------------------------------------------------------------
//         ImGui::Separator();
//         ImGui::Text("Starting this test will start JCS system. Ensure it is safe to do so.");
//         if (ImGui::Button("Start test")) {
//             if (state_ == state::off_s && !profile_.empty()) {
//                 has_fit_result_ = false;
//                 has_derived_data_ = false;
//                 state_ = state::initialise_s;
//             }
//         }
//         ImGui::SameLine();
//         if (ImGui::Button("Stop test")) {
//             if (state_ != state::off_s) {
//                 state_ = state::finish_s;
//             }
//         }
//         ImGui::SameLine();
//         if (ImGui::Button("Force sampler start")) {
//             sampler_.start();
//         }

//         // ---------------------------------------------------------------
//         // State display
//         // ---------------------------------------------------------------
//         ImGui::Separator();
//         ImGui::Text("Test State: ");
//         ImGui::SameLine();
//         switch (state_) {
//             default:
//             case state::off_s:             ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.0f, 1.0f), "Off");          break;
//             case state::initialise_s:      ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Initialising"); break;
//             case state::ramp_to_current_s: ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Ramping");      break;
//             case state::hold_step_s:
//                 ImGui::TextColored(ImVec4(0.0f, 0.8f, 0.0f, 1.0f), "Holding step %d/%d (%.1f A)",
//                     current_profile_step_ + 1, (int)profile_.size(),
//                     profile_[current_profile_step_].current_a);
//                 break;
//             case state::ramp_to_next_s:    ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Ramping to next"); break;
//             case state::cooldown_s:        ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.8f, 1.0f), "Cooldown");      break;
//             case state::finish_s:          ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Finishing");     break;
//         }

//         // ---------------------------------------------------------------
//         // Live signal display
//         // ---------------------------------------------------------------
//         ImGui::Separator();
//         sampler_.render_status();

//         static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg |
//                                              ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable |
//                                              ImGuiTableFlags_NoSavedSettings;
//         if (ImGui::BeginTable("LiveSignals", 3, table_flags)) {
//             ImGui::TableSetupColumn("Signal",         ImGuiTableColumnFlags_WidthFixed);
//             ImGui::TableSetupColumn("Commanded value", ImGuiTableColumnFlags_WidthFixed);
//             ImGui::TableSetupColumn("Measured value",  ImGuiTableColumnFlags_WidthStretch);
//             ImGui::TableHeadersRow();

//             ImGui::TableNextRow();
//             ImGui::TableSetColumnIndex(0); ImGui::Text("i_d");
//             ImGui::TableSetColumnIndex(1); ImGui::Text("%.6f", f32_input_signal_store_[ signal_in_source_i_d_.index_ ]);
//             ImGui::TableSetColumnIndex(2); ImGui::Text("%.6f", f32_output_signal_store_[signal_out_i_d_idx_]);

//             ImGui::TableNextRow();
//             ImGui::TableSetColumnIndex(0); ImGui::Text("v_d");
//             ImGui::TableSetColumnIndex(1); ImGui::Text("-");
//             ImGui::TableSetColumnIndex(2); ImGui::Text("%.6f", f32_output_signal_store_[signal_out_v_d_idx_]);

//             ImGui::EndTable();
//         }

//         // ---------------------------------------------------------------
//         // Sampler plots (live recording)
//         // ---------------------------------------------------------------
//         ImGui::Separator();
//         sampler_.render_plots();

//         // ---------------------------------------------------------------
//         // Post-processing: derive signals, fit model
//         // ---------------------------------------------------------------
//         ImGui::Separator();
//         ImGui::Text("Fitting");

//         if (ImGui::Button("Derive signals from sampler data")) {
//             // TODO: Extract time, v_d, i_d, t_housing arrays from sampler channels.
//             // The sampler stores data in its scrolling_buffer per channel.
//             // You will need to expose a method on the sampler or its channels
//             // to retrieve the recorded data as a contiguous vector, e.g.:
//             //   sampler_.get_channel_data(0, &time_vec, &data_vec);
//             //
//             // For now, this shows the structure. Wire up to your sampler's
//             // data retrieval once available.
//             //
//             // Channel mapping (user configures in sampler UI):
//             //   Channel 0: v_d
//             //   Channel 1: i_d
//             //   Channel 2: t_housing

//             std::vector<float> time_s, v_d_data, i_d_data, t_housing_data;
//             sampler_.get_channel_data(0, &time_s, &v_d_data);       // Channel 0: v_d
//             sampler_.get_channel_data(1, nullptr,  &i_d_data);      // Channel 1: i_d
//             sampler_.get_channel_data(2, nullptr,  &t_housing_data); // Channel 2: t_housing

//             derive_config_.rs_ref   = static_cast<double>(rs_ref_);
//             derive_config_.rs_t_ref = static_cast<double>(rs_t_ref_);
//             derive_config_.rs_alpha = static_cast<double>(rs_alpha_);
//             derive_config_.i_d_min_threshold = static_cast<double>(i_d_min_);
//             derive_config_.skip_time_s = static_cast<double>(ramp_time_s_) + 2.0;

//             if (mc_thermal_fitter::derive_signals(time_s, v_d_data, i_d_data, t_housing_data,
//                                                    derive_config_, &recorded_data_) == 0) {
//                 has_derived_data_ = true;
//                 // Auto-estimate initial guess
//                 bool is_2nd = (model_order_ == model_order::second_order_s);
//                 auto guess = mc_thermal_fitter::estimate_initial_params(recorded_data_, is_2nd);
//                 guess_r1_ = static_cast<float>(guess.r1);
//                 guess_c1_ = static_cast<float>(guess.c1);
//                 guess_r2_ = static_cast<float>(guess.r2);
//                 guess_c2_ = static_cast<float>(guess.c2);

//                 // Populate power plot
//                 size_t n = recorded_data_.time_s.size();
//                 if (n <= plot_power_ch_->y_.size()) {
//                     for (size_t i = 0; i < n; ++i) {
//                         plot_power_ch_->y_[i] = static_cast<float>(recorded_data_.power_w[i]);
//                     }
//                 }

//                 std::cout << "Derived " << n << " samples. "
//                           << "Auto-guess: R1=" << guess_r1_ << " C1=" << guess_c1_
//                           << " R2=" << guess_r2_ << " C2=" << guess_c2_ << "\n";
//             } else {
//                 std::cout << "ERROR: Failed to derive signals from sampler data.\n";
//             }
            
//         }

//         if (has_derived_data_) {
//             ImGui::Text("Derived data: %d samples", (int)recorded_data_.time_s.size());
//             ImGui::Text("T_initial: %.2f degC", recorded_data_.t_initial);
//         }

//         // ---------------------------------------------------------------
//         // Initial guess (auto-estimated, user-editable)
//         // ---------------------------------------------------------------
//         ImGui::Separator();
//         ImGui::Text("Initial guess (auto-estimated, adjust if needed)");
//         ImGui::InputFloat("R1 (K/W)", &guess_r1_, 0.1f, 1.0f, "%.4f");
//         ImGui::InputFloat("C1 (J/K)", &guess_c1_, 0.1f, 1.0f, "%.4f");
//         if (model_order_ == model_order::second_order_s) {
//             ImGui::InputFloat("R2 (K/W)", &guess_r2_, 0.1f, 1.0f, "%.4f");
//             ImGui::InputFloat("C2 (J/K)", &guess_c2_, 0.1f, 1.0f, "%.4f");
//         }

//         // ---------------------------------------------------------------
//         // Run the fit
//         // ---------------------------------------------------------------
//         ImGui::Separator();
//         {
//             ImGuiDisabled fit_disabled(!has_derived_data_);

//             if (ImGui::Button("Compute thermal model fit")) {
//                 mc_thermal_fitter::initial_guess guess;
//                 guess.r1 = static_cast<double>(guess_r1_);
//                 guess.c1 = static_cast<double>(guess_c1_);
//                 guess.r2 = static_cast<double>(guess_r2_);
//                 guess.c2 = static_cast<double>(guess_c2_);

//                 if (model_order_ == model_order::first_order_s) {
//                     fit_result_ = mc_thermal_fitter::fit_1st_order(recorded_data_, guess);
//                 } else {
//                     fit_result_ = mc_thermal_fitter::fit_2nd_order(recorded_data_, guess);
//                 }
//                 has_fit_result_ = true;

//                 // Populate validation plots
//                 size_t n = fit_result_.t1_predicted.size();
//                 for (size_t i = 0; i < n && i < plot_t1_measured_->y_.size(); ++i) {
//                     plot_t1_measured_->y_[i]  = static_cast<float>(recorded_data_.t1_measured[i]);
//                     plot_t1_predicted_->y_[i] = static_cast<float>(fit_result_.t1_predicted[i]);
//                 }
//                 for (size_t i = 0; i < n && i < plot_t2_measured_->y_.size(); ++i) {
//                     plot_t2_measured_->y_[i]  = static_cast<float>(recorded_data_.t2_measured[i]);
//                     plot_t2_predicted_->y_[i] = static_cast<float>(fit_result_.t2_predicted[i]);
//                 }

//                 std::cout << "Fit result: " << fit_result_.info << "\n";
//             }
//         }

//         // ---------------------------------------------------------------
//         // Fit results display
//         // ---------------------------------------------------------------
//         if (has_fit_result_) {
//             render_fit_results();
//         }

//         // ---------------------------------------------------------------
//         // Write to device
//         // ---------------------------------------------------------------
//         ImGui::Separator();
//         ImGui::Text("Write results to device");
//         {
//             ImGuiDisabled write_disabled(!has_fit_result_ || !fit_result_.converged);

//             if (ImGui::Button("Write thermal model to device")) {
//                 PARAM_NOTIFY( host_->write_float(target_device_, "thermal_model_r1",
//                     static_cast<float>(fit_result_.r1)), "Parameter failed: thermal_model_r1" )
//                 PARAM_NOTIFY( host_->write_float(target_device_, "thermal_model_c1",
//                     static_cast<float>(fit_result_.c1)), "Parameter failed: thermal_model_c1" )

//                 if (model_order_ == model_order::second_order_s) {
//                     PARAM_NOTIFY( host_->write_float(target_device_, "thermal_model_r2",
//                         static_cast<float>(fit_result_.r2)), "Parameter failed: thermal_model_r2" )
//                     PARAM_NOTIFY( host_->write_float(target_device_, "thermal_model_c2",
//                         static_cast<float>(fit_result_.c2)), "Parameter failed: thermal_model_c2" )
//                 }

//                 // Also write the Rs reference data used for the fit
//                 PARAM_NOTIFY( host_->write_float(target_device_, "thermal_model_rs_t_ref",
//                     rs_t_ref_), "Parameter failed: thermal_model_rs_t_ref" )
//                 PARAM_NOTIFY( host_->write_float(target_device_, "thermal_model_rs_alpha",
//                     rs_alpha_), "Parameter failed: thermal_model_rs_alpha" )

//                 std::cout << "Thermal model parameters written to device.\n";
//             }
//         }

//         // ---------------------------------------------------------------
//         // Validation plots
//         // ---------------------------------------------------------------
//         if (has_fit_result_) {
//             render_fit_plots();
//         }

//         // ---------------------------------------------------------------
//         // Sampler file export
//         // ---------------------------------------------------------------
//         ImGui::Separator();
//         sampler_.channels_write_to_file();
//     }

//     return jcs::RET_OK;
// }


// void gui_mc_thermal_calib::render_prerequisites() {
//     ImGui::Separator();
//     ImGui::Text("Reference resistance (from test_dq_r)");
//     {
//         float value = rs_ref_;
//         if (ImGui::InputFloat("Rs_ref (Ohm)", &value, 0.01f, 0.1f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
//             rs_ref_ = value;
//         }
//     }
//     {
//         float value = rs_t_ref_;
//         if (ImGui::InputFloat("Rs_t_ref (degC)", &value, 1.0f, 5.0f, "%.2f", ImGuiInputTextFlags_EscapeClearsAll)) {
//             rs_t_ref_ = value;
//         }
//     }
//     {
//         float value = rs_alpha_;
//         if (ImGui::InputFloat("Rs_alpha (1/degC)", &value, 0.0001f, 0.001f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
//             rs_alpha_ = value;
//         }
//     }
//     ImGui::Text("(Default alpha = 0.00393 for copper. Set to 0 to disable temperature compensation.)");
//     // Offer to read Rs from device
//     if (ImGui::Button("Read Rs from device")) {
//         float rs = 0.0f;
//         if (host_->read_float(target_device_, "motor_Rs", &rs) == jcs::RET_OK) {
//             rs_ref_ = rs;
//             std::cout << "Read Rs = " << rs << " Ohm from device.\n";
//         } else {
//             std::cout << "ERROR: Failed to read Rs from device.\n";
//         }
//     }
//     ImGui::Separator();
//     ImGui::Text("i_d interpolation threshold");
//     ImGui::Text("(Samples where |i_d| < threshold are interpolated. Avoids R=V/I blowup at very low current.)");
//     {
//         float value = i_d_min_;
//         if (ImGui::InputFloat("Min |i_d| threshold (A)", &value, 0.01f, 0.1f, "%.4f", ImGuiInputTextFlags_EscapeClearsAll)) {
//             i_d_min_ = value;
//         }
//     }
// }


// void gui_mc_thermal_calib::get_test_parameters() {
//     ImGui::Separator();
//     ImGui::Text("Test parameters");
//     {
//         float value = ramp_time_s_;
//         if (ImGui::InputFloat("Ramp time between steps (s)", &value, 0.1f, 1.0f, "%.2f", ImGuiInputTextFlags_EscapeClearsAll)) {
//             ramp_time_s_ = value;
//         }
//     }
//     {
//         float value = cooldown_time_s_;
//         if (ImGui::InputFloat("Cooldown time at zero current (s)", &value, 5.0f, 30.0f, "%.1f", ImGuiInputTextFlags_EscapeClearsAll)) {
//             cooldown_time_s_ = value;
//         }
//     }

//     // Input signal selection
//     helpers::combo_select("D-Axis current command (i_d)", gui_if_->get_f32_input_signal_names(), &signal_in_source_i_d_);
// }


// void gui_mc_thermal_calib::render_profile_editor() {
//     ImGui::Text("Current profile");
//     ImGui::Text("Each step applies a constant d-axis current for the specified duration.");
//     ImGui::Text("A cooldown phase at zero current follows automatically after all steps.");

//     static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg |
//                                          ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable |
//                                          ImGuiTableFlags_NoSavedSettings;
//     if (ImGui::BeginTable("ProfileSteps", 3, table_flags)) {
//         ImGui::TableSetupColumn("Step",       ImGuiTableColumnFlags_WidthFixed, 50.0f);
//         ImGui::TableSetupColumn("Current (A)", ImGuiTableColumnFlags_WidthFixed, 150.0f);
//         ImGui::TableSetupColumn("Duration (s)", ImGuiTableColumnFlags_WidthFixed, 150.0f);
//         ImGui::TableHeadersRow();

//         int remove_idx = -1;
//         for (int i = 0; i < static_cast<int>(profile_.size()); ++i) {
//             ImGui::TableNextRow();
//             ImGui::PushID(i);

//             ImGui::TableSetColumnIndex(0);
//             ImGui::Text("%d", i + 1);

//             ImGui::TableSetColumnIndex(1);
//             ImGui::SetNextItemWidth(120.0f);
//             ImGui::InputFloat("##current", &profile_[i].current_a, 0.1f, 1.0f, "%.2f");

//             ImGui::TableSetColumnIndex(2);
//             ImGui::SetNextItemWidth(120.0f);
//             ImGui::InputFloat("##duration", &profile_[i].duration_s, 1.0f, 10.0f, "%.1f");

//             ImGui::PopID();
//         }
//         ImGui::EndTable();
//     }

//     if (ImGui::Button("Add step")) {
//         float last_current = profile_.empty() ? 1.0f : profile_.back().current_a;
//         profile_.push_back({ last_current, 30.0f });
//     }
//     ImGui::SameLine();
//     if (ImGui::Button("Remove last step") && !profile_.empty()) {
//         profile_.pop_back();
//     }
//     ImGui::SameLine();
//     if (ImGui::Button("Reset to default")) {
//         profile_ = default_profile();
//     }

//     // Compute and display total test time
//     float total_s = cooldown_time_s_;
//     for (auto& step : profile_) {
//         total_s += step.duration_s + ramp_time_s_;
//     }
//     ImGui::Text("Total estimated test time: %.0f s (%.1f min)", total_s, total_s / 60.0f);
// }


// void gui_mc_thermal_calib::render_fit_results() {
//     ImGui::Separator();
//     ImGui::Text("Fit results");
//     ImGui::Text("Status: %s", fit_result_.info.c_str());

//     static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg |
//                                          ImGuiTableFlags_Borders | ImGuiTableFlags_NoSavedSettings;
//     if (ImGui::BeginTable("FitResults", 3, table_flags)) {
//         ImGui::TableSetupColumn("Parameter",  ImGuiTableColumnFlags_WidthFixed, 120.0f);
//         ImGui::TableSetupColumn("Value",      ImGuiTableColumnFlags_WidthFixed, 150.0f);
//         ImGui::TableSetupColumn("Unit",       ImGuiTableColumnFlags_WidthFixed, 80.0f);
//         ImGui::TableHeadersRow();

//         ImGui::TableNextRow();
//         ImGui::TableSetColumnIndex(0); ImGui::Text("R1");
//         ImGui::TableSetColumnIndex(1); ImGui::Text("%.6f", fit_result_.r1);
//         ImGui::TableSetColumnIndex(2); ImGui::Text("K/W");

//         ImGui::TableNextRow();
//         ImGui::TableSetColumnIndex(0); ImGui::Text("C1");
//         ImGui::TableSetColumnIndex(1); ImGui::Text("%.4f", fit_result_.c1);
//         ImGui::TableSetColumnIndex(2); ImGui::Text("J/K");

//         ImGui::TableNextRow();
//         ImGui::TableSetColumnIndex(0); ImGui::Text("tau1 (R1*C1)");
//         ImGui::TableSetColumnIndex(1); ImGui::Text("%.2f", fit_result_.r1 * fit_result_.c1);
//         ImGui::TableSetColumnIndex(2); ImGui::Text("s");

//         if (model_order_ == model_order::second_order_s) {
//             ImGui::TableNextRow();
//             ImGui::TableSetColumnIndex(0); ImGui::Text("R2");
//             ImGui::TableSetColumnIndex(1); ImGui::Text("%.6f", fit_result_.r2);
//             ImGui::TableSetColumnIndex(2); ImGui::Text("K/W");

//             ImGui::TableNextRow();
//             ImGui::TableSetColumnIndex(0); ImGui::Text("C2");
//             ImGui::TableSetColumnIndex(1); ImGui::Text("%.4f", fit_result_.c2);
//             ImGui::TableSetColumnIndex(2); ImGui::Text("J/K");

//             ImGui::TableNextRow();
//             ImGui::TableSetColumnIndex(0); ImGui::Text("tau2 (R2*C2)");
//             ImGui::TableSetColumnIndex(1); ImGui::Text("%.2f", fit_result_.r2 * fit_result_.c2);
//             ImGui::TableSetColumnIndex(2); ImGui::Text("s");
//         }

//         ImGui::TableNextRow();
//         ImGui::TableSetColumnIndex(0); ImGui::Text("RMS error T1");
//         ImGui::TableSetColumnIndex(1); ImGui::Text("%.4f", fit_result_.rms_error_t1);
//         ImGui::TableSetColumnIndex(2); ImGui::Text("K");

//         if (model_order_ == model_order::second_order_s) {
//             ImGui::TableNextRow();
//             ImGui::TableSetColumnIndex(0); ImGui::Text("RMS error T2");
//             ImGui::TableSetColumnIndex(1); ImGui::Text("%.4f", fit_result_.rms_error_t2);
//             ImGui::TableSetColumnIndex(2); ImGui::Text("K");
//         }

//         ImGui::EndTable();
//     }

//     // Copyable result text
//     ImGui::Separator();
//     std::string result_str = "";

//     result_str += "######################################################################## \n";
//     result_str += "# Thermal model parameters\n";
//     if (model_order_ == model_order::first_order_s) {
//         result_str += "thermal_model_mode: thermal_model_mode_1st_order\n";
//         result_str += "thermal_model_r1:   " + std::to_string(fit_result_.r1) + "\n";
//         result_str += "thermal_model_c1:   " + std::to_string(fit_result_.c1) + "\n";
//     } else {
//         result_str += "thermal_model_mode: thermal_model_mode_2nd_order\n";
//         result_str += "thermal_model_r1:   " + std::to_string(fit_result_.r1) + "\n";
//         result_str += "thermal_model_c1:   " + std::to_string(fit_result_.c1) + "\n";
//         result_str += "thermal_model_r2:   " + std::to_string(fit_result_.r2) + "\n";
//         result_str += "thermal_model_c2:   " + std::to_string(fit_result_.c2) + "\n";
//     }

//     result_str += "# Thermal model boundry source\n";
//     result_str += "thermal_model_boundary_source: PLEASE CONFIGURE\n";
//     result_str += "thermal_model_t_ambient:       PLEASE CONFIGURE if thermal_model_boundary_source = thermal_model_boundary_ambient_const\n";

//     result_str += "# Thermal model reference resistance\n";
//     result_str += "thermal_model_rs_t_ref: " + std::to_string(rs_t_ref_) + "\n";
//     result_str += "thermal_model_rs_alpha: " + std::to_string(rs_alpha_) + "\n";

//     helpers::result_text_copyable(result_str);
// }


// void gui_mc_thermal_calib::render_fit_plots() {
//     ImGui::Separator();
//     ImGui::Text("Validation plots");
//     ImGui::Text("Green = measured, Red = predicted from fitted model");

//     plot_t1_.plot();
//     plot_t2_.plot();
//     plot_power_.plot();
// }


// int gui_mc_thermal_calib::ready_test() {
//     {
//         bool ctrl_is_temperature_clamped = false;
//         PARAM_NOTIFY_ERROR( host_->read_bool(target_device_, "temperature_penalty_ctrl_is_clamped",
//             &ctrl_is_temperature_clamped), "Parameter failed: temperature_penalty_ctrl_is_clamped" )

//         if (ctrl_is_temperature_clamped == true) {
//             std::cout << "ERROR: Device control is temperature clamped. Cannot continue with test.\n";
//             return jcs::RET_ERROR;
//         }
//     }
//     if (can_start_) {
//         is_ready_ = true;
//     }
//     helpers::sleep_ms(500);
//     return jcs::RET_OK;
// }

// // ===================================================================
// // Default current profile
// // A multi-level profile to excite both fast (C1) and slow (C2) dynamics.
// // The user can edit this in the UI.
// // ===================================================================
// std::vector<gui_mc_thermal_calib::profile_step> gui_mc_thermal_calib::default_profile() {
//     return {
//         { 3.0f,  30.0f },  // I_high for 30s — fast winding heat up
//         { 1.0f,  30.0f },  // I_low  for 30s — partial cooldown, different power level
//         { 2.0f,  60.0f },  // I_mid  for 60s — intermediate, let housing start moving
//         { 3.0f,  30.0f },  // I_high for 30s — another burst
//     };
//     // Cooldown at zero current follows automatically (configurable duration)
// }