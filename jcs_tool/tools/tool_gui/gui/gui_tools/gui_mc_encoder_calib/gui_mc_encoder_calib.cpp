// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_mc_encoder_calib.h"
#include <iostream>
#include "jcs_user_external.h"
#include "jcs_dev_motor_controller.h"
#include <math.h>
#include "ImGuiFileDialog.h"

//////////////////////////////////////////////////////////////////////
gui_mc_encoder_calib::gui_mc_encoder_calib(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) :
    gui_type_base("Encoder Calibrator", host, gui_if, target_device),
    encoders_({"encoder_0", "encoder_1"}),
    active_encoder_("encoder_0", 0),
    configured_encoder_("encoder_0"), configured_estimator_("estimator_0"),
    state_(state::off_s),
    th_m_orig_("Theta reference and recorded", "Time (s)", "Theta (rad)", test_time_s_, host->base_frequency_get()),
    th_m_error_("Theta error", "Time (s)", "Theta (rad)", test_time_s_, host->base_frequency_get()),
    th_m_corrected_("Theta corrected", "Time (s)", "Theta (rad)", test_time_s_, host->base_frequency_get()),
    i_ramp_(1.0 / static_cast<double>(host_->base_frequency_get())),
    i_rotate_(1.0 / static_cast<double>(host_->base_frequency_get())),
    is_ready_(false),
    signal_in_source_i_d_("None", 0),
    signal_in_source_th_m_("None", 0),
    signal_out_th_m_0_idx_(0), 
    signal_out_i_d_idx_(0)
{
    test_current_ = 1.0f;
    ramp_time_s_ = 1.0f;
    dwell_time_s_ = 1.0f;
    rotation_tick_ = 0;
    // 1 revolution over test time
    rotate_speed_rads_ = (float)(2.0 * M_PI / (double)test_time_s_);

    th_m_orig_.add_channel("Reference", ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
    th_m_orig_.add_channel("Recorded",  ImVec4(0.0f, 1.0f, 0.0f, 1.0f));

    th_m_error_.add_channel("Corrected", ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
    th_m_error_.add_channel("Recorded",  ImVec4(0.0f, 1.0f, 0.0f, 1.0f));

    th_m_corrected_.add_channel("Corrected", ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
    th_m_corrected_.add_channel("Recorded",  ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
    th_m_corrected_.add_channel("Reference", ImVec4(0.0f, 0.0f, 1.0f, 1.0f));

    // Plot channel pointers
    th_m_orig_reference_ = th_m_orig_.get_channel("Reference");
    th_m_orig_recorded_  = th_m_orig_.get_channel("Recorded");

    th_m_error_corrected_ = th_m_error_.get_channel("Corrected");
    th_m_error_recorded_  = th_m_error_.get_channel("Recorded");

    th_m_corrected_corrected_ = th_m_corrected_.get_channel("Corrected");
    th_m_corrected_recorded_  = th_m_corrected_.get_channel("Recorded");
    th_m_corrected_reference_ = th_m_corrected_.get_channel("Reference");

    // Initialise corrector storage
    mc_encoder_corrector::initialise_correction_table(&correction_table_, calib_points_);
}

int gui_mc_encoder_calib::startup() {
    state_ = state::off_s;
    // Confiugure line storage
    f32_output_signal_store_.resize(host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0));
    f32_input_signal_store_.resize(host_->sig_input_sz_unsafe_rt(jcs::signal_type::float32_s, 0));

    // Signal index helpers
    can_start_ = true;
    if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::th_m_0", &signal_out_th_m_0_idx_) != jcs::RET_OK) { can_start_ = false; }
    if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::i_d",    &signal_out_i_d_idx_) != jcs::RET_OK)    { can_start_ = false; }

    required_output_signal_names_ = { target_device_+"::th_m_0", target_device_+"::i_d" };

    rotation_tick_ = 0;
    return jcs::RET_OK;
}

int gui_mc_encoder_calib::step_rt() {
    host_->sig_output_get_rt(0, &f32_output_signal_store_);

    switch (state_) {
        default:
        case state::finish_s:
        case state::off_s:
            break;

        case state::initialise_s:
            break;

        case state::ramp_to_current_s:
            // Ramp up nicely
            f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = i_ramp_.step();
            if (!i_ramp_.is_done()) {
                break;
            }
            host_->sig_input_set_rt(0, f32_input_signal_store_);

            i_rotate_.start_speed(0.0, rotate_speed_rads_, true, test_time_s_);
            rotation_tick_ = 0;
            state_ = state::rotate_s;
            // Fall through when done

        case state::rotate_s:
            // Set theta command
            f32_input_signal_store_[ signal_in_source_th_m_.index_ ] = i_rotate_.step();
            host_->sig_input_set_rt(0, f32_input_signal_store_);

            if (i_rotate_.is_done()) {
                // Start the ramp down
                i_ramp_.start(test_current_, 0.0, ramp_time_s_, 0.5, 0.5);
                state_ = state::finish_ramp_s;
                break;
            }
            {
                float th_reference = static_cast<double>(helpers::angle_norm_2pi( f32_input_signal_store_[signal_in_source_th_m_.index_] ));
                float th_recorded = static_cast<double>(helpers::angle_norm_2pi( f32_output_signal_store_[signal_out_th_m_0_idx_] ));
                th_m_orig_reference_->y_.at(rotation_tick_) = th_reference;
                th_m_orig_recorded_->y_.at(rotation_tick_) = th_recorded;
                // Compute and store the error
                th_m_error_recorded_->y_.at(rotation_tick_) = helpers::angle_norm_pipi(th_reference - th_recorded);
            }
            rotation_tick_++;
            break;

        case state::finish_ramp_s:
            // Ramp fown nicely to stop the rotor from jumping when setting D=0
            f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = i_ramp_.step();
            host_->sig_input_set_rt(0, f32_input_signal_store_);
            if (i_ramp_.is_done()) {
                state_ = state::finish_s;
            }
            break;
    }
    return jcs::RET_OK;
}

int gui_mc_encoder_calib::step_rt_always() {
    return jcs::RET_OK;
}

int gui_mc_encoder_calib::render() {
    switch (state_) {
        default:
        case state::finish_ramp_s:
        case state::off_s:
            break;

        case state::initialise_s:
            // Store config
            PARAM_NOTIFY_ACTION( host_->read_bool(target_device_, configured_encoder_+"_theta_bypass",           &conf_enc_theta_bypass_),          "Parameter failed: "+configured_encoder_+"_theta_bypass",           state_ = state::off_s; break; )
            PARAM_NOTIFY_ACTION( host_->read_bool(target_device_, configured_encoder_+"_theta_bypass_direction", &conf_enc_theta_bypass_direction_),"Parameter failed: "+configured_encoder_+"_theta_bypass_direction", state_ = state::off_s; break; )
            PARAM_NOTIFY_ACTION( host_->read_bool(target_device_, configured_estimator_+"_theta_passthrough",    &conf_est_theta_passthrough_),     "Parameter failed: "+configured_estimator_+"_theta_passthrough",    state_ = state::off_s; break; )
            // Configure for test
            PARAM_NOTIFY( host_->write_bool(target_device_, configured_encoder_+"_theta_bypass",           true), "Parameter failed: "+configured_encoder_+"_theta_bypass" )
            PARAM_NOTIFY( host_->write_bool(target_device_, configured_encoder_+"_theta_bypass_direction", true), "Parameter failed: "+configured_encoder_+"_theta_bypass_direction" )
            PARAM_NOTIFY( host_->write_bool(target_device_, configured_estimator_+"_theta_passthrough",    true), "Parameter failed: "+configured_estimator_+"_theta_passthrough" )

            // Stop and reset so encoder parameters are sampled
            gui_if_->stop();
            gui_if_->reset();

            // Clear the corrected plots
            std::fill(th_m_corrected_corrected_->y_.begin(), th_m_corrected_corrected_->y_.end(), 0.0f);
            std::fill(th_m_error_corrected_->y_.begin(), th_m_error_corrected_->y_.end(), 0.0f);

            f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = 0.0f;
            f32_input_signal_store_[ signal_in_source_th_m_.index_ ] = 0.0f;
            i_ramp_.start(0.0, test_current_, ramp_time_s_, 0.5, dwell_time_s_);
            rotation_tick_ = 0;
            // Start JCS host
            if (gui_if_->start() != jcs::RET_OK) {
                state_ = state::finish_s;
                break;
            }
            // Start the current controller
            PARAM_NOTIFY_ACTION( host_->write_enum(target_device_,    "controller_mode", "current_dq"), "Parameter failed: controller_mode", state_ = state::finish_s; break; )
            PARAM_NOTIFY_ACTION( host_->write_command(target_device_, "controller_start"), "Parameter failed: controller_start", state_ = state::finish_s; break; )

            state_ = state::ramp_to_current_s;
            break;

        case state::ramp_to_current_s:
        case state::rotate_s:
            break;
        case state::finish_s:
            // Restore config
            PARAM_NOTIFY( host_->write_bool(target_device_, configured_encoder_+"_theta_bypass",           conf_enc_theta_bypass_),          "Parameter failed: "+configured_encoder_+"_theta_bypass" )
            PARAM_NOTIFY( host_->write_bool(target_device_, configured_encoder_+"_theta_bypass_direction", conf_enc_theta_bypass_direction_),"Parameter failed: "+configured_encoder_+"_theta_bypass_direction" )
            PARAM_NOTIFY( host_->write_bool(target_device_, configured_estimator_+"_theta_passthrough",    conf_est_theta_passthrough_),     "Parameter failed: "+configured_estimator_+"_theta_passthrough" )
            f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = 0.0f;
            gui_if_->stop();
            gui_if_->reset();
            state_ = state::off_s;
            break;
    }

    ImGui::Text("Encoder lineariser tool");
    ImGui::Separator();
    ImGui::Text("This tool generates encoder linearisation coefficients.");
    ImGui::Text("When run, the tool applies a test current and rotates the motor through one revolution.");
    ImGui::Text("The rotor reference and encoder angle are recorded. Coefficients are generated from this data.");
    ImGui::Text("Iterate the test as you please to improve the linearisation, but note diminishing returns after a couple of iterations.");
    ImGui::Text(" ");
    ImGui::Text("Notes:");
    ImGui::Text("- Ensure correct configuration is used, current controllers are tuned and is encoder zeroed.");
    ImGui::Text("- Ensure device is not temperature clamped.");
    ImGui::Text("- Ensure motor can spin freely.");
    ImGui::Text(" ");
    ImGui::Text("You should configure:");
    ImGui::Text("- Active encoder.");
    ImGui::Text("- Test current - Should be high enough to solidly lock the rotor.");
    ImGui::Text("- D-axis input signal (eg: i_d).");
    ImGui::Text("- Theta input signal (eg: th_m).");
    ImGui::Text(" ");

    ImGui::Separator();
    helpers::output_signals_check(gui_if_->get_f32_output_signal_names(), &required_output_signal_names_);

    ImGui::Separator();
    if (ImGui::Button("Click to make motor controller ready for tests!")) {
        if (ready_test() != jcs::RET_OK) {
            // Test failed - just return
            return jcs::RET_OK;
        }
    }

    if (!can_start_) {
        ImGui::Text("Can't start! Most likely missing signal.");
    }
    if (!is_ready_) {
        ImGui::BeginDisabled();
    }

    get_test_parameters();

    ImGui::Separator();    
    // Controls
    ImGui::Text("Starting this test will start JCS system. Ensure it is safe to do so.");
    if (ImGui::Button("Start test")) {
        if (state_ == state::off_s) {
            state_ = state::initialise_s;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop test")) {
        if (state_ != state::off_s) {
            state_ = state::finish_s;
        }
    }

    ImGui::Separator();
    ImGui::Text("Rotator State: ");
    ImGui::SameLine();
    switch (state_) {
        default:
        case state::off_s:             ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.0f, 1.0f), "Off");      break;
        case state::initialise_s:      // Fallthrough
        case state::ramp_to_current_s: ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Ramping");  break;
        case state::rotate_s:          ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Rotating"); break;
        case state::finish_ramp_s:      // Fallthrough
        case state::finish_s:          ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Stopping"); break;
    }

    // Some nice stats
    static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | 
                                         ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings;
    if (ImGui::BeginTable("Measurements", 3, table_flags)) {
        ImGui::TableSetupColumn("Signal", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Commanded value", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Measured value", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("th_m_0");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%.6f", f32_input_signal_store_[ signal_in_source_th_m_.index_ ]);
        ImGui::TableSetColumnIndex(2); ImGui::Text("%.6f", f32_output_signal_store_[signal_out_th_m_0_idx_]);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("i_d");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%.6f", f32_input_signal_store_[ signal_in_source_i_d_.index_ ]);
        ImGui::TableSetColumnIndex(2); ImGui::Text("%.6f", f32_output_signal_store_[signal_out_i_d_idx_]);

        ImGui::EndTable();
    }

    ImGui::Separator();
    th_m_orig_.plot();

    ImGui::Separator();
    ImGui::Text("Calibration");
    if (ImGui::Button("Compute new calibration")) {
        mc_encoder_corrector::build_correction_table(&correction_table_, th_m_orig_reference_->y_, th_m_orig_recorded_->y_, calib_points_);
        // Apply the correction and sundry
        for (int i=0; i<th_m_orig_reference_->y_.size(); ++i) {
            float th_m = static_cast<float>(th_m_orig_recorded_->y_.at(i));
            float th_corrected = mc_encoder_corrector::apply_correction(th_m, correction_table_.corrections_increment);
            // Store the corrected data
            th_m_corrected_corrected_->y_.at(i) = th_corrected;
            // Store the original data for plotting - recorded
            th_m_corrected_reference_->y_.at(i) = th_m_orig_reference_->y_.at(i);
            // Store the original data for plotting - reference
            th_m_corrected_recorded_->y_.at(i) = th_m_orig_recorded_->y_.at(i);
            // compute the corrected error
            th_m_error_corrected_->y_.at(i) = helpers::angle_norm_pipi(th_m_orig_reference_->y_.at(i) - th_corrected);
            // Original error computed during test run
        }
        // Print calib factors
        // std::cout << "Correction values:\n";
        // for (int i=0; i<correction_table_.corrections.size()-1; i++) {
        //     std::cout << correction_table_.corrections[i] << ", ";
        // }
        // std::cout << correction_table_.corrections[correction_table_.corrections.size()-1] << "\n";
    }
    ImGui::SameLine();
    if (ImGui::Button("Write calibration to device")) {
        host_->write_float(target_device_, configured_encoder_+"_linearisation_coeffs", correction_table_.corrections);
    }

    ImGui::Separator();
    ImGui::Text("Tools");
    if (ImGui::Button("Clear calibration")) {
        // Clear the device calibration
        std::vector<float> cleared;
        cleared.resize(calib_points_);
        std::fill(cleared.begin(), cleared.end(), 0.0f);
        host_->write_float(target_device_, configured_encoder_+"_linearisation_coeffs", cleared);
        // Clear the corrector
        mc_encoder_corrector::clear_correction_table(&correction_table_);
    }
    ImGui::SameLine();
    if (ImGui::Button("Read calibration from device")) {
        host_->read_float(target_device_, configured_encoder_+"_linearisation_coeffs", &correction_table_.corrections);
    }

    ImGui::Separator();
    ImGui::Text("Encoder RMS error  : %7.5f", correction_table_.rms_error);
    ImGui::Text("Corrected RMS error: %7.5f", correction_table_.rms_error_corrected);

    ImGui::Separator();
    helpers::result_text_copyable(configured_encoder_+"_linearisation_coeffs: ", 9, correction_table_.corrections, 8);

    ImGui::Separator();
    th_m_error_.plot();
    th_m_corrected_.plot();

    // Cleanup, but don't clear is_ready here
    if (!is_ready_) {
        ImGui::EndDisabled();
    }

    return jcs::RET_OK;
}

int gui_mc_encoder_calib::ready_test() {
    {
        bool ctrl_is_temperature_clamped = false;
        PARAM_NOTIFY_ERROR( host_->read_bool(target_device_, "temperature_penalty_ctrl_is_clamped", &ctrl_is_temperature_clamped), "Parameter failed: temperature_penalty_ctrl_is_clamped" )

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

void gui_mc_encoder_calib::get_test_parameters() {
    ImGui::Separator();

    ImGui::Text("Select encoder");
    helpers::combo_select("Active encoder", &encoders_, &active_encoder_);
    switch (active_encoder_.index_) {
        default:
        case 0: configured_encoder_ = "encoder_0"; configured_estimator_ = "estimator_0"; break;
        case 1: configured_encoder_ = "encoder_1"; configured_estimator_ = "estimator_0"; break;
    }
    ImGui::Separator();
    // Get test parameters
    {
        float value = test_current_;
        if (ImGui::InputFloat("Test current (A)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            test_current_ = value;
        }
    }
    {
        float value = ramp_time_s_;
        if (ImGui::InputFloat("Ramp up time (s)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            ramp_time_s_ = value;
        }
    }
    {
        float value = dwell_time_s_;
        if (ImGui::InputFloat("Ramp dwell time (s)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            dwell_time_s_ = value;
        }
    }
    {
        float value = rotate_speed_rads_;
        if (ImGui::InputFloat("Motor mechanical rotation speed (rad/s)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            rotate_speed_rads_ = value;
        }
    }
    // Input signal selection
    helpers::combo_select("i_d command", gui_if_->get_f32_input_signal_names(), &signal_in_source_i_d_);
    helpers::combo_select("th_m command",    gui_if_->get_f32_input_signal_names(), &signal_in_source_th_m_);
}
