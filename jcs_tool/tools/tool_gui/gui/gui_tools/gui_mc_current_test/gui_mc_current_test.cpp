// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_mc_current_test.h"
#include <iostream>
#include "jcs_user_external.h"
#include "jcs_dev_motor_controller.h"

//////////////////////////////////////////////////////////////////////
gui_mc_current_test::gui_mc_current_test(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) :
    gui_type_base("Current Test", host, gui_if, target_device),
    sampler_(host->base_frequency_get(), gui_if->get_f32_output_signal_names(), 4, 10, 10),
    i_ramp_(1.0 / static_cast<double>(host_->base_frequency_get())),
    i_rotate_(1.0 / static_cast<double>(host_->base_frequency_get())),
    is_ready_(false),
    signal_in_source_i_d_("None", 0),
    signal_in_source_th_m_("None", 0),
    signal_in_source_w_m_("None", 0),
    signal_out_th_m_0_idx_(0), signal_out_w_m_0_idx_(0), signal_out_i_d_idx_(0)
{
    test_current_ = 1.0f;
    ramp_time_s_ = 1.0f;
    dwell_time_s_ = 1.0f;
    rotate_speed_rads_ = 12.566f; // 2 RPS
}

int gui_mc_current_test::startup() {
    if (sampler_.startup((double)jcs::external::time_now_ns()) != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }
    // Confiugure line storage
    f32_output_signal_store_.resize(host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0));
    f32_input_signal_store_.resize(host_->sig_input_sz_unsafe_rt(jcs::signal_type::float32_s, 0));

    // Signal index helpers
    can_start_ = true;
    if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::th_m_encoder_0", &signal_out_th_m_0_idx_) != jcs::RET_OK) { can_start_ = false; }
    if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::w_m_0", &signal_out_w_m_0_idx_) != jcs::RET_OK)   { can_start_ = false; }
    if (helpers::signals_names_contains(gui_if_->get_f32_output_signal_names(), target_device_+"::i_d", &signal_out_i_d_idx_) != jcs::RET_OK)       { can_start_ = false; }

    required_input_signal_names_ = { "HOST::th_m",
                                     "HOST::w_m",
                                     "HOST::i_d" };

    required_output_signal_names_ = { target_device_+"::th_m_encoder_0",
                                      target_device_+"::w_m_0",
                                      target_device_+"::i_d" };

    return jcs::RET_OK;
}

int gui_mc_current_test::step_rt() {
    host_->sig_input_set_rt(0, f32_input_signal_store_);
    host_->sig_output_get_rt(0, &f32_output_signal_store_);

    sampler_.step_rt((double)jcs::external::time_now_ns(), &f32_output_signal_store_);

    switch (state_) {
        default:
        case state::finish_s:
        case state::off_s:
            break;

        case state::ramp_to_current_s:
            f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = i_ramp_.step();
            if (!i_ramp_.is_done()) {
                break;
            }
            i_rotate_.start_speed(0.0, rotate_speed_rads_, true, sampler_.get_sample_time_s());
            sampler_.start();
            state_ = state::rotate_s;
            // Fall through when done
        case state::rotate_s:
            f32_input_signal_store_[ signal_in_source_th_m_.index_ ] = i_rotate_.step();
            f32_input_signal_store_[ signal_in_source_w_m_.index_ ] = i_rotate_.omega();
            if (i_rotate_.is_done()) {
                f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = 0.0f;
                sampler_.stop();
                state_ = state::finish_s;
            }
            break;
    }
    return jcs::RET_OK;
}

int gui_mc_current_test::step_rt_always() {
    return jcs::RET_OK;
}

int gui_mc_current_test::render() {

    ImGui::Text("Current test tool");
    ImGui::Separator();
    ImGui::Text("This tool ramps, then applies a test current to the motor.");
    ImGui::Text("While the test current is applied, the motor is rotated to distribute heat effects evenly about the motor and controller power electronics.");
    ImGui::Text("Use this tool to record the motor or controller temperature under constant current application, e.g. obtaining motor temperature response.");
    ImGui::Text("Notes:");
    ImGui::Text("- Ensure correct configuration is used. Current controllers tuned and encoder zeroed");
    ImGui::Text("- Ensure motor configuration has parameter estimator_0_theta_passthrough set to yes.");
    ImGui::Text("- Ensure device is not temperature clamped.");
    ImGui::Text("- Ensure motor can spin freely.");

    ImGui::Separator();
    helpers::input_signals_check(gui_if_->get_f32_input_signal_names(), &required_input_signal_names_);
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
    
    sampler_.render_interface();
    ImGui::Separator();
    
    // Controls
    ImGui::Text("Starting this test will start JCS system. Ensure it is safe to do so.");
    if (ImGui::Button("Start test")) {
        switch (state_) {
            default:
            case state::off_s:
                f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = 0.0f;
                f32_input_signal_store_[ signal_in_source_th_m_.index_ ] = 0.0f;
                f32_input_signal_store_[ signal_in_source_w_m_.index_ ] = 0.0f;
                i_ramp_.start(0.0, test_current_, ramp_time_s_, 0.5, dwell_time_s_);
                // Start JCS host
                if (gui_if_->start() != jcs::RET_OK) {
                    state_ = state::off_s;
                    break;
                }
                state_ = state::ramp_to_current_s;
                break;
            case state::ramp_to_current_s:
            case state::rotate_s:
            case state::finish_s:
                break;
        }
    }

    ImGui::SameLine();
    if (ImGui::Button("Stop test")) {
        switch (state_) {
            default:
            case state::off_s:
                break;
            case state::ramp_to_current_s:
            case state::rotate_s:
            case state::finish_s:
                f32_input_signal_store_[ signal_in_source_i_d_.index_ ] = 0.0f;
                sampler_.stop();
                gui_if_->stop();
                state_ = state::off_s;
                break;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Force sampler start")) {
        sampler_.start();
    }
    ImGui::Separator();

    ImGui::Text("Rotator State: ");
    ImGui::SameLine();
    switch (state_) {
        default:
        case state::off_s:
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.0f, 1.0f), "Off");
            break;
        case state::ramp_to_current_s:
            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Ramping");
            break;
        case state::rotate_s:
            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Rotating");
            break;

        case state::finish_s:
            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Stopping");
            sampler_.stop();
            gui_if_->stop();
            state_ = state::off_s;
            break;
    }
    ImGui::Separator();

    sampler_.render_status();
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
        ImGui::TableSetColumnIndex(0); ImGui::Text("w_m_0");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%.6f", f32_input_signal_store_[ signal_in_source_w_m_.index_ ]);
        ImGui::TableSetColumnIndex(2); ImGui::Text("%.6f", f32_output_signal_store_[signal_out_w_m_0_idx_]);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("i_d");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%.6f", f32_input_signal_store_[ signal_in_source_i_d_.index_ ]);
        ImGui::TableSetColumnIndex(2); ImGui::Text("%.6f", f32_output_signal_store_[signal_out_i_d_idx_]);

        ImGui::EndTable();
    }
    ImGui::Separator();

    sampler_.render_plots();
    ImGui::Separator();

    sampler_.channels_write_to_file();

    // Cleanup, but don't clear is_ready here
    if (!is_ready_) {
        ImGui::EndDisabled();
    }

    return jcs::RET_OK;
}

int gui_mc_current_test::ready_test() {
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

void gui_mc_current_test::get_test_parameters() {
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
    helpers::combo_select("D-Axis current input", gui_if_->get_f32_input_signal_names(), &signal_in_source_i_d_);
    helpers::combo_select("Rotor theta input",    gui_if_->get_f32_input_signal_names(), &signal_in_source_th_m_);
    helpers::combo_select("Rotor omega input",    gui_if_->get_f32_input_signal_names(), &signal_in_source_w_m_);
}