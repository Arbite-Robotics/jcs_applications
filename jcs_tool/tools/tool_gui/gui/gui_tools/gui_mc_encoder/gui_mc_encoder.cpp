// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_mc_encoder.h"
#include <iostream>
#include "jcs_user_external.h"
#include "helpers.h"
#include "imgui_helpers.h"
#include "jcs_dev_motor_controller.h"

//////////////////////////////////////////////////////////////////////
gui_mc_encoder::gui_mc_encoder(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) :
    gui_type_base("Encoder", host, gui_if, target_device),
    encoders_({"encoder_0", "encoder_1"}),
    active_encoder_("encoder_0", 0)
{
    is_ready_ = false;
    i_d_alignment_ = 1.0f;
    i_d_alignment_ramp_time_ms_ = 2000;
    i_d_alignment_settle_time_ms_ = 2000;
    encoder_position_offset_ = 0.0f;
}

int gui_mc_encoder::startup() {
    return jcs::RET_OK;
}

int gui_mc_encoder::step_rt() {
    return jcs::RET_OK;
}

int gui_mc_encoder::render() {
    ImGui::Text("Encoder tool");

    ImGui::Separator();
    if (ImGui::Button("Click to make motor controller ready for tests!")) {
        if (ready_test() != jcs::RET_OK) { return jcs::RET_OK; }
        is_ready_ = true;
    }

    {
        ImGuiDisabled ui_disabled(!is_ready_);
        auto clean_up_on_error = [&]() {
            is_ready_ = false;
        };
        CHECK_CLEANUP_OK( render_zero_encoder(), clean_up_on_error(); )
    }

    return jcs::RET_OK;
}

int gui_mc_encoder::render_zero_encoder() {
    ImGui::Separator();
    ImGui::Text("Zero encoder");
    ImGui::Separator();
    ImGui::Text("This tool will:");
    ImGui::Text(" - Ramp the D-Axis current for the ramp up time.");
    ImGui::Text(" - Allow the rotor to settle for the settle time.");
    ImGui::Text(" - Record the encoder offset.");
    ImGui::NewLine();
    ImGui::Text("Notes:");
    ImGui::Text("- Ensure a large enough D-Axis current to overcome any friction or cogging torque effects.");
    ImGui::Text("- Ensure device is not temperature clamped.");
    ImGui::Text("- Ensure anti-cogging feature is disabled.");
    ImGui::Text("- Ensure settling time is sufficient to allow the rotor to come to a complete standstill.");
    ImGui::NewLine();

    ImGui::Separator();
    ImGui::Text("Select encoder");
    helpers::combo_select("Active encoder", &encoders_, &active_encoder_);

    ImGui::Separator();
    {
        float value = i_d_alignment_;
        if (ImGui::InputFloat("D-Axis alignment current (A)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            i_d_alignment_ = value;
        }
    }
    {
        int value = (int)i_d_alignment_ramp_time_ms_;
        if (ImGui::InputInt("D-Axis alignment ramp up time (ms)", &value, 1, 10, ImGuiInputTextFlags_EscapeClearsAll)) {
            i_d_alignment_ramp_time_ms_ = (uint16_t)value;
        }
    }
    {
        int value = (int)i_d_alignment_settle_time_ms_;
        if (ImGui::InputInt("D-Axis alignment settle time (ms)", &value, 1, 10, ImGuiInputTextFlags_EscapeClearsAll)) {
            i_d_alignment_settle_time_ms_ = (uint16_t)value;
        }
    }

    if (ImGui::Button("Start zero")) {
        {
            bool ctrl_is_temperature_clamped = false;
            PARAM_NOTIFY_ERROR( host_->read_bool(target_device_,  "temperature_penalty_ctrl_is_clamped", &ctrl_is_temperature_clamped), "Parameter failed: temperature_penalty_ctrl_is_clamped" )

            if (ctrl_is_temperature_clamped) {
                std::cout << "ERROR: Device control is temperature clamped. Cannot continue with test.\n";
                return jcs::RET_ERROR;
            }
        }

        PARAM_NOTIFY_ERROR( host_->write_float(target_device_,  "i_d_alignment", i_d_alignment_), "Parameter failed: i_d_alignment" )
        PARAM_NOTIFY_ERROR( host_->write_uint16(target_device_, "i_d_alignment_ramp_time_ms",   i_d_alignment_ramp_time_ms_),   "Parameter failed: i_d_alignment_ramp_time_ms" )
        PARAM_NOTIFY_ERROR( host_->write_uint16(target_device_, "i_d_alignment_settle_time_ms", i_d_alignment_settle_time_ms_), "Parameter failed: i_d_alignment_settle_time_ms" )
        // Configure controller mode into align mode
        PARAM_NOTIFY_ERROR( host_->write_enum(target_device_,   "controller_mode", "test_align"), "Parameter failed: controller_mode" )
        // Starting the controller starts the test
        PARAM_NOTIFY_ERROR( host_->write_command(target_device_, "controller_start"), "Parameter failed: controller_start" )

        // Wait for the dwell period
        bool in_dwell = false;
        long int time_start_dwell_ms = helpers::time_now_ms();
        long int dwell_timeout_ms = (long int)i_d_alignment_ramp_time_ms_ + i_d_alignment_settle_time_ms_ + 6000;
        while (!in_dwell) {
            // Note: This blocks the UI - TODO small state machine?
            PARAM_NOTIFY_ERROR( host_->read_bool(target_device_, "align_in_dwell", &in_dwell), "Parameter failed: align_in_dwell" )
            helpers::sleep_ms(100);

            if ( (helpers::time_now_ms() - time_start_dwell_ms) > dwell_timeout_ms) {
                std::cout << "ERROR: Timeout waiting for dwell. Motor controller might be in an error state.\n";
                return jcs::RET_ERROR;
            }
        }

        std::string cmd_position_zero = active_encoder_.source_ + "_position_zero";
        PARAM_NOTIFY_ERROR( host_->write_command(target_device_, cmd_position_zero), "Parameter failed: " + cmd_position_zero )
        // Stop the test
        PARAM_NOTIFY_ERROR( host_->write_command(target_device_, "controller_stop"), "Parameter failed: controller_stop" )
        helpers::sleep_ms(200);
        std::string cmd_position_offset = active_encoder_.source_ + "_position_offset";
        PARAM_NOTIFY_ERROR( host_->read_float(target_device_, cmd_position_offset, &encoder_position_offset_), "Parameter failed: " + cmd_position_offset )
    }

    ImGui::Separator();
    std::string string_position_offset = active_encoder_.source_ + "_position_offset: ";
    helpers::result_text_copyable(string_position_offset, encoder_position_offset_);

    return jcs::RET_OK;
}

int gui_mc_encoder::ready_test() {
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Disable the host sentry as we do not plan to enter synchronous mode     
    PARAM_NOTIFY_ERROR( host_->write_bool(target_device_, "host_sentry_active", false), "Parameter failed: host_sentry_active" )
    // Start the motor controller and wait for it to calibrate
    PARAM_NOTIFY_ERROR( host_->write_command(target_device_, "start"), "Parameter failed: start" )
    helpers::sleep_ms(500);
    return jcs::RET_OK;
}