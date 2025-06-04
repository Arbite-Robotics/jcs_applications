// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_mc_encoder.h"
#include <iostream>
#include "jcs_user_external.h"
#include "helpers.h"

#include "jcs_dev_motor_controller.h"

//////////////////////////////////////////////////////////////////////
gui_mc_encoder::gui_mc_encoder(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) :
    gui_type_base("Encoder", host, gui_if, target_device),
    encoders_({"encoder_0", "encoder_1"}),
    active_encoder_("encoder_0", 0)
{
    is_ready_ = false;
    i_d_alignment_ = 1.0f;
    i_d_alignment_ramp_time_ms_ = 2000.0f;
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
        if (ready_test() != jcs::RET_OK) {
            // Test failed - just return
            return jcs::RET_OK;
        }
        is_ready_ = true;
    }

    if (!is_ready_) {
        ImGui::BeginDisabled();
    }

    // Cleanup function
    auto clean_up_on_error = [&]() {
        if (!is_ready_) {
            ImGui::EndDisabled();
        }
        is_ready_ = false;
    };

    CHECK_CLEANUP_OK( render_zero_encoder(), clean_up_on_error(); )

    // Cleanup, but don't clear is_ready here
    if (!is_ready_) {
        ImGui::EndDisabled();
    }

    return jcs::RET_OK;
}

int gui_mc_encoder::render_zero_encoder() {
    ImGui::Separator();
    ImGui::Text("Zero encoder");
    ImGui::Separator();
    ImGui::Text("This tool will ramp the D-Axis current, then record the encoder offset.");
    ImGui::Text("Notes:");
    ImGui::Text("- Ensure a large enough D-Axis current to overcome any friction or cogging torque effects.");
    ImGui::Text("- Ensure device is not temperature clamped.");
    ImGui::Text("- Ensure anti-cogging feature is disabled.");
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
        float value = i_d_alignment_ramp_time_ms_;
        if (ImGui::InputFloat("D-Axis alignment ramp up time (ms)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            i_d_alignment_ramp_time_ms_ = value;
        }
    }
    if (ImGui::Button("Start zero")) {

        {
            bool ctrl_is_temperature_clamped = false;
            PARAM_NOTIFY_ERROR( host_->read_bool(target_device_,  "temperature_penalty_ctrl_is_clamped", &ctrl_is_temperature_clamped), "Parameter failed: temperature_penalty_ctrl_is_clamped" )

            if (ctrl_is_temperature_clamped == true) {
                std::cout << "ERROR: Device control is temperature clamped. Cannot continue with test.\n";
                return jcs::RET_ERROR;
            }
        }

        PARAM_NOTIFY_ERROR( host_->write_float(target_device_,  "i_d_alignment", i_d_alignment_), "Parameter failed: i_d_alignment" )
        PARAM_NOTIFY_ERROR( host_->write_float(target_device_,  "i_d_alignment_ramp_time_ms", i_d_alignment_ramp_time_ms_), "Parameter failed: i_d_alignment_ramp_time_ms" )

        // Configure controller mode into align mode
        PARAM_NOTIFY_ERROR( host_->write_enum(target_device_,   "controller_mode", "test_align"), "Parameter failed: controller_mode" )
        // Starting the controller starts the test
        PARAM_NOTIFY_ERROR( host_->write_command(target_device_, "controller_start"), "Parameter failed: controller_start" )

        // Wait for the dwell period
        bool in_dwell = false;
        long int time_start_dwell_ms = helpers::time_now_ms();
        long int dwell_timeout_ms = (long int)i_d_alignment_ramp_time_ms_ + 6000;
        while (!in_dwell) {
            PARAM_NOTIFY_ERROR( host_->read_bool(target_device_, "align_in_dwell", &in_dwell), "Parameter failed: align_in_dwell" )
            helpers::sleep_ms(100);

            if ( (helpers::time_now_ms() - time_start_dwell_ms) > dwell_timeout_ms) {
                std::cout << "ERROR: Timeout waiting for dwell. Motor controller might be in an error state.\n";
                return jcs::RET_ERROR;
            }
        }
        // Snooze so we are well within the dwell period
        helpers::sleep_ms(3000);

        // Zero the encoder
        switch (active_encoder_.index_) {
            default:
            case 0:
                PARAM_NOTIFY_ERROR( host_->write_command(target_device_, "encoder_0_position_zero"), "Parameter failed: encoder_0_position_zero" )
                break;
            case 1:
                PARAM_NOTIFY_ERROR( host_->write_command(target_device_, "encoder_1_position_zero"), "Parameter failed: encoder_1_position_zero" )
                break;
        }

        // Stop the test
        PARAM_NOTIFY_ERROR( host_->write_command(target_device_, "controller_stop"), "Parameter failed: controller_stop" )

        helpers::sleep_ms(200);

        // Read back the zero position
        switch (active_encoder_.index_) {
            default:
            case 0:
                PARAM_NOTIFY_ERROR( host_->read_float(target_device_, "encoder_0_position_offset", &encoder_position_offset_), "Parameter failed: encoder_0_position_offset" )
                break;
            case 1:
                PARAM_NOTIFY_ERROR( host_->read_float(target_device_, "encoder_1_position_offset", &encoder_position_offset_), "Parameter failed: encoder_1_position_offset" )
                break;
        }
    }

    ImGui::Separator();
    switch (active_encoder_.index_) {
        default:
        case 0:
            helpers::result_text_copyable("encoder_0_position_offset: ", encoder_position_offset_);
            break;
        case 1:
            helpers::result_text_copyable("encoder_1_position_offset: ", encoder_position_offset_);
            break;
    }

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