// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_bc_tune.h"
#include <iostream>
#include "helpers.h"
#include <cmath>

#include "jcs_dev_motor_controller.h"

//////////////////////////////////////////////////////////////////////
gui_bc_tune::gui_bc_tune(jcs::jcs_host* host, std::string const& target_device) :
    gui_type_base("Tuning", host, target_device)
{
    channel_combo_idx_ = 0;
    active_channel_ = "controller_0";
}

int gui_bc_tune::startup() {
    return jcs::RET_OK;
}

int gui_bc_tune::step_rt() {
    return jcs::RET_OK;
}

int gui_bc_tune::render() {

    ImGui::Text("Measure resistance and inductance");
    // ImGui::Text("Notes:");
    // ImGui::Text("- Ensure device is not temperature clamped.");

    ImGui::Separator();

    // Choose channel
    helpers::combo_select("Controller channel", &channels_, &channel_combo_idx_, &active_channel_);
    
    /////////////////////////////////////////////////////////////////////////////////////////////
    if (active_channel_ == "controller_0") {
        test_r_get_parameters("controller_0", &test_r_parameters_[0]);
        test_l_get_parameters("controller_0", &test_l_parameters_[0]);
    } else
    if (active_channel_ == "controller_1") {
        test_r_get_parameters("controller_1", &test_r_parameters_[1]);
        test_l_get_parameters("controller_1", &test_l_parameters_[1]);
    }

    if (ImGui::Button("Measure resistance and inductance")) {
        if (active_channel_ == "controller_0") {
            if (do_test_r(&test_r_parameters_[0], "controller_0") != jcs::RET_OK) { return jcs::RET_OK; }
            if (do_test_l(&test_l_parameters_[0], "controller_0") != jcs::RET_OK) { return jcs::RET_OK; }
        } else
        if (active_channel_ == "controller_1") {
            if (do_test_r(&test_r_parameters_[1], "controller_1") != jcs::RET_OK) { return jcs::RET_OK; }
            if (do_test_l(&test_l_parameters_[1], "controller_1") != jcs::RET_OK) { return jcs::RET_OK; }
        }
    }

    return jcs::RET_OK;
}


void gui_bc_tune::test_r_get_parameters(std::string const& channel, test_r* storage) {
    ImGui::PushID((channel + "r").c_str());
    ImGui::Separator();
    ImGui::Text("Resistance test parameters for channel: %s", channel.c_str());

    {
        float value = storage->amplitude;
        if (ImGui::InputFloat("Amplitude (V)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll)) {
            storage->amplitude = value;
        }
    }
    {
        int value = (int)storage->time_ms;
        if (ImGui::InputInt("Time (ms)", &value, 1, 10, ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll)) {
            storage->time_ms = (uint32_t)value;
        }
    }

    ImGui::Text("Measured resistance (Ohms): %.6f", storage->result);
    ImGui::PopID();
}

void gui_bc_tune::test_l_get_parameters(std::string const& channel, test_l* storage) {
    ImGui::PushID((channel + "l").c_str());
    ImGui::Separator();
    ImGui::Text("Inductance test parameters for channel: %s", channel.c_str());

    {
        float value = storage->bias;
        if (ImGui::InputFloat("Bias (V)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll)) {
            storage->bias = value;
        }
    }
    {
        float value = storage->amplitude;
        if (ImGui::InputFloat("Amplitude (V)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll)) {
            storage->amplitude = value;
        }
    }
    {
        float value = storage->frequency;
        if (ImGui::InputFloat("Frequency (Hz)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll)) {
            storage->frequency = value;
        }
    }
    {
        int value = (int)storage->time_ms;
        if (ImGui::InputInt("Time (ms)", &value, 1, 10, ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll)) {
            storage->time_ms = (uint32_t)value;
        }
    }
    ImGui::Text("Measured inductance (H): %.6f, %.6f (uH)", storage->result, storage->result*1000000.0f);
    ImGui::PopID();
}

int gui_bc_tune::do_test_r(test_r* storage, std::string const& channel) {
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Measure resistance
    std::cout << "Reading " << channel << " resistance\n";
    // Save the original mode
    PARAM_NOTIFY_ERROR( host_->read_enum(target_device_, channel+"_mode", &storage->original_mode), "Parameter failed: mode" )

    // Set parameters first
    PARAM_NOTIFY_ERROR( host_->write_float(target_device_,  channel+"_test_r_v_amplitude", storage->amplitude), "Parameter failed: test_r_v_amplitude" )
    PARAM_NOTIFY_ERROR( host_->write_uint16(target_device_, channel+"_test_r_time_ms", storage->time_ms), "Parameter failed: test_r_time_ms" )
    // Configure controller mode for resistance measurement and start the test
    PARAM_NOTIFY_ERROR( host_->write_enum(target_device_, channel+"_mode", "test_r"), "Parameter failed: mode" )
    PARAM_NOTIFY_ERROR( host_->write_command(target_device_, channel+"_start"), "Parameter failed: start" )
    // Wait for test to finish
    bool running = true;
    do {
        helpers::sleep_ms(100);
        PARAM_NOTIFY_ERROR( host_->read_bool(target_device_, channel+"_is_running", &running), "Parameter failed: is_running" )
    } while (running);

    // Sleep for a bit longer than test time
    helpers::sleep_ms(2000);

    storage->result = 0.0f;
    PARAM_NOTIFY_ERROR( host_->read_float(target_device_, channel+"_R", &storage->result), "Parameter failed: R" )
    std::cout << "Got resistance: " << storage->result << " Ohms\n\n";

    // Restore the original mode
    PARAM_NOTIFY_ERROR( host_->write_enum(target_device_, channel+"_mode", storage->original_mode), "Parameter failed: mode" )

    return jcs::RET_OK;
}

int gui_bc_tune::do_test_l(test_l* storage, std::string const& channel) {
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Measure resistance
    std::cout << "Reading " << channel << " inductance\n";
    // Save the original mode
    PARAM_NOTIFY_ERROR( host_->read_enum(target_device_, channel+"_mode", &storage->original_mode), "Parameter failed: mode" )
    // Set parameters first
    PARAM_NOTIFY_ERROR( host_->write_float(target_device_,  channel+"_test_l_v_bias",      storage->bias),     "Parameter failed: test_ls_v_dq_bias" )
    PARAM_NOTIFY_ERROR( host_->write_float(target_device_,  channel+"_test_l_v_amplitude", storage->amplitude),"Parameter failed: test_ls_v_dq_amplitude" )
    PARAM_NOTIFY_ERROR( host_->write_float(target_device_,  channel+"_test_l_freq_hz",     storage->frequency),"Parameter failed: test_ls_test_freq_hz" )
    PARAM_NOTIFY_ERROR( host_->write_uint16(target_device_, channel+"_test_l_time_ms",     storage->time_ms),  "Parameter failed: test_ls_test_time_ms" )
    // Configure controller mode for inductance measurement and start the test
    PARAM_NOTIFY_ERROR( host_->write_enum(target_device_, channel+"_mode", "test_l"), "Parameter failed: controller_0_mode" )
    PARAM_NOTIFY_ERROR( host_->write_command(target_device_, channel+"_start"), "Parameter failed: controller_0_start" )
    // Note: Here we demo that you don't have to poll the status of the test - you can just wait it out
    // Sleep for a bit longer than test time
    helpers::sleep_ms(storage->time_ms + 2000);

    // Result is stored in motor_Lx
    storage->result = 0.0f;
    PARAM_NOTIFY_ERROR( host_->read_float(target_device_, channel+"_L", &storage->result), "Parameter failed: L" )
    std::cout << "Got inductance: " << storage->result << " H\n";

    // Restore the original mode
    PARAM_NOTIFY_ERROR( host_->write_enum(target_device_, channel+"_mode", storage->original_mode), "Parameter failed: mode" )

    return jcs::RET_OK;
}
