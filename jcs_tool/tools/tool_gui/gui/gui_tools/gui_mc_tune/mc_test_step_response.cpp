// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "mc_test_step_response.h"
#include <iostream>
#include "helpers.h"
#include "imgui.h"
#include "jcs_dev_motor_controller.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// Static data
const std::vector<std::string> mc_test_step_response::axis_names       = { "d", "q" };
const std::vector<std::string> mc_test_step_response::axis_enum_values = { "measure_test_axis_d", "measure_test_axis_q" };
const std::vector<std::string> mc_test_step_response::mode_names       = { "Current", "Torque" };
const std::vector<std::string> mc_test_step_response::mode_enum_values = { "test_dq_step_response_current", "test_dq_step_response_torque" };

/////////////////////////////////////////////////////////////////////////////////////////////
mc_test_step_response::mc_test_step_response()
    : amplitude(1.0f),
      time_ms(1000),
      axis_index(0),
      mode_index(0),
      plot("Step Response", "t (s)", "y",
           jcs::node_parameter::dev_motor_controller::oscilloscope_sample_length)
{
    plot.update_sample_rate(30000);
    plot.add_channel("response", ImVec4(0.0f, 1.0f, 0.4f, 1.0f));
}

void mc_test_step_response::render_ui() {
    ImGui::PushID("test_step");
    ImGui::Text("Step response test parameters");
    ImGui::Text("Notes:");
    ImGui::Text("- Only Q-axis test available when in torque mode.");
    ImGui::Text("- Current mode internally sets Kt=1 for the duration of the test. (Restored when test completes)");

    // Axis dropdown
    {
        int idx = axis_index;
        helpers::combo_select("Test axis", &axis_names, &idx, nullptr);
        axis_index = idx;
    }
    // Mode dropdown
    {
        int idx = mode_index;
        helpers::combo_select("Test mode", &mode_names, &idx, nullptr);
        mode_index = idx;
    }

    {
        float value = amplitude;
        if (ImGui::InputFloat("Amplitude", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            amplitude = value;
        }
    }
    {
        int value = (int)time_ms;
        if (ImGui::InputInt("Time (ms)", &value, 1, 10, ImGuiInputTextFlags_EscapeClearsAll)) {
            time_ms = (uint32_t)value;
        }
    }
    ImGui::PopID();
}

void mc_test_step_response::render_plot() {
    plot.plot();
}

int mc_test_step_response::execute(jcs::jcs_host* host, std::string const& target) {
    std::string const& axis_enum = axis_enum_values[axis_index];
    std::string const& mode_enum = mode_enum_values[mode_index];
    std::string const& axis      = axis_names[axis_index];

    std::cout << "Running step response test: axis=" << axis << " mode=" << mode_names[mode_index] << "\n";

    // Set test parameters
    PARAM_NOTIFY_ERROR( host->write_enum(target,   "test_step_response_dq_test_axis", axis_enum),  "Parameter failed: test_step_response_dq_test_axis" )
    PARAM_NOTIFY_ERROR( host->write_enum(target,   "test_step_response_mode",         mode_enum),  "Parameter failed: test_step_response_mode" )
    PARAM_NOTIFY_ERROR( host->write_float(target,  "test_step_response_amplitude",    amplitude),  "Parameter failed: test_step_response_amplitude" )
    PARAM_NOTIFY_ERROR( host->write_uint16(target, "test_step_response_time_ms",      time_ms),    "Parameter failed: test_step_response_time_ms" )

    PARAM_NOTIFY_ERROR( host->write_enum(target,   "controller_mode", "test_dq_step_response"), "Parameter failed: controller_mode" )

    // Configure oscilloscope
    PARAM_NOTIFY_ERROR( host->write_enum(target,   "oscilloscope_trigger_config", "osc_trigger_rising_edge"), "Parameter failed: oscilloscope_trigger_config" )
    PARAM_NOTIFY_ERROR( host->write_uint32(target, "oscilloscope_sample_rate_hz", 30000), "Parameter failed: oscilloscope_sample_rate_hz" )
    PARAM_NOTIFY_ERROR( host->write_float(target,  "oscilloscope_trigger_level", 0.01f),  "Parameter failed: oscilloscope_trigger_level" )
    PARAM_NOTIFY_ERROR( host->write_uint32(target, "oscilloscope_trigger_buffer_position", 0), "Parameter failed: oscilloscope_trigger_buffer_position" )

    if (axis == "d") {
        PARAM_NOTIFY_ERROR( host->write_enum(target, "oscilloscope_trigger_source",   "mc_osc_foc_i_d_cmd"), "Parameter failed: oscilloscope_trigger_source" )
        PARAM_NOTIFY_ERROR( host->write_enum(target, "oscilloscope_channel_0_source", "mc_osc_foc_i_d"),     "Parameter failed: oscilloscope_channel_0_source" )
    } else if (axis == "q") {
        PARAM_NOTIFY_ERROR( host->write_enum(target, "oscilloscope_trigger_source",   "mc_osc_foc_i_q_cmd"), "Parameter failed: oscilloscope_trigger_source" )
        PARAM_NOTIFY_ERROR( host->write_enum(target, "oscilloscope_channel_0_source", "mc_osc_foc_i_q"),     "Parameter failed: oscilloscope_channel_0_source" )
    } else {
        return jcs::RET_ERROR;
    }

    PARAM_NOTIFY_ERROR( host->write_command(target, "oscilloscope_wait_trigger"), "Parameter failed: oscilloscope_wait_trigger" )
    PARAM_NOTIFY_ERROR( host->write_command(target, "controller_start"),          "Parameter failed: controller_start" )

    helpers::sleep_ms(time_ms + 2000);

    // Read oscilloscope data into the plot channel
    plot_measurement_multi::channel* ch = plot.get_channel("response");
    if (ch == nullptr) {
        std::cout << "ERROR: plot channel 'response' not found\n";
        return jcs::RET_ERROR;
    }

    // Read as float vector then copy to double channel
    std::vector<float> data;
    data.resize(jcs::node_parameter::dev_motor_controller::oscilloscope_sample_length);
    PARAM_NOTIFY_ERROR( host->read_float(target, "oscilloscope_channel_0", &data), "Parameter failed: oscilloscope_channel_0" )

    for (int i = 0; i < (int)data.size() && i < (int)ch->y_.size(); i++) {
        ch->y_[i] = (double)data[i];
    }

    return jcs::RET_OK;
}