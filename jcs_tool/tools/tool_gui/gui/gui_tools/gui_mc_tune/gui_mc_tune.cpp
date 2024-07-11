// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_mc_tune.h"
#include <iostream>
#include "helpers.h"
#include <cmath>

#include "jcs_dev_motor_controller.h"

//////////////////////////////////////////////////////////////////////
gui_mc_tune::gui_mc_tune(jcs::jcs_host* host, std::string const& target_device) :
    gui_type_base("Tuning", host, target_device)
{
    is_ready_ = false;
    test_step_response_[0] = new test_step_response("measure_test_axis_d");
    test_step_response_[1] = new test_step_response("measure_test_axis_q");
    pole_pairs_ = 7;
    i_ctl_bw_hz_ = 500.0f;
    i_ctl_bw_rads_ = 0.0f;
}

int gui_mc_tune::startup() {
    return jcs::RET_OK;
}

int gui_mc_tune::step_rt() {
    return jcs::RET_OK;
}

int gui_mc_tune::render() {

    // {
    //     int pp_temp = (int)pole_pairs_;
    //     if (ImGui::InputInt("Pole Pairs", &value, 1, 10, ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll)) {
    //         pole_pairs_ = (uint32_t)value;
    //     }
    // }

    ImGui::Text("Measure D/Q axis resistance and inductance");
    ImGui::Text("Notes:");
    ImGui::Text("- Ensure cogging compensation is disabled.");
    ImGui::Text("- Ensure device is not temperature clamped.");

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

    /////////////////////////////////////////////////////////////////////////////////////////////
    // D/Q axis parameters
    // Get D axis parameters
    test_r_get_parameters("measure_test_axis_d", &test_r_parameters_[0]);
    test_l_get_parameters("measure_test_axis_d", &test_l_parameters_[0]);

    // Get Q axis parameters
    test_r_get_parameters("measure_test_axis_q", &test_r_parameters_[1]);
    test_l_get_parameters("measure_test_axis_q", &test_l_parameters_[1]);

    if (ImGui::Button("Measure phase resistance and inductance")) {

        {
            bool ctrl_is_temperature_clamped = false;
            PARAM_NOTIFY_CLEANUP_OK( host_->read_bool(target_device_,  "temperature_penalty_ctrl_is_clamped", &ctrl_is_temperature_clamped), "Parameter failed: temperature_penalty_ctrl_is_clamped", clean_up_on_error(); )

            if (ctrl_is_temperature_clamped == true) {
                std::cout << "ERROR: Device control is temperature clamped. Cannot continue with test.\n";
                clean_up_on_error();
                return jcs::RET_OK;
            }
        }


        CHECK_CLEANUP_OK( do_test_r(&test_r_parameters_[0], "measure_test_axis_d"), clean_up_on_error(); )
        CHECK_CLEANUP_OK( do_test_l(&test_l_parameters_[0], "measure_test_axis_d"), clean_up_on_error(); )

        CHECK_CLEANUP_OK( do_test_r(&test_r_parameters_[1], "measure_test_axis_q"), clean_up_on_error(); )
        CHECK_CLEANUP_OK( do_test_l(&test_l_parameters_[1], "measure_test_axis_q"), clean_up_on_error(); )
    }
    /////////////////////////////////////////////////////////////////////////////////////////////
    // Current controller bandwidth
    ImGui::Separator();
    ImGui::Text("Configure current controller bandwidth");
    ImGui::Text("(Depends on good phase resistance and inductance values)");

    {
        float value = i_ctl_bw_hz_;
        if (ImGui::InputFloat("Current controller bandwidth (Hz)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll)) {
            i_ctl_bw_hz_ = value;
        }
    }
    i_ctl_bw_rads_ = i_ctl_bw_hz_ * 2.0f * (float)M_PI;
    
    ImGui::Text("Current controller bandwidth %.3f (Rad/s)", i_ctl_bw_rads_);
    compute_controller_gains("measure_test_axis_d", &test_r_parameters_[0], &test_l_parameters_[0], &controller_gains_[0]);
    compute_controller_gains("measure_test_axis_q", &test_r_parameters_[1], &test_l_parameters_[1], &controller_gains_[1]);

    if (ImGui::Button("Write current controller gains")) {
        PARAM_NOTIFY_CLEANUP_OK( host_->write_float(target_device_, "i_d_kp", controller_gains_[0].kp), "Parameter failed: i_d_kp", clean_up_on_error(); )
        PARAM_NOTIFY_CLEANUP_OK( host_->write_float(target_device_, "i_d_ki", controller_gains_[0].ki), "Parameter failed: i_d_ki", clean_up_on_error(); )

        PARAM_NOTIFY_CLEANUP_OK( host_->write_float(target_device_, "i_q_kp", controller_gains_[1].kp), "Parameter failed: i_q_kp", clean_up_on_error(); )
        PARAM_NOTIFY_CLEANUP_OK( host_->write_float(target_device_, "i_q_ki", controller_gains_[1].ki), "Parameter failed: i_q_ki", clean_up_on_error(); )
    }
    ImGui::Separator();
    /////////////////////////////////////////////////////////////////////////////////////////////
    // D/Q step response tests
    // Once test_step_response command is in - add a thing here to test the step resonse and display the plot
    step_response_get_parameters(test_step_response_[0]);
    if (ImGui::Button("Start D-axis step response test")) {
        CHECK_CLEANUP_OK( step_response_do_test(test_step_response_[0]), clean_up_on_error(); )
    }
    test_step_response_[0]->plot();

    ImGui::Separator();

    step_response_get_parameters(test_step_response_[1]);
    if (ImGui::Button("Start Q-axis step response test")) {
        CHECK_CLEANUP_OK( step_response_do_test(test_step_response_[1]), clean_up_on_error(); )
    }
    test_step_response_[1]->plot();

    // Cleanup, but don't clear is_ready here
    if (!is_ready_) {
        ImGui::EndDisabled();
    }
    return jcs::RET_OK;
}

void gui_mc_tune::compute_controller_gains(std::string const& axis, test_r* r_store, test_l* l_store, controller_gains* gains) {
    ImGui::PushID(axis.c_str());
    gains->kp = i_ctl_bw_rads_ * l_store->result;
    gains->ki = i_ctl_bw_rads_ * r_store->result;
    ImGui::Text("Current controller axis: %s", axis.c_str());
    {
        float value = gains->kp;
        if (ImGui::InputFloat("Kp", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll)) {
            gains->kp = value;
        }
    }
    {
        float value = gains->ki;
        if (ImGui::InputFloat("Ki", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll)) {
            gains->ki = value;
        }
    }
    ImGui::PopID();
}

void gui_mc_tune::test_r_get_parameters(std::string const& axis, test_r* storage) {
    ImGui::PushID((axis + "r").c_str());
    ImGui::Separator();
    ImGui::Text("Phase resistance test parameters for axis: %s", axis.c_str());

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

void gui_mc_tune::test_l_get_parameters(std::string const& axis, test_l* storage) {
    ImGui::PushID((axis + "l").c_str());
    ImGui::Separator();
    ImGui::Text("Phase inductance test parameters for axis: %s", axis.c_str());

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

int gui_mc_tune::ready_test() {
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Disable the host sentry as we do not plan to enter synchronous mode     
    PARAM_NOTIFY_ERROR( host_->write_bool(target_device_, "host_sentry_active", false), "Parameter failed: host_sentry_active" )
    // Start the motor controller and wait for it to calibrate
    PARAM_NOTIFY_ERROR( host_->write_command(target_device_, "start"), "Parameter failed: start" )
    helpers::sleep_ms(500);
    return jcs::RET_OK;
}

int gui_mc_tune::do_test_r(test_r* storage, std::string const& axis) {
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Measure DQ resistance
    std::cout << "Reading " << axis << " phase resistance\n";
    // Set parameters first
    PARAM_NOTIFY_ERROR( host_->write_enum(target_device_,   "test_rs_dq_test_axis",        axis), "Parameter failed: test_rs_dq_test_axis" )
    PARAM_NOTIFY_ERROR( host_->write_float(target_device_,  "test_rs_v_dq_test_amplitude", storage->amplitude), "Parameter failed: test_rs_v_dq_test_amplitude" )
    PARAM_NOTIFY_ERROR( host_->write_uint16(target_device_, "test_rs_test_time_ms",        storage->time_ms), "Parameter failed: test_rs_test_time_ms" )
    // Configure controller mode for resistance measurement
    PARAM_NOTIFY_ERROR( host_->write_enum(target_device_,   "controller_mode", "test_dq_r"), "Parameter failed: controller_mode" )
    // Starting the controller starts the test
    PARAM_NOTIFY_ERROR( host_->write_command(target_device_, "controller_start"), "Parameter failed: controller_start" )

    // Wait for test to finish
    bool running = true;
    do {
        helpers::sleep_ms(100);
        PARAM_NOTIFY_ERROR( host_->read_bool(target_device_, "controller_is_running", &running), "Parameter failed: controller_is_running" )
    } while (running);
    // Sleep for a bit longer than test time
    helpers::sleep_ms(2000);

    // Result is stored in motor_Rd
    storage->result = 0.0f;
    if (axis == "measure_test_axis_d") {
        PARAM_NOTIFY_ERROR( host_->read_float(target_device_, "motor_Rd", &storage->result), "Res read failed: motor_Rd" )
    } else
    if (axis == "measure_test_axis_q") {
        PARAM_NOTIFY_ERROR( host_->read_float(target_device_, "motor_Rq", &storage->result), "Res read failed: motor_Rq" )
    }
    std::cout << "Got resistance: " << storage->result << " Ohms\n\n";

    return jcs::RET_OK;
}

int gui_mc_tune::do_test_l(test_l* storage, std::string const& axis) {
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Measure DQ resistance
    std::cout << "Reading " << axis << " phase inductance\n";
    // Set parameters first
    PARAM_NOTIFY_ERROR( host_->write_enum(target_device_,   "test_ls_dq_test_axis",   axis), "Parameter failed: test_ls_dq_test_axis" )
    PARAM_NOTIFY_ERROR( host_->write_float(target_device_,  "test_ls_v_dq_bias",      storage->bias),     "Parameter failed: test_ls_v_dq_bias" )
    PARAM_NOTIFY_ERROR( host_->write_float(target_device_,  "test_ls_v_dq_amplitude", storage->amplitude),"Parameter failed: test_ls_v_dq_amplitude" )
    PARAM_NOTIFY_ERROR( host_->write_float(target_device_,  "test_ls_test_freq_hz",   storage->frequency),"Parameter failed: test_ls_test_freq_hz" )
    PARAM_NOTIFY_ERROR( host_->write_uint16(target_device_, "test_ls_test_time_ms",   storage->time_ms),  "Parameter failed: test_ls_test_time_ms" )
    // Configure controller mode for resistance measurement
    PARAM_NOTIFY_ERROR( host_->write_enum(target_device_,   "controller_mode", "test_dq_l"), "Parameter failed: controller_mode" )
    // Starting the controller starts the test
    PARAM_NOTIFY_ERROR( host_->write_command(target_device_, "controller_start"), "Parameter failed: controller_start" )

    // Wait for test to finish
    bool running = true;
    do {
        helpers::sleep_ms(100);
        PARAM_NOTIFY_ERROR( host_->read_bool(target_device_, "controller_is_running", &running), "Parameter failed: controller_is_running" )
    } while (running);

    // Sleep for a bit longer than test time
    helpers::sleep_ms(2000);

    // Result is stored in motor_Lx
    storage->result = 0.0f;
    if (axis == "measure_test_axis_d") {
        PARAM_NOTIFY_ERROR( host_->read_float(target_device_, "motor_Ld", &storage->result), "Res read failed: motor_Ld" )
    } else
    if (axis == "measure_test_axis_q") {
        PARAM_NOTIFY_ERROR( host_->read_float(target_device_, "motor_Lq", &storage->result), "Res read failed: motor_Lq" )
    }
    std::cout << "Got inductance: " << storage->result << " H\n\n";

    return jcs::RET_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
gui_mc_tune::test_step_response::test_step_response(std::string const& axis) :
    axis_(axis), 
    amplitude_(1.0f), 
    time_ms_(1000),
    plot_cursors_(false)
{
    for (int i=0; i<4; i++) {
        cursor_tag_[i] = 0.0;
    }
    data_.resize(jcs::node_parameter::dev_motor_controller::oscilloscope_sample_length);
    x_.resize(jcs::node_parameter::dev_motor_controller::oscilloscope_sample_length);
    // Configure x for time using default sample rate
    for (int i=0; i<x_.size(); i++) {
        x_[i] = (float)i / 30000.0f;
    }
}

void gui_mc_tune::step_response_get_parameters(test_step_response* test) {
    ImGui::PushID((test->axis_).c_str());
    ImGui::Text("Step response test parameters for axis: %s", test->axis_.c_str());

    {
        float value = test->amplitude_;
        if (ImGui::InputFloat("Amplitude (A)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll)) {
            test->amplitude_ = value;
        }
    }
    {
        int value = (int)test->time_ms_;
        if (ImGui::InputInt("Time (ms)", &value, 1, 10, ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll)) {
            test->time_ms_ = (uint32_t)value;
        }
    }
    ImGui::PopID();
}

int gui_mc_tune::step_response_do_test(test_step_response* test) {
    ImGui::PushID((test->axis_).c_str());

    auto clean_up = []() {
        ImGui::PopID();
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Do DQ axis step response test
    // Set parameters first
    PARAM_NOTIFY_CLEANUP_ERROR( host_->write_enum(target_device_,   "test_step_response_dq_test_axis", test->axis_), "Parameter failed: test_rs_dq_test_axis", clean_up(); )
    PARAM_NOTIFY_CLEANUP_ERROR( host_->write_float(target_device_,  "test_step_response_amplitude",    test->amplitude_), "Parameter failed: test_rs_v_dq_test_amplitude", clean_up(); )
    PARAM_NOTIFY_CLEANUP_ERROR( host_->write_uint16(target_device_, "test_step_response_time_ms",      test->time_ms_), "Parameter failed: test_rs_test_time_ms", clean_up(); )
    // Configure controller mode for the test
    PARAM_NOTIFY_CLEANUP_ERROR( host_->write_enum(target_device_,   "controller_mode", "test_dq_step_response"), "Parameter failed: controller_mode", clean_up(); )
    // Configure the oscilloscope
    // Set the trigger
    if (test->axis_ == "measure_test_axis_d") {
        PARAM_NOTIFY_CLEANUP_ERROR( host_->write_enum(target_device_,  "oscilloscope_trigger_source",   "mc_osc_foc_i_d_cmd"), "Parameter failed: oscilloscope_trigger_source", clean_up(); )
        PARAM_NOTIFY_CLEANUP_ERROR( host_->write_float(target_device_, "oscilloscope_trigger_level",    0.01f),  "Parameter failed: oscilloscope_trigger_level", clean_up(); )
        PARAM_NOTIFY_CLEANUP_ERROR( host_->write_enum(target_device_,  "oscilloscope_channel_0_source", "mc_osc_foc_i_d"), "Parameter failed: oscilloscope_channel_0_source", clean_up(); )
    } else
    if (test->axis_ == "measure_test_axis_q") {
        PARAM_NOTIFY_CLEANUP_ERROR( host_->write_enum(target_device_,  "oscilloscope_trigger_source",   "mc_osc_foc_i_q_cmd"), "Parameter failed: oscilloscope_trigger_source", clean_up(); )
        PARAM_NOTIFY_CLEANUP_ERROR( host_->write_float(target_device_, "oscilloscope_trigger_level",    0.01f),  "Parameter failed: oscilloscope_trigger_level", clean_up(); )
        PARAM_NOTIFY_CLEANUP_ERROR( host_->write_enum(target_device_,  "oscilloscope_channel_0_source", "mc_osc_foc_i_q"), "Parameter failed: oscilloscope_channel_0_source", clean_up(); )
    }
    else {
        clean_up();
        return jcs::RET_ERROR;
    }
    // Set oscilloscope to wait
    PARAM_NOTIFY_CLEANUP_ERROR( host_->write_command(target_device_, "oscilloscope_wait_trigger"), "Parameter failed: oscilloscope_wait_trigger", clean_up(); )
    // Starting the controller starts the test
    PARAM_NOTIFY_CLEANUP_ERROR( host_->write_command(target_device_, "controller_start"), "Parameter failed: controller_start", clean_up(); )

    // Sleep for a bit longer than test time
    helpers::sleep_ms(test->time_ms_ + 2000);
    // Get oscilloscope data and plot
    PARAM_NOTIFY_CLEANUP_ERROR( host_->read_float(target_device_, "oscilloscope_channel_0",  &test->data_), "Parameter failed: oscilloscope_channel_0", clean_up(); )

    clean_up();
    return jcs::RET_OK;
}

void gui_mc_tune::test_step_response::plot() {
    ImGui::PushID(axis_.c_str());

    ImGui::Checkbox("Show measurement cursors", &plot_cursors_);

    if (ImPlot::BeginPlot(axis_.c_str())) {
        ImPlot::SetupAxes("t", "y");

        if (plot_cursors_) {
            // X0
            ImPlot::DragLineX(0, &cursor_tag_[0], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
            ImPlot::TagX(cursor_tag_[0], ImVec4(1,0,0,1), "%.3f", cursor_tag_[0]);
            // X1
            ImPlot::DragLineX(1, &cursor_tag_[1], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
            ImPlot::TagX(cursor_tag_[1], ImVec4(1,0,0,1), "%.3f", cursor_tag_[1]);
            // Y0
            ImPlot::DragLineY(2, &cursor_tag_[2], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
            ImPlot::TagY(cursor_tag_[2], ImVec4(1,0,0,1), "%.3f", cursor_tag_[2]);
            // Y1
            ImPlot::DragLineY(3, &cursor_tag_[3], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
            ImPlot::TagY(cursor_tag_[3], ImVec4(1,0,0,1), "%.3f", cursor_tag_[3]);
        }

        ImPlot::PlotLine(axis_.c_str(), &x_[0], &data_[0], x_.size());
        ImPlot::EndPlot();
    }

    if (plot_cursors_) {
        static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | 
                                             ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings;

        if (ImGui::BeginTable("Measurements", 2, table_flags)) {
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("delta T");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.6f", cursor_tag_[0] - cursor_tag_[1]);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("delta Y");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.6f", cursor_tag_[2] - cursor_tag_[3]);
            ImGui::EndTable();
        }
    }
    ImGui::PopID();
}