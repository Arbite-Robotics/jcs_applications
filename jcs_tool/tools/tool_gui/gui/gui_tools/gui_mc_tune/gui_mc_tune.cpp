// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_mc_tune.h"
#include <iostream>
#include "helpers.h"
#include <cmath>

#include "jcs_dev_motor_controller.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// test_resistance
test_resistance::test_resistance()
    : amplitude(1.0f), time_ms(3000), ramp_ms(500), result(0.0f)
{}

void test_resistance::render_ui() {
    ImGui::PushID("test_r");
    ImGui::Separator();
    ImGui::Text("Phase resistance test parameters");

    {
        float value = amplitude;
        if (ImGui::InputFloat("Amplitude (V)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            amplitude = value;
        }
    }
    {
        int value = (int)time_ms;
        if (ImGui::InputInt("Time (ms)", &value, 1, 10, ImGuiInputTextFlags_EscapeClearsAll)) {
            time_ms = (uint32_t)value;
        }
    }
    {
        int value = (int)ramp_ms;
        if (ImGui::InputInt("Ramp time (ms)", &value, 1, 10, ImGuiInputTextFlags_EscapeClearsAll)) {
            ramp_ms = (uint32_t)value;
        }
    }

    ImGui::Text("Measured resistance (Ohms): %.6f", result);
    helpers::result_text_copyable("motor_Rs: ", result);
    ImGui::PopID();
}

int test_resistance::execute(jcs::jcs_host* host, std::string const& target) {
    std::cout << "Reading synchronous resistance\n";

    PARAM_NOTIFY_ERROR( host->write_float(target,  "test_rs_v_dq_test_amplitude", amplitude), "Parameter failed: test_rs_v_dq_test_amplitude" )
    PARAM_NOTIFY_ERROR( host->write_uint16(target, "test_rs_test_time_ms",        time_ms),   "Parameter failed: test_rs_test_time_ms" )
    PARAM_NOTIFY_ERROR( host->write_uint16(target, "test_rs_ramp_time_ms",        ramp_ms),   "Parameter failed: test_rs_ramp_time_ms" )

    PARAM_NOTIFY_ERROR( host->write_enum(target,   "controller_mode", "test_rs"), "Parameter failed: controller_mode" )
    PARAM_NOTIFY_ERROR( host->write_command(target, "controller_start"),           "Parameter failed: controller_start" )

    // Check for error
    bool test_error = true;
    helpers::sleep_ms(100);
    PARAM_NOTIFY_ERROR( host->read_bool(target, "controller_is_error", &test_error), "Parameter failed: controller_is_error" )
    if (test_error) {
        std::cout << "ERROR: controller_start failed to start the test. Check that motor controller is ready to go.\n";
        return jcs::RET_ERROR;
    }

    // Wait for completion
    bool running = true;
    do {
        helpers::sleep_ms(100);
        PARAM_NOTIFY_ERROR( host->read_bool(target, "controller_is_running", &running), "Parameter failed: controller_is_running" )
    } while (running);
    helpers::sleep_ms(2000);

    // Read result
    result = 0.0f;
    PARAM_NOTIFY_ERROR( host->read_float(target, "motor_Rs", &result), "Res read failed: motor_Rs" )
    std::cout << "Got resistance: " << result << " Ohms\n\n";

    return jcs::RET_OK;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// test_inductance
test_inductance::test_inductance(std::string const& axis)
    : axis(axis), bias(0.0f), amplitude(1.0f), frequency(1000.0f),
      time_ms(3000), ramp_ms(500), settle_ms(500),
      result(0.0f)
{}

void test_inductance::render_ui() {
    ImGui::PushID((axis + "l").c_str());
    ImGui::Separator();
    ImGui::Text("Phase inductance test parameters for %s-axis", axis.c_str());

    {
        float value = bias;
        if (ImGui::InputFloat("Bias (V)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            bias = value;
        }
    }
    {
        float value = amplitude;
        if (ImGui::InputFloat("Amplitude (V)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            amplitude = value;
        }
    }
    {
        float value = frequency;
        if (ImGui::InputFloat("Frequency (Hz)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            frequency = value;
        }
    }
    {
        int value = (int)time_ms;
        if (ImGui::InputInt("Time (ms)", &value, 1, 10, ImGuiInputTextFlags_EscapeClearsAll)) {
            time_ms = (uint32_t)value;
        }
    }
    {
        int value = (int)ramp_ms;
        if (ImGui::InputInt("Ramp time (ms)", &value, 1, 10, ImGuiInputTextFlags_EscapeClearsAll)) {
            ramp_ms = (uint32_t)value;
        }
    }
    {
        int value = (int)settle_ms;
        if (ImGui::InputInt("Settle time (ms)", &value, 1, 10, ImGuiInputTextFlags_EscapeClearsAll)) {
            settle_ms = (uint32_t)value;
        }
    }

    ImGui::Text("Measured inductance (H): %.6f, %.6f (uH)", result, result * 1000000.0f);
    helpers::result_text_copyable("motor_L" + axis + ": ", result);
    ImGui::PopID();
}

int test_inductance::execute(jcs::jcs_host* host, std::string const& target) {
    std::cout << "Reading " << axis << " axis phase inductance\n";

    std::string meas_axis = axis == "d" ? "measure_test_axis_d" : "measure_test_axis_q";
    PARAM_NOTIFY_ERROR( host->write_enum(target,   "test_ls_dq_test_axis",   meas_axis),  "Parameter failed: test_ls_dq_test_axis" )
    PARAM_NOTIFY_ERROR( host->write_float(target,  "test_ls_v_dq_bias",      bias),       "Parameter failed: test_ls_v_dq_bias" )
    PARAM_NOTIFY_ERROR( host->write_float(target,  "test_ls_v_dq_amplitude", amplitude),  "Parameter failed: test_ls_v_dq_amplitude" )
    PARAM_NOTIFY_ERROR( host->write_float(target,  "test_ls_test_freq_hz",   frequency),  "Parameter failed: test_ls_test_freq_hz" )
    PARAM_NOTIFY_ERROR( host->write_uint16(target, "test_ls_test_time_ms",   time_ms),    "Parameter failed: test_ls_test_time_ms" )
    PARAM_NOTIFY_ERROR( host->write_uint16(target, "test_ls_ramp_time_ms",   ramp_ms),    "Parameter failed: test_ls_ramp_time_ms" )
    PARAM_NOTIFY_ERROR( host->write_uint16(target, "test_ls_settle_time_ms", settle_ms),  "Parameter failed: test_ls_settle_time_ms" )

    PARAM_NOTIFY_ERROR( host->write_enum(target,   "controller_mode", "test_dq_l"), "Parameter failed: controller_mode" )
    PARAM_NOTIFY_ERROR( host->write_command(target, "controller_start"),            "Parameter failed: controller_start" )

    // Check for error
    bool test_error = true;
    helpers::sleep_ms(100);
    PARAM_NOTIFY_ERROR( host->read_bool(target, "controller_is_error", &test_error), "Parameter failed: controller_is_error" )
    if (test_error) {
        std::cout << "ERROR: controller_start failed to start the test. Check that motor controller is ready to go.\n";
        return jcs::RET_ERROR;
    }

    // Wait for completion
    bool running = true;
    do {
        helpers::sleep_ms(100);
        PARAM_NOTIFY_ERROR( host->read_bool(target, "controller_is_running", &running), "Parameter failed: controller_is_running" )
    } while (running);
    helpers::sleep_ms(2000);

    // Read result
    result = 0.0f;
    if (axis == "d") {
        PARAM_NOTIFY_ERROR( host->read_float(target, "motor_Ld", &result), "Res read failed: motor_Ld" )
    } else if (axis == "q") {
        PARAM_NOTIFY_ERROR( host->read_float(target, "motor_Lq", &result), "Res read failed: motor_Lq" )
    }
    std::cout << "Got inductance: " << result << " H\n\n";

    return jcs::RET_OK;
}

void test_inductance::copy_result_from(test_inductance const& other) {
    result = other.result;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Controller gains
controller_gains compute_controller_gains(float bw_rads, float resistance, float inductance) {
    controller_gains g;
    g.kp = bw_rads * inductance;
    g.ki = bw_rads * resistance;
    return g;
}

void render_controller_gains(std::string const& axis, controller_gains const& gains) {
    ImGui::PushID(axis.c_str());
    ImGui::Text("Current controller axis: %s", axis.c_str());
    helpers::result_text_copyable("i_" + axis + "_kp: ", gains.kp);
    helpers::result_text_copyable("i_" + axis + "_ki: ", gains.ki);
    ImGui::PopID();
}

/////////////////////////////////////////////////////////////////////////////////////////////
// test_step_response
test_step_response::test_step_response(std::string const& axis)
    : axis(axis), amplitude(1.0f), time_ms(1000), plot_cursors(false)
{
    for (int i = 0; i < 4; i++) {
        cursor_tag[i] = 0.0;
    }
    data.resize(jcs::node_parameter::dev_motor_controller::oscilloscope_sample_length);
    x.resize(jcs::node_parameter::dev_motor_controller::oscilloscope_sample_length);
    for (int i = 0; i < (int)x.size(); i++) {
        x[i] = (float)i / 30000.0f;
    }
}

void test_step_response::render_ui() {
    ImGui::PushID(axis.c_str());
    ImGui::Text("Step response test parameters for axis: %s", axis.c_str());

    {
        float value = amplitude;
        if (ImGui::InputFloat("Amplitude (A)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
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

void test_step_response::render_plot() {
    ImGui::PushID(axis.c_str());

    ImGui::Checkbox("Show measurement cursors", &plot_cursors);

    if (ImPlot::BeginPlot(("Axis: " + axis).c_str())) {
        ImPlot::SetupAxes("t", "y");

        if (plot_cursors) {
            ImPlot::DragLineX(0, &cursor_tag[0], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
            ImPlot::TagX(cursor_tag[0], ImVec4(1,0,0,1), "%.3f", cursor_tag[0]);
            ImPlot::DragLineX(1, &cursor_tag[1], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
            ImPlot::TagX(cursor_tag[1], ImVec4(1,0,0,1), "%.3f", cursor_tag[1]);
            ImPlot::DragLineY(2, &cursor_tag[2], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
            ImPlot::TagY(cursor_tag[2], ImVec4(1,0,0,1), "%.3f", cursor_tag[2]);
            ImPlot::DragLineY(3, &cursor_tag[3], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
            ImPlot::TagY(cursor_tag[3], ImVec4(1,0,0,1), "%.3f", cursor_tag[3]);
        }

        ImPlot::PlotLine(axis.c_str(), &x[0], &data[0], x.size());
        ImPlot::EndPlot();
    }

    if (plot_cursors) {
        static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders |
                                             ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings;

        if (ImGui::BeginTable("Measurements", 2, table_flags)) {
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("delta T");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.6f", cursor_tag[0] - cursor_tag[1]);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("delta Y");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.6f", cursor_tag[2] - cursor_tag[3]);
            ImGui::EndTable();
        }
    }
    ImGui::PopID();
}

int test_step_response::execute(jcs::jcs_host* host, std::string const& target) {
    float motor_Kt = 1.0f;

    // Read and normalise motor_Kt for the test
    PARAM_NOTIFY_ERROR( host->read_float(target, "motor_Kt", &motor_Kt), "Parameter failed: motor_Kt" )
    if (motor_Kt != 1.0f) {
        PARAM_NOTIFY_ERROR( host->write_float(target, "motor_Kt", 1.0f), "Parameter failed: motor_Kt" )
    }

    auto restore_kt = [&]() {
        if (motor_Kt != 1.0f) {
            PARAM_NOTIFY( host->write_float(target, "motor_Kt", motor_Kt), "Parameter failed: motor_Kt" )
        }
    };

    // Set test parameters
    std::string meas_axis = axis == "d" ? "measure_test_axis_d" : "measure_test_axis_q";
    PARAM_NOTIFY_ERROR( host->write_enum(target,   "test_step_response_dq_test_axis", meas_axis),  "Parameter failed: test_rs_dq_test_axis" )
    PARAM_NOTIFY_ERROR( host->write_float(target,  "test_step_response_amplitude",    amplitude),  "Parameter failed: test_rs_v_dq_test_amplitude" )
    PARAM_NOTIFY_ERROR( host->write_uint16(target, "test_step_response_time_ms",      time_ms),    "Parameter failed: test_rs_test_time_ms" )

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
        restore_kt();
        return jcs::RET_ERROR;
    }

    PARAM_NOTIFY_ERROR( host->write_command(target, "oscilloscope_wait_trigger"), "Parameter failed: oscilloscope_wait_trigger" )
    PARAM_NOTIFY_ERROR( host->write_command(target, "controller_start"),          "Parameter failed: controller_start" )

    helpers::sleep_ms(time_ms + 2000);

    // NOTE: standby/ready workaround for large parameter timeout bug
    PARAM_NOTIFY_ERROR( host->write_command(target, "stop"), "Parameter failed: stop" )
    helpers::sleep_ms(500);

    PARAM_NOTIFY_ERROR( host->read_float(target, "oscilloscope_channel_0", &data), "Parameter failed: oscilloscope_channel_0" )

    PARAM_NOTIFY_ERROR( host->write_command(target, "start"), "Parameter failed: start" )
    helpers::sleep_ms(500);

    restore_kt();
    return jcs::RET_OK;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// gui_mc_tune
gui_mc_tune::gui_mc_tune(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) :
    gui_type_base("Tuning", host, gui_if, target_device),
    is_ready_(false),
    pole_pairs_(7),
    test_l_{ test_inductance("d"), test_inductance("q") },
    lq_equals_ld_(true),
    i_ctl_bw_hz_(500.0f),
    i_ctl_bw_rads_(0.0f),
    test_step_{ test_step_response("d"), test_step_response("q") }
{}

int gui_mc_tune::startup() {
    return jcs::RET_OK;
}

int gui_mc_tune::step_rt() {
    return jcs::RET_OK;
}

int gui_mc_tune::render() {
    ImGui::Text("Tuning and measure D/Q axis resistance and inductance");
    ImGui::Separator();
    ImGui::Text("Notes:");
    ImGui::Text("- Ensure cogging compensation is disabled.");
    ImGui::Text("- Ensure device is not temperature clamped.");

    ImGui::Separator();
    if (ImGui::Button("Click to make motor controller ready for tests!")) {
        if (ready_test() != jcs::RET_OK) {
            return jcs::RET_OK;
        }
    }

    if (!is_ready_) {
        ImGui::BeginDisabled();
    }

    auto clean_up_on_error = [&]() {
        if (!is_ready_) {
            ImGui::EndDisabled();
        }
        is_ready_ = false;
    };

    /////////////////////////////////////////////////////////////////////////////////////////////
    // Resistance and Ldq parameters
    test_r_.render_ui();
    test_l_[0].render_ui();

    ImGui::Separator();
    ImGui::Text("For surface-mount magnet motors Ld=Lq (most outrunner and gimbal motors). Set to skip Q-axis test.");
    ImGui::Text("For interior magnet motors Lq>Ld. Disable to perform Q-axis test.");
    ImGui::Checkbox("Lq = Ld", &lq_equals_ld_);

    if (lq_equals_ld_) {
        test_l_[1].copy_result_from(test_l_[0]);
        ImGui::BeginDisabled();
        test_l_[1].render_ui();
        ImGui::EndDisabled();
    } else {
        test_l_[1].render_ui();
    }

    if (ImGui::Button("Measure phase resistance and inductance")) {
        // Temperature clamp check
        {
            bool ctrl_is_temperature_clamped = false;
            PARAM_NOTIFY_CLEANUP_OK( host_->read_bool(target_device_, "temperature_penalty_ctrl_is_clamped", &ctrl_is_temperature_clamped), "Parameter failed: temperature_penalty_ctrl_is_clamped", clean_up_on_error(); )

            if (ctrl_is_temperature_clamped) {
                std::cout << "ERROR: Device control is temperature clamped. Cannot continue with test.\n";
                clean_up_on_error();
                return jcs::RET_OK;
            }
        }
        // Disable measurement filter for higher frequency tests
        std::string i_dq_measurement_filter = "";
        {
            PARAM_NOTIFY_CLEANUP_OK( host_->read_enum(target_device_, "i_dq_measurement_filter", &i_dq_measurement_filter), "Parameter failed: i_dq_measurement_filter", clean_up_on_error(); )
            PARAM_NOTIFY_CLEANUP_OK( host_->write_enum(target_device_, "i_dq_measurement_filter", "dq_filter_none"), "Parameter failed: i_dq_measurement_filter", clean_up_on_error(); )
        }

        // Run resistance test
        CHECK_CLEANUP_OK( test_r_.execute(host_, target_device_), clean_up_on_error(); )

        // Run Ld test
        CHECK_CLEANUP_OK( test_l_[0].execute(host_, target_device_), clean_up_on_error(); )

        // Run Lq test (or copy from Ld)
        if (lq_equals_ld_) {
            test_l_[1].copy_result_from(test_l_[0]);
            // Write Lq out
            PARAM_NOTIFY_CLEANUP_OK( host_->write_float(target_device_, "motor_Lq", test_l_[1].result), "Parameter failed: motor_Lq", clean_up_on_error(); )
        } else {
            CHECK_CLEANUP_OK( test_l_[1].execute(host_, target_device_), clean_up_on_error(); )
        }

        // Restore original dq filter
        PARAM_NOTIFY_CLEANUP_OK( host_->write_enum(target_device_, "i_dq_measurement_filter", i_dq_measurement_filter), "Parameter failed: i_dq_measurement_filter", clean_up_on_error(); )
    }

    /////////////////////////////////////////////////////////////////////////////////////////////
    // Current controller bandwidth
    ImGui::NewLine();
    ImGui::Separator();
    ImGui::Text("Configure current controller bandwidth");
    ImGui::Text("(Depends on good phase resistance and inductance values)");
    {
        float value = i_ctl_bw_hz_;
        if (ImGui::InputFloat("Current controller bandwidth (Hz)", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            i_ctl_bw_hz_ = value;
        }
    }
    i_ctl_bw_rads_ = i_ctl_bw_hz_ * 2.0f * (float)M_PI;
    ImGui::Text("Current controller bandwidth %.3f (Rad/s)", i_ctl_bw_rads_);

    controller_gains_[0] = compute_controller_gains(i_ctl_bw_rads_, test_r_.result, test_l_[0].result);
    controller_gains_[1] = compute_controller_gains(i_ctl_bw_rads_, test_r_.result, test_l_[1].result);

    render_controller_gains("d", controller_gains_[0]);
    render_controller_gains("q", controller_gains_[1]);

    if (ImGui::Button("Write current controller gains")) {
        PARAM_NOTIFY_CLEANUP_OK( host_->write_float(target_device_, "i_d_kp", controller_gains_[0].kp), "Parameter failed: i_d_kp", clean_up_on_error(); )
        PARAM_NOTIFY_CLEANUP_OK( host_->write_float(target_device_, "i_d_ki", controller_gains_[0].ki), "Parameter failed: i_d_ki", clean_up_on_error(); )
        PARAM_NOTIFY_CLEANUP_OK( host_->write_float(target_device_, "i_q_kp", controller_gains_[1].kp), "Parameter failed: i_q_kp", clean_up_on_error(); )
        PARAM_NOTIFY_CLEANUP_OK( host_->write_float(target_device_, "i_q_ki", controller_gains_[1].ki), "Parameter failed: i_q_ki", clean_up_on_error(); )
    }

    /////////////////////////////////////////////////////////////////////////////////////////////
    // D/Q step response tests
    ImGui::Separator();
    ImGui::NewLine();

    test_step_[0].render_ui();
    if (ImGui::Button("Start D-axis step response test")) {
        CHECK_CLEANUP_OK( test_step_[0].execute(host_, target_device_), clean_up_on_error(); )
    }
    test_step_[0].render_plot();

    ImGui::Separator();

    test_step_[1].render_ui();
    if (ImGui::Button("Start Q-axis step response test")) {
        CHECK_CLEANUP_OK( test_step_[1].execute(host_, target_device_), clean_up_on_error(); )
    }
    test_step_[1].render_plot();

    // End disabled state if needed (but don't clear is_ready_)
    if (!is_ready_) {
        ImGui::EndDisabled();
    }
    return jcs::RET_OK;
}

int gui_mc_tune::ready_test() {
    PARAM_NOTIFY_ERROR( host_->write_bool(target_device_, "host_sentry_active", false), "Parameter failed: host_sentry_active" )
    PARAM_NOTIFY_ERROR( host_->write_command(target_device_, "start"), "Parameter failed: start" )
    is_ready_ = true;
    helpers::sleep_ms(500);
    return jcs::RET_OK;
}

int gui_mc_tune::standby_test() {
    PARAM_NOTIFY_ERROR( host_->write_command(target_device_, "stop"), "Parameter failed: stop" )
    is_ready_ = false;
    helpers::sleep_ms(500);
    return jcs::RET_OK;
}
