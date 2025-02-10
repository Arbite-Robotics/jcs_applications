// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_mc_cogging.h"
#include <iostream>
#include "jcs_user_external.h"
#include "helpers.h"
#include <cmath>
#include <numeric>
#include "ImGuiFileDialog.h"

#include "jcs_dev_motor_controller.h"

//////////////////////////////////////////////////////////////////////
gui_mc_cogging::gui_mc_cogging(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) :
    gui_type_base("Cogging Compensation", host, gui_if, target_device),
    plot_result_("Cogging raw", "th_m_0", "i_q", rotation_steps_),
    plot_final_("Cogging final", "th_m_0", "i_q", rotation_steps_),
    plot_vis_("Cogging visualisation", "th_m_0", "i_q", rotation_steps_)
{
    initialise();
    cogging_compensator_upper_w_m_low_ = 100.0f;
    cogging_compensator_upper_w_m_high_ = 200.0f;
    compensated_at_zero_pos_ = 0.0f;

    fb_th_m_0_idx_= 0;
    fb_w_m_0_idx_ = 1;
    fb_i_q_idx_ = 2;
    cmd_th_m_0_idx_ = 0;

    required_input_signal_names_ = { "HOST::th_m" };
    required_output_signal_names_ = { target_device_+"::th_m_0",
                                      target_device_+"::w_m_0",
                                      target_device_+"::i_q" };
    can_start_ = true;
}

int gui_mc_cogging::startup() {
    // Get signal sizes
    unsigned int host_sigs_out_sz = host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0);
    unsigned int host_sigs_in_sz  = host_->sig_input_sz_unsafe_rt(jcs::signal_type::float32_s, 0);

    // Per tick signal storage
    signals_out_.resize(host_sigs_out_sz);
    signals_in_.resize(host_sigs_in_sz);

    return jcs::RET_OK;
}

int gui_mc_cogging::step_rt() {

    host_->sig_input_set_rt(0, signals_in_);
    host_->sig_output_get_rt(0, &signals_out_);

    switch (state_) {
        default:
        case behaviour::standby_s: 
        case behaviour::finish_s: 
            break;

        case behaviour::wait_position_s:
            // Compute the error and normalise to [-pi, pi]
            theta_error_ = theta_command_ - signals_out_[fb_th_m_0_idx_];
            theta_error_ = helpers::angle_norm_pipi(theta_error_);

            // Wait until position is within threashold, with ideally 0 velocity
            if (fabsf(theta_error_) < theta_threshold_ && fabsf(signals_out_[fb_w_m_0_idx_]) < omega_threshold_) {
                // Store th_m_0 and i_q directly into plot storage
                plot_result_.x_[rotation_tick_] = signals_out_[fb_th_m_0_idx_];
                plot_result_.y_[rotation_tick_] = signals_out_[fb_i_q_idx_];

                // Rotate to next position
                state_ = behaviour::rotate_s;
            }
            // Set new rotation
            signals_in_[cmd_th_m_0_idx_] = theta_command_;
            break;

        case behaviour::rotate_s:
            // Increment new position
            theta_command_ = helpers::angle_norm_pipi(((float)rotation_tick_ * 2.0f * (float)M_PI) / (float)rotation_steps_);
            rotation_tick_++;
            if (rotation_tick_ > rotation_steps_) {
                // DONE
                // state_ = behaviour::standby_s;
                state_ = behaviour::finish_s;
                break;
            }
            // Go wait for new position
            state_ = behaviour::wait_position_s;
            break;
    }

    return jcs::RET_OK;
}

int gui_mc_cogging::render() {

    ImGui::Text("Cogging compensator coefficients tool");
    ImGui::Separator();
    ImGui::Text("Notes:");
    ImGui::Text("- Ensure correct configuration is used.");
    ImGui::Text("- Ensure motor configuration has parameter estimator_0_theta_passthrough set to yes.");
    ImGui::Text("- Ensure device is not temperature clamped.");
    ImGui::Text("- Ensure motor can spin freely.");
    ImGui::Separator();

    ImGui::Text("Tool expects the following:");
    ImGui::Text("- JCS output signal motor position: `th_m_0`");
    ImGui::Text("- JCS output signal motor velocity: `w_m_0`");
    ImGui::Text("- JCS output signal motor current:  `i_q`");
    ImGui::Text("- JCS input signal motor commanded position: `th_m`");

    ImGui::Separator();
    if (helpers::input_signals_check(gui_if_->get_f32_input_signal_names(), &required_input_signal_names_) == false)    { can_start_ = false; }
    if (helpers::output_signals_check(gui_if_->get_f32_output_signal_names(), &required_output_signal_names_) == false) { can_start_ = false; }

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

    ImGui::Separator();
    ImGui::Text("Configure signals");
    helpers::combo_select("th_m_0 source",  gui_if_->get_f32_output_signal_names(), &fb_th_m_0_idx_, NULL);
    helpers::combo_select("w_m_0 source",   gui_if_->get_f32_output_signal_names(), &fb_w_m_0_idx_, NULL);
    helpers::combo_select("i_q source",     gui_if_->get_f32_output_signal_names(), &fb_i_q_idx_, NULL);
    helpers::combo_select("th_m_0 command", gui_if_->get_f32_input_signal_names(),  &cmd_th_m_0_idx_, NULL);

    ImGui::Separator();
    ImGui::Text("Starting this test will start JCS system. Ensure it is safe to do so.");
    if (ImGui::Button("Start test")) {
        switch (state_) {
        default:
        case behaviour::standby_s:
            initialise();
            // Start JCS host
            if (gui_if_->start() != jcs::RET_OK) {
                state_ = behaviour::standby_s;
                break;
            }
            // Get the zero position at which compensation takes place
            if (host_->read_float(target_device_, "encoder_0_position_offset", &compensated_at_zero_pos_) != jcs::RET_OK) {
                state_ = behaviour::standby_s;
                break;
            }
            helpers::sleep_ms(50);
            state_ = behaviour::wait_position_s;
            break;
        case behaviour::wait_position_s:
        case behaviour::rotate_s:
        case behaviour::finish_s: 
            break;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel test")) {
        switch (state_) {
        default:
        case behaviour::standby_s: 
            break;
        case behaviour::wait_position_s:
        case behaviour::rotate_s:
        case behaviour::finish_s: 
            gui_if_->stop();
            state_ = behaviour::standby_s;
            break;
        }
    }    

    ImGui::Separator();
    switch(state_) {
        default:
        case behaviour::standby_s: 
            ImGui::Text("STOPPED");
            break;

        case behaviour::wait_position_s:
        case behaviour::rotate_s:
            ImGui::Text("RUNNING");    
            break;

        case behaviour::finish_s:
            ImGui::Text("RUNNING"); 
            gui_if_->stop();
            compute_outputs();
            state_ = behaviour::standby_s;
            break;
    }

    // Some nice stats
    static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | 
                                         ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings;
    if (ImGui::BeginTable("Measurements", 2, table_flags)) {
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("th_m_0");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%.6f", signals_out_[fb_th_m_0_idx_]);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("w_m_0");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%.6f", signals_out_[fb_w_m_0_idx_]);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("i_q");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%.6f", signals_out_[fb_i_q_idx_]);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("th_m");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%.6f", signals_in_[cmd_th_m_0_idx_]);

        ImGui::EndTable();
    }
    {
        float progress = (float)rotation_tick_ / (float)rotation_steps_; 
        char buf[32];
        sprintf(buf, "%d/%d", rotation_tick_, rotation_steps_);
        ImGui::ProgressBar(progress, ImVec2(0.0f, 0.0f), buf);
        ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
        ImGui::Text("Test progress");
    }
    // {
    //     float error = (1.0f - (theta_error_ / (2.0f*(float)M_PI)));
    //     ImGui::ProgressBar(error, ImVec2(0.0f, 0.0f));
    //     ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
    //     ImGui::Text("Position error");
    // }

    plot_result_.plot();

    ImGui::Separator();
    ImGui::Text("The final result has the following:");
    ImGui::Text(" - Angle in the range [0, 2pi].");
    ImGui::Text(" - Any bias due to friction removed. (Subract mean i_q)");

    ImGui::Text("Raw output mean: %.3f", computed_mean_);

    if (ImGui::Button("Recompute outputs")) { compute_outputs(); }

    plot_final_.plot();
    plot_vis_.plot();

    ImGui::Separator();
    ImGui::Text("Cogging compensator activation range");
    {
        float value = cogging_compensator_upper_w_m_low_;
        if (ImGui::InputFloat("cogging_compensator_upper_w_m_low", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            cogging_compensator_upper_w_m_low_ = value;
        }
    }
    {
        float value = cogging_compensator_upper_w_m_high_;
        if (ImGui::InputFloat("cogging_compensator_upper_w_m_high", &value, 0.1f, 1.0f, "%.6f", ImGuiInputTextFlags_EscapeClearsAll)) {
            cogging_compensator_upper_w_m_high_ = value;
        }
    }

    ImGui::Separator();
    if (ImGui::Button("Write coefficienct to device")) {
        host_->write_float(target_device_, "cogging_compensator_coeffs", plot_final_.y_);
        host_->write_float(target_device_, "cogging_compensator_upper_w_m_low", cogging_compensator_upper_w_m_low_);
        host_->write_float(target_device_, "cogging_compensator_upper_w_m_high", cogging_compensator_upper_w_m_high_);
    }

    write_coeffs_to_file();

    // Cleanup, but don't clear is_ready here
    if (!is_ready_) {
        ImGui::EndDisabled();
    }

    return jcs::RET_OK;
}

int gui_mc_cogging::ready_test() {
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
    helpers::sleep_ms(200);
    return jcs::RET_OK;
}

void gui_mc_cogging::initialise() {
    theta_error_ = 0.0f;
    theta_command_ = 0.0f;
    rotation_tick_ = 0;
    computed_mean_ = 0.0f;
}

void gui_mc_cogging::compute_outputs() {
    computed_mean_ = std::accumulate(plot_result_.y_.begin(), plot_result_.y_.end(), 0.0) / plot_result_.y_.size();

    // x and y for both plots are the same size
    for (int i=0; i<plot_result_.y_.size(); i++) {
        plot_final_.x_[i] = helpers::angle_norm_2pi(plot_result_.x_[i]);
        plot_final_.y_[i] = plot_result_.y_[i] - computed_mean_;

        plot_vis_.x_[i] = (5.0 + plot_final_.y_[i]) * cos(plot_final_.x_[i]); 
        plot_vis_.y_[i] = (5.0 + plot_final_.y_[i]) * sin(plot_final_.x_[i]); 
    }
}

int gui_mc_cogging::write_coeffs_to_file() {
    // Choose file to write to
    if (ImGui::Button("Write coefficients to file")) {
        IGFD::FileDialogConfig config;
        config.path = ".";
        ImGuiFileDialog::Instance()->OpenDialog("choose_dir_key", "Choose Directory", nullptr, config);
    }
    // display
    if (ImGuiFileDialog::Instance()->Display("choose_dir_key"))  {
        if (ImGuiFileDialog::Instance()->IsOk()) {
            if (emit_config(ImGuiFileDialog::Instance()->GetCurrentPath()) != jcs::RET_OK) {
                return jcs::RET_ERROR;
            }
        }
        ImGuiFileDialog::Instance()->Close();
    }
    return jcs::RET_OK;
}

int gui_mc_cogging::emit_config(std::string const& file_path) {
    // Write to file
    std::string file_name = "dev_" + target_device_ + "_cogging_compensator_coefficients.yaml";
    std::string path_and_file = file_path + "/" + file_name;

    std::cout << "Writing to: " << path_and_file << "\n";

    std::ofstream config_file(path_and_file); 

    config_file << "  ########################################################################\n";
    config_file << "  # Cogging compensator coefficients\n";
    config_file << "  #\n";
    config_file << "  # Compensation was performed at rotor position offset:\n";
    config_file << "  # encoder_0_position_offset: " << compensated_at_zero_pos_ << "\n";
    config_file << "  #\n";
    config_file << "  # Coefficients map - " << plot_final_.y_.size() << " points\n";

    std::string coefs_key = "  cogging_compensator_coeffs: [ ";
    std::string pre_space(coefs_key.size(), ' ');

    config_file << coefs_key;

    for (int i=0; i<plot_final_.y_.size()-1; i++) {
        config_file << plot_final_.y_[i] << ", ";
        
        if ((i+1) % 32 == 0) {
            config_file << "\n";
            config_file << pre_space;
        }
    }
    config_file << plot_final_.y_.back() << "]\n";

    config_file << "  #\n";
    config_file << "  #  ff_i\n";
    config_file << "  #    ^\n";
    config_file << "  #    |\n";
    config_file << "  #    |      -----------\n";
    config_file << "  #    |     /           \\_ \n";
    config_file << "  #    |    /              \\_\n";
    config_file << "  #    |   /                 \\_\n";
    config_file << "  #    |  /                    \\_\n";
    config_file << "  #    | /                       \\_\n";
    config_file << "  #    |/                          \\_\n";
    config_file << "  #    o---------------------------------------->  w_m\n";
    config_file << "  #          ^          ^           ^\n";
    config_file << "  #          |          |           `----------- cogging_compensator_upper_w_m_high \n";
    config_file << "  #          |           ----------------------- cogging_compensator_upper_w_m_low\n";
    config_file << "  #           ---------------------------------- cogging_compensator_lower_w_m_high\n";
    config_file << "  #\n";
    config_file << "  # Ramp the cogging compensator from no contribution to full contribution\n";
    config_file << "  # over the range 0 Rad/S to cogging_compensator_lower_w_m_high\n";
    config_file << "  # This stops the compensator causing current draw when the rotor is stopped.\n";
    config_file << "  # Notes: \n";
    config_file << "  # - Set to 0 to disable (default)\n";
    config_file << "  # - This feature is heavily affected by rotor speed estimator noise\n";
    config_file << "  # cogging_compensator_lower_w_m_high: 1.0\n";
    config_file << "  #\n";
    config_file << "  # Ramp the cogging compensator from full contribution to no contribution \n";
    config_file << "  # over this range.\n";
    config_file << "  # Cogging compensation has little effect at speed (cogging torque drops), \n";
    config_file << "  # so feedforward term can reduce as the rotor speed goes up \n";
    config_file << "  cogging_compensator_upper_w_m_low:  " << cogging_compensator_upper_w_m_low_ << "\n";
    config_file << "  cogging_compensator_upper_w_m_high: " << cogging_compensator_upper_w_m_high_ << "\n";

    return jcs::RET_OK;
}