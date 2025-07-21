// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_host_input_stimulus.h"
#include "jcs_user_external.h"
#include "helpers.h"

gui_host_input_stimulus::gui_host_input_stimulus(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) :
    gui_type_base("Input stimulus", host, gui_if, target_device),
    state_(state::off_s)
{
    channels_.resize(6);
}

int gui_host_input_stimulus::startup() {
    state_ = state::off_s;
    // Build the channels first
    for (int i=0; i<channels_.size(); i++) {
        channels_[i] = new channel(static_cast<double>(host_->base_frequency_get()));
    }
    f32_isignal_store_.resize(host_->sig_input_sz_unsafe_rt(jcs::signal_type::float32_s, 0));
    return jcs::RET_OK;
}

int gui_host_input_stimulus::step_rt() {
    return jcs::RET_OK;
}

int gui_host_input_stimulus::step_rt_always() {
    switch (state_) {
        default:
        case state::off_s:
            break;

        case state::running_s:
            {
                bool all_done = true;
                for (int i=0; i<channels_.size(); i++) {
                    if (channels_[i]->is_active_) {
                        if (channels_[i]->input_stimulus_->is_running()) {
                            all_done = false;
                        }
                        channels_[i]->input_stimulus_->step_rt();
                        f32_isignal_store_[channels_[i]->input_combo_idx_] = channels_[i]->input_stimulus_->value_get();
                    }
                }
                if (all_done) {
                    state_ = state::off_s;
                }
            }
            host_->sig_input_set_rt(0, f32_isignal_store_);
            break;
    }
    return jcs::RET_OK;
}

int gui_host_input_stimulus::render() {
    ImGui::Text("Select input stimuli");
    ImGui::Text("Notes:");
    ImGui::Text("- Ensure Signal Plot -> Input signals active is unticked ");

    ImGui::Separator();
    ImGui::Text("When stimuli configured, click start");

    if (ImGui::Button("Stimulus start")) {
        for (int i=0; i<channels_.size(); i++) {
            if (channels_[i]->is_active_) {
                channels_[i]->input_stimulus_->start();
            }
        }
        state_ = state::running_s;
    }
    ImGui::SameLine();
    if (ImGui::Button("Stimulus stop")) {
        for (int i=0; i<channels_.size(); i++) {
            channels_[i]->input_stimulus_->stop();
        }
        state_ = state::off_s;
    }
    ImGui::Separator();
    ImGui::Text("Status: ");
    ImGui::SameLine();
    switch (state_) {
        default:
        case state::off_s:
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.0f, 1.0f), "Off");
            break;
        case state::running_s:
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "Running");
            break;
    }

    for (int i=0; i<channels_.size(); i++) {
        ImGui::Separator();
        ImGui::Text("Stimulus channel %u", i);
        ImGui::SameLine();
        if (channels_[i]->input_stimulus_->is_running()) {
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "Running");
        } else {
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.0f, 1.0f), "Off");
        }
        std::string checkbox_id = "Active##" + std::to_string(i);
        ImGui::Checkbox(checkbox_id.c_str(), &channels_[i]->is_active_);
        helpers::combo_select("Stimulus signal input ##" + std::to_string(i), gui_if_->get_f32_input_signal_names(), &channels_[i]->input_combo_idx_, nullptr);
        channels_[i]->input_stimulus_->render_parameters();
    }

    return jcs::RET_OK;
}

gui_host_input_stimulus::channel::channel(double base_freq_hz) {
    input_stimulus_ = new gui_stimulus(base_freq_hz);
    input_combo_idx_ = 0;
    is_active_ = false;
}