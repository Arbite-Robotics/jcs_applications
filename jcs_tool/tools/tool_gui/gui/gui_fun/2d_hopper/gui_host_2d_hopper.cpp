// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_host_2d_hopper.h"
#include "imgui_stdlib.h"
#include <iostream>
#include "helpers.h"

#include "hopper_2d_simple_kine.h"
#include "hopper_2d_vmc_simple_xy_ctl.h"
#include "hopper_2d_virtual_model_ctl.h"

gui_host_2d_hopper::gui_host_2d_hopper(jcs::jcs_host* host, std::string const& target_device) : 
    gui_type_base("Host 2d hopper", host, target_device)
{
    active_idx_ = 0;

    hopper_control_source_.push_back(new hopper_2d_simple_kine());
    hopper_control_source_.push_back(new hopper_2d_vmc_simple_xy_ctl());
    hopper_control_source_.push_back(new hopper_2d_virtual_model_ctl());

    for (int i=0; i<hopper_control_source_.size(); i++) {
        source_names_.push_back(hopper_control_source_[i]->name_get());
    }

    controller_state_ = controller_state::stopped_s;
}

int gui_host_2d_hopper::startup() {
    // Resize base rate float storage vector
    f32_osignal_store_.resize(host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0));
    f32_isignal_store_.resize(host_->sig_input_sz_unsafe_rt(jcs::signal_type::float32_s, 0));

    // Build a list of all available output float type, base rate signals
    for (int i=0; i<host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0); i++) {
        std::string node_name;
        if (host_->sig_output_node_name_get(jcs::signal_type::float32_s, 0, i, &node_name) != jcs::RET_OK) {
            std::cout << "gui_host_2d_hopper: Error getting node name for output signal at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        std::string name;
        if (host_->sig_output_name_get(jcs::signal_type::float32_s, 0, i, &name) != jcs::RET_OK) {
            std::cout << "gui_host_2d_hopper: Error getting output signal name at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        // Name will be node name + signal name
        f32_output_signal_names_.push_back(node_name + "::" + name);
    }

    // Build a list of all available input float type, base rate signals
    for (int i=0; i<host_->sig_input_sz_unsafe_rt(jcs::signal_type::float32_s, 0); i++) {
        std::string node_name;
        if (host_->sig_input_node_name_get(jcs::signal_type::float32_s, 0, i, &node_name) != jcs::RET_OK) {
            std::cout << "gui_host_2d_hopper: Error getting node name for input signal at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        std::string name;
        if (host_->sig_input_name_get(jcs::signal_type::float32_s, 0, i, &name) != jcs::RET_OK) {
            std::cout << "gui_host_2d_hopper: Error getting input signal name at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        // Name will be node name + signal name
        f32_input_signal_names_.push_back(node_name + "::" + name);
    }

    // Startup
    for (int i=0; i<hopper_control_source_.size(); i++) {
        // Connect the signal store
        hopper_control_source_[i]->hopper_startup(&f32_osignal_store_, &f32_isignal_store_);
        // Startup the contol sources
        if (hopper_control_source_[i]->startup(static_cast<double>(host_->base_frequency_get())) != jcs::RET_OK) {
            std::cout << "gui_host_2d_hopper: Control source: " << hopper_control_source_[i]->name_get() << " failed to start\n";
            return jcs::RET_ERROR;
        }
    }

    return jcs::RET_OK;
}

int gui_host_2d_hopper::step_rt() {
    switch (controller_state_) {
        default:
        case controller_state::stopped_s:
            break;

        case controller_state::init_s:
            if (hopper_control_source_[active_idx_]->step_rt_init() != jcs::RET_OK) {
                controller_state_ = controller_state::stopped_s;
            }
            controller_state_ = controller_state::running_s;
            break;

        case controller_state::running_s:
            // Only currently supporting base rates
            host_->sig_output_get_rt(0, &f32_osignal_store_);
            hopper_control_source_[active_idx_]->step_rt_run();
            host_->sig_input_set_rt(0, f32_isignal_store_);
            break;
    }
    return jcs::RET_OK;
}

int gui_host_2d_hopper::step_rt_always() {
    return jcs::RET_OK;
}

int gui_host_2d_hopper::render() {

    helpers::combo_select("Active hopper controller", &source_names_, &active_idx_, nullptr);

    ImGui::Separator();

    ImGui::Text("Controller State: ");
    ImGui::SameLine();
    switch (controller_state_) {
        default:
        case controller_state::stopped_s:
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.0f, 1.0f), "Stopped");
            break;

        case controller_state::init_s:
        case controller_state::running_s:
            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Running");
            break;
    }

    if (!hopper_control_source_[active_idx_]->can_start()) {
        ImGui::Text("Active controller can't start. Check signal configuration");
        ImGui::BeginDisabled();
    }

    ImGui::Text("Controller: ");
    ImGui::SameLine();
    if (ImGui::Button("Start")) {
        controller_state_ = controller_state::init_s;
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop")) {
        controller_state_ = controller_state::stopped_s;
    }

    ImGui::Separator();

    hopper_control_source_[active_idx_]->render();

    if (!hopper_control_source_[active_idx_]->can_start()) {
        ImGui::EndDisabled();
    }

    return jcs::RET_OK;
}
