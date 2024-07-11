// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_device_motor_controller.h"
#include "gui_parameter.h"
#include "gui_oscilloscope.h"
#include "gui_mc_encoder.h"
#include "gui_mc_tune.h"
#include "gui_mc_cogging.h"
#include "gui_firmware_update.h"
#include <iostream>

#include "jcs_dev_motor_controller.h"

gui_device_motor_controller::gui_device_motor_controller(jcs::jcs_host* host, std::string const& name) :
    gui_device_base(host, name)
{
    gui_element_.push_back(new gui_parameter(host_, name_,
        &jcs::node_parameter::dev_motor_controller::parameters,
        &jcs::node_parameter::dev_motor_controller::parameter_enums));
    gui_element_.push_back(new gui_oscilloscope(host_, name_,
        &jcs::node_parameter::dev_motor_controller::oscilloscope_sources,
        &jcs::node_parameter::dev_motor_controller::oscilloscope_trigger_config,
        jcs::node_parameter::dev_motor_controller::oscilloscope_sample_rate_hz,
        jcs::node_parameter::dev_motor_controller::oscilloscope_sample_length,
        jcs::node_parameter::dev_motor_controller::oscilloscope_n_channels));
    gui_element_.push_back(new gui_mc_encoder(host_, name_));
    gui_element_.push_back(new gui_mc_tune(host_, name_));
    gui_element_.push_back(new gui_mc_cogging(host_, name_));
    gui_element_.push_back(new gui_firmware_update(host_, name_));
}


int gui_device_motor_controller::startup() {
    for (int i=0; i<gui_element_.size(); i++) {
        if (gui_element_[i]->startup() != jcs::RET_OK) { 
            std::cout << "gui_device_motor_controller: " << gui_element_[i]->type_name_ << " startup error\n";
            return jcs::RET_ERROR;
        }
    }
    return jcs::RET_OK;
}

int gui_device_motor_controller::step_rt() {
    for (int i=0; i<gui_element_.size(); i++) {
        if (gui_element_[i]->step_rt() != jcs::RET_OK) { 
            return jcs::RET_ERROR;
        }
    }
    return jcs::RET_OK;
}

int gui_device_motor_controller::render() {

    ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
    if (ImGui::BeginTabBar("SELECT", tab_bar_flags))
    {
        for (int i=0; i<gui_element_.size(); i++) {
            if (ImGui::BeginTabItem(gui_element_[i]->type_name_.c_str()))
            {
                if (gui_element_[i]->render() != jcs::RET_OK) {
                    ImGui::EndTabItem();
                    return jcs::RET_ERROR;
                }
                ImGui::EndTabItem();
            }
        }
        ImGui::EndTabBar();
    }

    return jcs::RET_OK;
}