// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_device_load_switch.h"
#include "gui_parameter.h"
#include "gui_oscilloscope.h"
#include "gui_firmware_update.h"
#include <iostream>

#include "jcs_dev_load_switch.h"

gui_device_load_switch::gui_device_load_switch(jcs::jcs_host* host, gui_interface* gui_if, std::string const& name) :
    gui_device_base(host, gui_if, name)
{
    gui_element_.push_back(new gui_parameter(host_, gui_if_, name_,
        &jcs::node_parameter::dev_load_switch::parameters,
        &jcs::node_parameter::dev_load_switch::parameter_enums));
    gui_element_.push_back(new gui_oscilloscope(host_, gui_if_, name_,
        &jcs::node_parameter::dev_load_switch::oscilloscope_sources,
        &jcs::node_parameter::dev_load_switch::oscilloscope_trigger_config,
        jcs::node_parameter::dev_load_switch::oscilloscope_sample_rate_hz,
        jcs::node_parameter::dev_load_switch::oscilloscope_sample_length,
        jcs::node_parameter::dev_load_switch::oscilloscope_n_channels));
    gui_element_.push_back(new gui_firmware_update(host_, gui_if_, name_));
}

int gui_device_load_switch::startup() {
    for (int i=0; i<gui_element_.size(); i++) {
        if (gui_element_[i]->startup() != jcs::RET_OK) { 
            std::cout << "gui_device_load_switch: " << gui_element_[i]->type_name_ << " startup error\n";
            return jcs::RET_ERROR;
        }
    }
    return jcs::RET_OK;
}

int gui_device_load_switch::step_rt() {
    return jcs::RET_OK;
}

int gui_device_load_switch::render() {

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