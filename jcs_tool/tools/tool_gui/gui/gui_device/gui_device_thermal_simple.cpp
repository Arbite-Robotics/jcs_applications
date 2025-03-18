// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_device_thermal_simple.h"
#include "gui_parameter.h"
#include "gui_firmware_update.h"
#include <iostream>

#include "jcs_dev_thermal_simple.h"

gui_device_thermal_simple::gui_device_thermal_simple(jcs::jcs_host* host, gui_interface* gui_if, std::string const& name) :
    gui_device_base(host, gui_if, name)
{
    gui_element_.push_back(new gui_parameter(host_, gui_if_, name_, 
        &jcs::node_parameter::dev_thermal_simple::parameters, &jcs::node_parameter::dev_thermal_simple::parameter_enums));
    gui_element_.push_back(new gui_firmware_update(host_, gui_if_, name_));
}

int gui_device_thermal_simple::startup() {
    for (int i=0; i<gui_element_.size(); i++) {
        if (gui_element_[i]->startup() != jcs::RET_OK) { 
            std::cout << "gui_device_thermal_simple: " << gui_element_[i]->type_name_ << " startup error\n";
            return jcs::RET_ERROR;
        }
    }
    return jcs::RET_OK;
}

int gui_device_thermal_simple::step_rt() {
    return jcs::RET_OK;
}

int gui_device_thermal_simple::render() {

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