// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_device_host.h"
#include <iostream>
#include "implot.h"

#include "tool_gui.h"//debug

#include "imgui.h"
#include "helpers.h"

gui_device_host::gui_device_host(jcs::jcs_host* host, gui_interface* gui_if, std::string const& name) :
    gui_device_base(host, gui_if, name)
{
    gui_host_logger_ = new gui_host_logger(host_, gui_if_, name_);
    gui_plot_ = new gui_plot(host_, gui_if_, name_);
    gui_host_statistics_ = new gui_host_statistics(host_, gui_if_, name_);
    gui_host_oscilloscope_ = new gui_host_oscilloscope(host_, gui_if_, name_);
    gui_host_analysis_ = new gui_host_analysis(host_, gui_if_, name_);
    gui_host_network_firmware_update_ = new gui_host_network_firmware_update(host_, gui_if_, name_);
    gui_host_2d_hopper_ = new gui_host_2d_hopper(host_, gui_if_, name_);

    gui_element_.push_back(static_cast<gui_type_base*>(gui_host_logger_));
    gui_element_.push_back(static_cast<gui_type_base*>(gui_plot_));
    gui_element_.push_back(static_cast<gui_type_base*>(gui_host_statistics_));
    gui_element_.push_back(static_cast<gui_type_base*>(gui_host_oscilloscope_));
    gui_element_.push_back(static_cast<gui_type_base*>(gui_host_analysis_));
    gui_element_.push_back(static_cast<gui_type_base*>(gui_host_network_firmware_update_));
    gui_element_.push_back(static_cast<gui_type_base*>(gui_host_2d_hopper_));

    gui_element_host_ptr_.push_back(static_cast<gui_device_host_base*>(gui_host_logger_));
    gui_element_host_ptr_.push_back(static_cast<gui_device_host_base*>(gui_plot_));
    gui_element_host_ptr_.push_back(static_cast<gui_device_host_base*>(gui_host_statistics_));
    gui_element_host_ptr_.push_back(static_cast<gui_device_host_base*>(gui_host_oscilloscope_));
    gui_element_host_ptr_.push_back(static_cast<gui_device_host_base*>(gui_host_analysis_));
    gui_element_host_ptr_.push_back(static_cast<gui_device_host_base*>(gui_host_network_firmware_update_));
    gui_element_host_ptr_.push_back(static_cast<gui_device_host_base*>(gui_host_2d_hopper_));

}

int gui_device_host::startup() {
    for (int i=0; i<gui_element_.size(); i++) {
        if (gui_element_[i]->startup() != jcs::RET_OK) { 
            std::cout << "gui_device_host: " << gui_element_[i]->type_name_ << " startup error\n";
            return jcs::RET_ERROR;
        }
    }
    return jcs::RET_OK;
}

int gui_device_host::step_rt_always() {
    for (int i=0; i<gui_element_host_ptr_.size(); i++) {
        if (gui_element_host_ptr_[i]->step_rt_always() != jcs::RET_OK) {
            return jcs::RET_ERROR;
        }
    }

    return jcs::RET_OK;
}

int gui_device_host::step_rt() {
    for (int i=0; i<gui_element_.size(); i++) {
        if (gui_element_[i]->step_rt() != jcs::RET_OK) { 
            return jcs::RET_ERROR;
        }
    }
    return jcs::RET_OK;
}

int gui_device_host::render() {

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