// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_device_host.h"
#include <iostream>
#include "implot.h"
#include "gui_plot.h"
#include "gui_host_statistics.h"
#include "gui_host_logger.h"
#include "gui_host_oscilloscope.h"
#include "gui_host_analysis.h"
#include "gui_host_2d_hopper.h"
#include "tool_gui.h"//debug

#include "imgui.h"
#include "helpers.h"

gui_device_host::gui_device_host(jcs::jcs_host* host, std::string const& name) :
    gui_device_base(host, name)
{
    gui_element_.push_back(new gui_host_logger(host_, name_));
    gui_element_.push_back(new gui_plot(host_, name_));
    gui_element_.push_back(new gui_host_statistics(host_, name_));
    gui_element_.push_back(new gui_host_oscilloscope(host_, name_));
    gui_element_.push_back(new gui_host_analysis(host_, name_));
    gui_element_.push_back(new gui_host_2d_hopper(host_, name_));
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
    // Alway tick the logger over
    // Logger is at index 0
    if (static_cast<gui_host_logger*>(gui_element_[0])->step_rt_special() != jcs::RET_OK) {
        return jcs::RET_ERROR;
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