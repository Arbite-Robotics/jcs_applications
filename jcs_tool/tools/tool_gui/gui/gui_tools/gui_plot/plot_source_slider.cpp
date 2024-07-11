// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "plot_source_slider.h"
#include "imgui.h"

void plot_source_slider::update() {
    // std::string info;
    // info.append(node_name_);
    // info.append("::");
    // info.append(name_);
    // ImGui::SliderFloat(info.c_str(), val_ptr_, (float)min_, (float)max_); 

    char label[128];
    sprintf(label, "%s::%s", node_name_.c_str(), name_.c_str());
    ImGui::SliderFloat(label, val_ptr_, (float)min_, (float)max_); 

}