// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "plot_sink_opstate.h"
#include "imgui.h"

plot_sink_opstate::plot_sink_opstate(std::string const& node_name, std::string const& name, uint8_t* val_ptr) : plot_sink(node_name, name) {
    val_ptr_ = val_ptr;
}

void plot_sink_opstate::update() {
    // Plot column
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    // Update data
    uint8_t value = (uint8_t)(*val_ptr_);
    // // Update interface
    // std::string info;
    // // Dev name and signal
    // info.append(name_);
    // info.append("::");
    // info.append(node_name_);

    // Decode 1st 3 bits of op state
    ImGui::PushID(this);
    uint8_t opstate = value & 0x07;
    switch (opstate) {
        case 0: //off
            ImGui::TextDisabled("off");
            break;
        case 1: // init
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "init");
            break;
        case 2: // running
            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "running");
            break;
        default:
        case 3: // error
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "error");
            break;
    }
    ImGui::SameLine(100);
    // Decode remaining 5 bits as general flags
    bool bit_val = 0;
    uint8_t flags = value;
    for (int i=0; i<4; i++) {
        bit_val = (flags & 0x80) ? 1 : 0;
        flags = flags << 1;
        ImGui::PushID(i);
        ImGui::Checkbox("##", &bit_val); ImGui::SameLine();
        ImGui::PopID();
    }
    bit_val = (flags & 0x80) ? 1 : 0;
    flags = flags << 1;
    ImGui::Checkbox("##", &bit_val);

    ImGui::SameLine();

    // Print hex of opstate + node name etc
    ImGui::Text("(0x%02x)  %s::%s", value, name_.c_str(), node_name_.c_str());
    ImGui::PopID();

    // Information column
    ImGui::TableSetColumnIndex(1);
    // Nothing
}
