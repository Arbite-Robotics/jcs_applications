//
//
//
#ifndef IMGUI_HELPERS_H_
#define IMGUI_HELPERS_H_

#include "imgui.h"

// Disable scope
struct ImGuiDisabled {
    bool active;
    explicit ImGuiDisabled(bool disable) : active(disable) {
        if (active) { ImGui::BeginDisabled(); }
    }
    ~ImGuiDisabled() {
        if (active) { ImGui::EndDisabled(); }
    }
};

#endif