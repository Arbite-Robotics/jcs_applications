// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
// 
#ifndef TOOL_GUI_H_
#define TOOL_GUI_H_

#include "jcs_tool_if.h"
#include <vector>
#include <string>
#include "imgui.h"

#include "imgui_impl_glfw.h"
#include <GLFW/glfw3.h>
#include "imgui_impl_opengl2.h"

#include <chrono>

#include "gui_device_base.h"

class tool_gui : public jcs_tool_if {
public:
    tool_gui(std::string name, jcs::jcs_host* host);
    ~tool_gui();

    int load_config(std::string tool_config);

    int step_startup_rt();
    int step_rt();
    int step_shutdown_rt();
    int step_parameter_startup();
    int step_parameter();
    int step_parameter_shutdown();

private:
    int render_display();
    int render_top_display(ImVec2* w_pos, ImVec2* w_size);

    void build_store();

    std::vector<jcs::jcs_device>* device_tree_;
    std::vector<gui_device_base*> store_;
    // Device selection helpers
    int device_select_idx_;

    enum class run_status {
        running,
        stopped,
        estop
    };
    run_status run_status_;

    // Imgui
    GLFWwindow* window_;
    ImVec4 clear_color_;
    std::chrono::system_clock::time_point t_next_;

    void style_dracula_darker();
};

#endif