// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_PARAMETER_H_
#define GUI_PARAMETER_H_

#include "jcs_host.h"
#include "imgui.h"
#include "imgui_stdlib.h"
#include <vector>
#include <string>
#include "gui_type_base.h"
#include "gui_interface.h"
#include "gui_parameter_types.h"
#include "jcs_parameter.h"

//////////////////////////////////////////////////////////////////////
class gui_parameter : public gui_type_base {
public:
    gui_parameter(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device, 
        std::vector<jcs::parameter> const* params, std::vector<jcs::parameter_enum> const* enums);
    ~gui_parameter() {}

    int startup();
    int step_rt();
    int render();

private:
    std::vector<jcs::parameter> const* params_;
    std::vector<jcs::parameter_enum> const* enums_;

    // Local parameter storage
    std::vector<param_base*> param_store_;

    // Helper tool for reading all parameters
    struct parameter_do_all {
        int param_index_;
        float fraction_;
        float increment_;
        enum class state {
            running,
            stopped
        };
        state state_;

        parameter_do_all() : state_(state::stopped), param_index_(0), fraction_(0.0f), increment_(0.0f) {}
        void cancel() { state_ = state::stopped; }
        void tick(std::vector<param_base*>* param_store, std::string const& target_device);
    };

    parameter_do_all do_all_;

    // Tools
    int write_config_to_file();
    int emit_config(std::string const& file_path);
};

#endif