// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_DEVICE_BASE_H_
#define GUI_DEVICE_BASE_H_

#include <string>
#include <vector>
#include "gui_type_base.h"
#include "gui_interface.h"
#include "jcs_host.h"
#include "imgui.h"

class gui_device_base {
public:
    gui_device_base(jcs::jcs_host* host, gui_interface* gui_if, std::string const& name) :
        host_(host), gui_if_(gui_if), name_(name) {}
    ~gui_device_base() {}

    std::string const& name_get() { return name_; }

    virtual int startup() = 0;
    virtual int step_rt() = 0;
    virtual int render() = 0; 

protected:
    std::vector<gui_type_base*> gui_element_;
    std::string name_;
    jcs::jcs_host* host_;
    gui_interface* gui_if_;

};
#endif