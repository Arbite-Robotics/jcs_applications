// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_PROCESS_PID_H_
#define GUI_PROCESS_PID_H_

#include <string>
#include "gui_device_base.h"
#include "gui_interface.h"
#include "jcs_host.h"

#include "jcs_proc_pid.h"

class gui_process_pid : public gui_device_base {
public:
    gui_process_pid(jcs::jcs_host* host, gui_interface* gui_if, std::string const& name);
    ~gui_process_pid() {}

    int startup();
    int step_rt();
    int render(); 
};
#endif