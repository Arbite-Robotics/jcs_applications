// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_PROCESS_INTERPOLATOR_H_
#define GUI_PROCESS_INTERPOLATOR_H_

#include <string>
#include "gui_device_base.h"
#include "jcs_host.h"

#include "jcs_proc_interpolator.h"

class gui_process_interpolator : public gui_device_base {
public:
    gui_process_interpolator(jcs::jcs_host* host, std::string const& name);
    ~gui_process_interpolator() {}

    int startup();
    int step_rt();
    int render(); 
};
#endif