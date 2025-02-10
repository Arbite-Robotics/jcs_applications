// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_DEVICE_ENCODER_ABSOLUTE_H_
#define GUI_DEVICE_ENCODER_ABSOLUTE_H_

#include <string>
#include "gui_device_base.h"
#include "gui_interface.h"
#include "jcs_host.h"

class gui_device_encoder_absolute : public gui_device_base {
public:
    gui_device_encoder_absolute(jcs::jcs_host* host, gui_interface* gui_if, std::string const& name);
    ~gui_device_encoder_absolute() {}

    int startup();
    int step_rt();
    int render(); 

};
#endif