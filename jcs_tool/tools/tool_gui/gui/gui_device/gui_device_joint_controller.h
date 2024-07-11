// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_DEVICE_JOINT_CONTROLLER_H_
#define GUI_DEVICE_JOINT_CONTROLLER_H_

#include <string>
#include "gui_device_base.h"
#include "jcs_host.h"

#include "jcs_dev_joint_controller.h"

class gui_device_joint_controller : public gui_device_base {
public:
    gui_device_joint_controller(jcs::jcs_host* host, std::string const& name);
    ~gui_device_joint_controller() {}

    int startup();
    int step_rt();
    int render(); 
};
#endif