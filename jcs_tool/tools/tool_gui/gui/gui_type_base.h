// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_TYPE_BASE_H_
#define GUI_TYPE_BASE_H_

#include "jcs_host.h"
#include <string>

class gui_type_base {
public:
    gui_type_base(std::string type_name, jcs::jcs_host* host, std::string const& target_device) :
        type_name_(type_name), host_(host), target_device_(target_device) {}
    ~gui_type_base() {}

    virtual int startup() = 0;
    virtual int step_rt() = 0;
    virtual int render() = 0;

// protected:
    std::string type_name_;
    jcs::jcs_host* host_;
    std::string target_device_;
};

#endif