// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef tool_id_H_
#define tool_id_H_

#include "jcs_tool_if.h"

class tool_id : public jcs_tool_if {
public:
    tool_id(std::string name, jcs::jcs_host* host);
    ~tool_id();

    int load_config(std::string tool_config);

    int step_startup_rt();
    int step_rt();
    int step_shutdown_rt();
    int step_parameter_startup();
    int step_parameter();
    int step_parameter_shutdown();

};

#endif