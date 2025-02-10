// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_INTERFACE_H_
#define GUI_INTERFACE_H_

#include <vector>
#include <string>

class gui_interface {
public:
    virtual int start() = 0;
    virtual int stop() = 0;
    virtual std::vector<std::string>* get_f32_input_signal_names() = 0;
    virtual std::vector<std::string>* get_f32_output_signal_names() = 0;
};
#endif