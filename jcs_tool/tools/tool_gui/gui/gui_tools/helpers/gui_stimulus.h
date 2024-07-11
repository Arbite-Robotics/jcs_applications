// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_STIMULUS_H_
#define GUI_STIMULUS_H_

#include "stimulus.h"
#include <vector>
#include <string>

class gui_stimulus {
public:
    gui_stimulus(double sample_rate_hz);

    void step_rt();

    void render_parameters();
    void start();
    void stop();
    bool is_running();
    float value_get();

private:
    std::vector<stimulus*> stimulus_source_;
    int active_idx_;
    std::vector<std::string> source_names_;
};

#endif