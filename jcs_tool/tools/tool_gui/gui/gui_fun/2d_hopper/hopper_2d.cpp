// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "hopper_2d.h"
#include "imgui.h"

void hopper_2d::hopper_startup(std::vector<float>* f32_jcs_osig, std::vector<float>* f32_jcs_isig) {
    // Swap the signals to ease thinknig about it when buried in the controller
    // JCS output signals connect to controller input signals
    f32_ctl_isig_ = f32_jcs_osig ;
    // JCS input signals connect to controller output signals
    f32_ctl_osig_ = f32_jcs_isig;
}