// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef HOPPER_2D_H_
#define HOPPER_2D_H_

#include <string>
#include <vector>

class hopper_2d {
public:
    hopper_2d(std::string const& name) : name_(name) {}

    std::string const& name_get() { return name_; }

    void hopper_startup(std::vector<float>* f32_jcs_osig, std::vector<float>* f32_jcs_isig);

    virtual int startup(double sample_rate_hz) = 0;
    virtual int step_rt_init() = 0;
    virtual int step_rt_run() = 0;
    virtual int render() = 0;
    virtual bool can_start() = 0;


protected:
    std::string name_;
    std::vector<float>* f32_ctl_osig_;
    std::vector<float>* f32_ctl_isig_;
};

#endif