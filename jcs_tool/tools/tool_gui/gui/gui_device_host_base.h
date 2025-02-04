// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_DEVICE_HOST_BASE_H_
#define GUI_DEVICE_HOST_BASE_H_

class gui_device_host_base {
public:
    virtual int step_rt_always() = 0;
};

#endif