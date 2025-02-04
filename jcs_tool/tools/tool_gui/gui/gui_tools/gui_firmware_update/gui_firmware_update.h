// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_FIRMWARE_UPDATE_H_
#define GUI_FIRMWARE_UPDATE_H_

#include "jcs_host.h"
#include "gui_type_base.h"
#include "gui_device_host_base.h"
#include "tool_gui_settings.h"
#include <vector>
#include <string>
#include "imgui.h"

class gui_firmware_update : public gui_type_base, public gui_device_host_base {
public:
    gui_firmware_update(jcs::jcs_host* host, std::string const& target_device);
    ~gui_firmware_update() {}

    int startup();
    int step_rt();
    int step_rt_always();
    int render();

private:
    enum class status {
        standby_s,
        success_s,
        failed_s
    };
    // Firmware
    std::string fw_name_;
    std::string fw_path_;
    bool fw_write_active_;
    status fw_status_;
    void fw_update_render();

    // Bootloader
    std::string fl_name_;
    std::string fl_path_;
    bool fl_write_active_;
    status fl_status_;
    void fl_update_render();

};

#endif