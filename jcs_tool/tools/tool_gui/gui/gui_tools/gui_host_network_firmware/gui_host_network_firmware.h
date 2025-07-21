// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_HOST_NETWORK_FIRMWARE_UPDATE_H_
#define GUI_HOST_NETWORK_FIRMWARE_UPDATE_H_

#include "jcs_host.h"
#include "gui_type_base.h"
#include "gui_interface.h"
#include "gui_device_host_base.h"
#include "tool_gui_settings.h"
#include <vector>
#include <string>
#include "imgui.h"

class gui_host_network_firmware_update : public gui_type_base, public gui_device_host_base {
public:
    gui_host_network_firmware_update(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device);
    ~gui_host_network_firmware_update() {}

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
    std::vector<std::string> fw_names_;
    int fw_selected_;
    bool fw_write_active_;
    status fw_status_;
    bool fw_only_write_listed_;
    void fw_update_render();

    // Bootloader
    std::vector<std::string> fl_names_;
    int fl_selected_;
    bool fl_write_active_;
    status fl_status_;
    bool fl_only_write_listed_;
    void fl_update_render();

};

#endif