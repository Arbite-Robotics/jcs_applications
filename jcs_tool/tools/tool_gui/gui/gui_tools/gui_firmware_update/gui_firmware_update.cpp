
// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_firmware_update.h"
#include <iostream>

#include "ImGuiFileDialog.h"

gui_firmware_update::gui_firmware_update(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) : 
    gui_type_base("Firmware", host, gui_if, target_device)
{
    fw_write_active_ = false;
    fw_status_ = status::standby_s;
    fw_name_ = "";
    fw_path_ = "";

    fl_write_active_ = false;
    fl_status_ = status::standby_s;
    fl_name_ = "";
    fl_path_ = "";
}

int gui_firmware_update::startup() {
    return jcs::RET_OK;
}

int gui_firmware_update::step_rt() {
    return jcs::RET_OK;
}

int gui_firmware_update::step_rt_always() {
    return jcs::RET_OK;
}

int gui_firmware_update::render() {

    ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
    if (ImGui::BeginTabBar("SELECT", tab_bar_flags)) {
        if (ImGui::BeginTabItem("Firmware")) {
            fw_update_render();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Flashloader")) {
            fl_update_render();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    return jcs::RET_OK;
}

void gui_firmware_update::fw_update_render() {
    ImGui::Text("Select firmware file");
    ImGui::Separator();

    ImGui::PushID((target_device_ + "FW").c_str());

    ImGuiFileDialog fileDialog;

    IGFD::FileDialogConfig config;
    config.flags =  ImGuiFileDialogFlags_NoDialog |
                    ImGuiFileDialogFlags_DisableCreateDirectoryButton |
                    ImGuiFileDialogFlags_DisableThumbnailMode |
                    ImGuiFileDialogFlags_DisableBookmarkMode |
                    ImGuiFileDialogFlags_DisableQuickPathSelection |
                    ImGuiFileDialogFlags_ReadOnlyFileNameField;
    config.path = ".";
    ImGuiFileDialog::Instance()->OpenDialog("embedded", "Choose File", ".bin", config);

    // display
    if (ImGuiFileDialog::Instance()->Display("embedded", ImGuiWindowFlags_NoCollapse, ImVec2(0,0), ImVec2(0,350))) {
        // action if OK
        if (ImGuiFileDialog::Instance()->IsOk()) {
            fw_status_ = status::standby_s;
            fw_name_ = ImGuiFileDialog::Instance()->GetFilePathName();
            fw_path_ = ImGuiFileDialog::Instance()->GetCurrentPath();
            // action
            std::cout << "Got file  " << fw_name_ << "\n";
        }
    }
    ImGui::PopID();

    ImGui::Separator();

    ImGui::Text("Target device: %s", target_device_.c_str());
    ImGui::Text("Firmware path: %s", fw_path_.c_str());
    ImGui::Text("Firmware file: %s", fw_name_.c_str());
    // ImGui::Text("Firmware file: %s", );

    ImGui::Checkbox("Activate", &fw_write_active_);

    if (fw_write_active_) {
        if (ImGui::Button("Write Firmware")) {
            if (host_->write_new_firmware(target_device_, fw_name_) == jcs::RET_OK) {
                fw_status_ = status::success_s;
            } else {
                fw_status_ = status::failed_s;
            }
            fw_write_active_ = false;
        }
    }
    switch (fw_status_) {
        default:
        case status::standby_s:
            break;
        case status::success_s:
            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Firmware write successful!");
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "WARNING!");
            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Device %s will not respond correctly until power cycle!", target_device_.c_str());
            break;
        case status::failed_s:
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "ERROR!");
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "Firmware write unsuccessful. See jcs_host logs.");
            break;
    }
}

void gui_firmware_update::fl_update_render() {
    ImGui::Text("Select flashloader file");
    ImGui::Separator();

    ImGui::PushID((target_device_ + "FL").c_str());

    ImGuiFileDialog fileDialog;

    IGFD::FileDialogConfig config;
    config.flags =  ImGuiFileDialogFlags_NoDialog |
                    ImGuiFileDialogFlags_DisableCreateDirectoryButton |
                    ImGuiFileDialogFlags_DisableThumbnailMode |
                    ImGuiFileDialogFlags_DisableBookmarkMode |
                    ImGuiFileDialogFlags_DisableQuickPathSelection |
                    ImGuiFileDialogFlags_ReadOnlyFileNameField;
    config.path = ".";
    ImGuiFileDialog::Instance()->OpenDialog("embedded", "Choose File", ".bin", config);

    // display
    if (ImGuiFileDialog::Instance()->Display("embedded", ImGuiWindowFlags_NoCollapse, ImVec2(0,0), ImVec2(0,350))) {
        // action if OK
        if (ImGuiFileDialog::Instance()->IsOk()) {
            fl_status_ = status::standby_s;
            fl_name_ = ImGuiFileDialog::Instance()->GetFilePathName();
            fl_path_ = ImGuiFileDialog::Instance()->GetCurrentPath();
            // action
            std::cout << "Got file  " << fl_name_ << "\n";
        }
    }
    ImGui::PopID();

    ImGui::Separator();

    ImGui::Text("Target device: %s", target_device_.c_str());
    ImGui::Text("Flashloader path: %s", fl_path_.c_str());
    ImGui::Text("Flashloader file: %s", fl_name_.c_str());

    ImGui::Checkbox("Activate", &fl_write_active_);

    if (fl_write_active_) {
        if (ImGui::Button("Write Flashloader")) {
            if (host_->write_new_flashloader(target_device_, fl_name_) == jcs::RET_OK) {
                fl_status_ = status::success_s;
            } else {
                fl_status_ = status::failed_s;
            }
            fl_write_active_ = false;
        }
    }
    switch (fl_status_) {
        default:
        case status::standby_s:
            break;
        case status::success_s:
            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Flashloader write successful!");
            break;
        case status::failed_s:
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "ERROR!");
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "Flashloader write unsuccessful. See jcs_host logs.");
            break;
    }
}