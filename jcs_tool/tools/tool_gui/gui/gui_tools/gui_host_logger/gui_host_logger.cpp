// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_host_logger.h"
#include "jcs_user_external.h"

gui_host_logger::gui_host_logger(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) : 
    gui_type_base("Host logger", host, gui_if, target_device),
    sampler_(host->base_frequency_get(), gui_if->get_f32_output_signal_names(), 10, 10, 10)
{}

int gui_host_logger::startup() {
    if (sampler_.startup((double)jcs::external::time_now_ns()) != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }
    // Confiugure line storage
    f32_output_signal_store_.resize(host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0));
    return jcs::RET_OK;
}

int gui_host_logger::step_rt() {
    return jcs::RET_OK;
}

int gui_host_logger::step_rt_always() {
    host_->sig_output_get_rt(0, &f32_output_signal_store_);
    sampler_.step_rt((double)jcs::external::time_now_ns(), &f32_output_signal_store_);
    return jcs::RET_OK;
}

int gui_host_logger::render() {
    if (ImGui::Button("Sampler start")) {
        sampler_.start();
    }
    ImGui::SameLine();
    if (ImGui::Button("Sampler stop")) {
        sampler_.stop();
    }
    ImGui::Separator();

    sampler_.render_interface();
    ImGui::Separator();

    sampler_.render_status();
    sampler_.render_plots();
    ImGui::Separator();

    sampler_.channels_write_to_file();

    return jcs::RET_OK;
}
