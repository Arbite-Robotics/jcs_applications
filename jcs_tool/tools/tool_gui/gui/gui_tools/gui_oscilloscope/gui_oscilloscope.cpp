
// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_oscilloscope.h"
#include "imgui_stdlib.h"
#include "implot.h"
#include <cmath>
#include <iostream>
#include "helpers.h"
#include "ImGuiFileDialog.h"

#include "jcs_dev_motor_controller.h"

gui_oscilloscope::gui_oscilloscope(jcs::jcs_host* host, std::string const& target_device, 
    std::vector<std::string> const* oscilloscope_sources, std::vector<std::string> const* oscilloscope_trigger_config,
    int const default_sample_rate_hz, int const sample_length, int const n_channels) : 
    gui_type_base("Oscilloscope", host, target_device),
    oscilloscope_sources_(oscilloscope_sources),
    oscilloscope_trigger_config_(oscilloscope_trigger_config),
    sample_rate_(default_sample_rate_hz),
    sample_length_(sample_length),
    n_channels_(n_channels)
{
    trigger_source_ = oscilloscope_sources_->at(0);
    trigger_source_combo_idx_ = 0;
    trigger_config_ = oscilloscope_trigger_config_->at(0);
    trigger_config_combo_idx_ = 0;

    trigger_level_ = 0.0f;
    trigger_buffer_position_ = 0;

    is_done_sampling_ = true;

    for (int i=0; i<n_channels_; i++) {
        channels_[i] = new channel("Channel " + std::to_string(i),
                                   oscilloscope_sources_->at(0),
                                   sample_length_,
                                   sample_rate_);
    }
}

int gui_oscilloscope::startup() {
    return jcs::RET_OK;
}

int gui_oscilloscope::step_rt() {
    return jcs::RET_OK;
}

int gui_oscilloscope::render() {
    if (render_interface() != jcs::RET_OK) {
        // Commands may have failed - return OK to try and continue
        return jcs::RET_ERROR;
    }
    return render_plot();
}

int gui_oscilloscope::render_plot() {
    for (int i=0; i<n_channels_; i++) {
        channels_[i]->plot(); ImGui::Separator();
    }
    return jcs::RET_OK;
}

int gui_oscilloscope::render_interface() {
    ImGui::Text("Oscilloscope Settings");
    ImGuiInputTextFlags input_text_flags = ImGuiInputTextFlags_EscapeClearsAll;
    // Sample rate
    {
        int sample_temp = sample_rate_;
        if (ImGui::InputInt("Sample Rate", &sample_temp, 1, 100, input_text_flags)) {
            sample_rate_ = sample_temp;
            // Update the channels with the new sample rate
            for (int ch=0; ch<4; ch++) {
                channels_[ch]->update_sample_rate(sample_rate_);
            }
        }
    }
    // Display some nice info
    {
        double sample_buffer_time = (double)sample_length_ / (double)sample_rate_;
        ImGui::Text("Sample buffer covers timespan (s): %.4f", sample_buffer_time);
    }
    // Trigger source
    helpers::combo_select("Trigger Source", oscilloscope_sources_, &trigger_source_combo_idx_, &trigger_source_);
    // Trigger config
    helpers::combo_select("Trigger Config", oscilloscope_trigger_config_, &trigger_config_combo_idx_, &trigger_config_);
    // Trigger level
    {
        float trigger_temp = trigger_level_;
        if (ImGui::InputFloat("Trigger Level", &trigger_temp, 0.1f, 1.0f, "%.3f", input_text_flags)) {
            trigger_level_ = trigger_temp;
        }
    }
    // Trigger buffer position
    {
        const unsigned int zero = 0;
        const unsigned int max_buffer = sample_length_;
        ImGui::SliderScalar("Trigger buffer position", ImGuiDataType_U32, &trigger_buffer_position_, &zero, &max_buffer);
    }
    // Channel Source
    for (int i=0; i<n_channels_; i++) {
        helpers::combo_select("Channel " + std::to_string(i) + " Source", oscilloscope_sources_, &channels_[i]->source_combo_idx_, &channels_[i]->source_);
    }

    if (ImGui::Button("Write Settings")) {
        std::cout << "Writing oscilloscope settings\n";
        std::cout << "oscilloscope_sample_rate_hz:          " << sample_rate_ << "\n";
        std::cout << "oscilloscope_trigger_source:          " << trigger_source_ << "\n";
        std::cout << "oscilloscope_trigger_config:          " << trigger_config_ << "\n";
        std::cout << "oscilloscope_trigger_level:           " << trigger_level_ << "\n";
        std::cout << "oscilloscope_trigger_buffer_position: " << trigger_buffer_position_ << "\n";
        for (int i=0; i<n_channels_; i++) {
            std::cout << "oscilloscope_channel_" + std::to_string(i) + "_source: " << channels_[i]->source_ << "\n";
        }
        PARAM_NOTIFY_ERROR( host_->write_uint32(target_device_, "oscilloscope_sample_rate_hz", sample_rate_),    "Parameter failed: oscilloscope_sample_rate_hz" )
        PARAM_NOTIFY_ERROR( host_->write_enum(target_device_,   "oscilloscope_trigger_source", trigger_source_), "Parameter failed: oscilloscope_trigger_source" )
        PARAM_NOTIFY_ERROR( host_->write_enum(target_device_,   "oscilloscope_trigger_config", trigger_config_), "Parameter failed: oscilloscope_trigger_config" )
        PARAM_NOTIFY_ERROR( host_->write_float(target_device_,  "oscilloscope_trigger_level",  trigger_level_),  "Parameter failed: oscilloscope_trigger_level" )
        PARAM_NOTIFY_ERROR( host_->write_uint32(target_device_, "oscilloscope_trigger_buffer_position", trigger_buffer_position_), "Parameter failed: oscilloscope_trigger_buffer_position" )
        for (int i=0; i<n_channels_; i++) {
            PARAM_NOTIFY_ERROR( host_->write_enum(target_device_,  "oscilloscope_channel_" + std::to_string(i) + "_source", channels_[i]->source_), "Parameter failed: oscilloscope_channel_" + std::to_string(i) + "_source" )
        }
        std::cout << "Done\n";
    }
    ImGui::Separator();

    if (is_done_sampling_) {
        ImGui::Text("Status: Ready");
    } else {
        ImGui::Text("Status: Busy");
    }

    ImGui::Separator();
    ImGui::Text("Oscilloscope Controls");

    if (ImGui::Button("Stop")) {
        is_done_sampling_ = true;
        PARAM_NOTIFY( host_->write_command(target_device_, "oscilloscope_stop"), "Parameter failed: oscilloscope_stop" )
    }
    ImGui::SameLine();
    // Manually get sampling status
    if (ImGui::Button("Is Sampling?")) {
        PARAM_NOTIFY( host_->read_bool(target_device_, "oscilloscope_is_done", &is_done_sampling_), "Parameter failed: oscilloscope_is_done")
    }

    // Disable some controls if we are waiting for trigger or sampling
    if (!is_done_sampling_) {
        ImGui::BeginDisabled();
    }

    // Cleanup helper function
    auto clean_up_on_error = [&]() {
        if (!is_done_sampling_) {
            ImGui::EndDisabled();
            is_done_sampling_ = true;
        }
    };

    ImGui::SameLine();
    bool is_waiting_for_trigger = false;
    if (ImGui::Button("Wait For Trigger")) {
        PARAM_NOTIFY_CLEANUP_ERROR( host_->write_command(target_device_, "oscilloscope_wait_trigger"), "Parameter failed: oscilloscope_wait_trigger", clean_up_on_error(); )
        is_waiting_for_trigger = true;
    }

    if (ImGui::Button("Get All Channels")) {
        for (int i=0; i<n_channels_; i++) {
            PARAM_NOTIFY( host_->read_float(target_device_, "oscilloscope_channel_" + std::to_string(i), &channels_[i]->data_), "Parameter failed: oscilloscope_channel_" + std::to_string(i) )
        }
    }
    ImGui::SameLine();
    for (int i=0; i<n_channels_-1; i++) {
        if (ImGui::Button( ("Get Channel " + std::to_string(i)).c_str() )) {
            PARAM_NOTIFY( host_->read_float(target_device_, "oscilloscope_channel_" + std::to_string(i), &channels_[i]->data_), "Parameter failed: oscilloscope_channel_" + std::to_string(i) )
        }
        ImGui::SameLine();
    }
    // Last channel
    int ch = n_channels_-1;
    if (ImGui::Button( ("Get Channel " + std::to_string(ch)).c_str() )) {
        PARAM_NOTIFY( host_->read_float(target_device_, "oscilloscope_channel_" + std::to_string(ch), &channels_[ch]->data_), "Parameter failed: oscilloscope_channel_" + std::to_string(ch) )
    }

    if (!is_done_sampling_) {
        ImGui::EndDisabled();
    }
    // Must set sampling state outside of Imgui disabled
    if (is_waiting_for_trigger) {
        is_done_sampling_ = false;
    }

    ImGui::Separator();
    write_channels_to_file();
    return jcs::RET_OK;
}

gui_oscilloscope::channel::channel(std::string const& name, std::string const& source, int const sample_length, int const initial_sample_rate) :
    name_(name),
    source_(source),
    source_combo_idx_(0),
    data_(sample_length),
    x_(sample_length),
    plot_cursors_(false)
{
    for (int c=0; c<4; c++) {
        cursor_tag_[c] = 0.0;
    }
    update_sample_rate(initial_sample_rate);
}

void gui_oscilloscope::channel::plot() {
    // Imgui internally hashes the text in button to generate an id
    // But! It needs a unique id. Generating heaps of "internal" labelled buttons
    // will generate a lot of not unique ids. Push "this" and use that to generate a hash
    // ImGui::PushID(this);
    ImGui::PushID(name_.c_str());

    ImGui::Checkbox("Show measurement cursors", &plot_cursors_);

    if (ImPlot::BeginPlot(name_.c_str())) {
        ImPlot::SetupAxes("x","y");

        if (plot_cursors_) {
            // X0
            ImPlot::DragLineX(0, &cursor_tag_[0], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
            ImPlot::TagX(cursor_tag_[0], ImVec4(1,0,0,1), "%.3f", cursor_tag_[0]);
            // X1
            ImPlot::DragLineX(1, &cursor_tag_[1], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
            ImPlot::TagX(cursor_tag_[1], ImVec4(1,0,0,1), "%.3f", cursor_tag_[1]);
            // Y0
            ImPlot::DragLineY(2, &cursor_tag_[2], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
            ImPlot::TagY(cursor_tag_[2], ImVec4(1,0,0,1), "%.3f", cursor_tag_[2]);
            // Y1
            ImPlot::DragLineY(3, &cursor_tag_[3], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
            ImPlot::TagY(cursor_tag_[3], ImVec4(1,0,0,1), "%.3f", cursor_tag_[3]);
        }

        ImPlot::PlotLine(source_.c_str(), &x_[0], &data_[0], x_.size());
        ImPlot::EndPlot();
    }
    if (plot_cursors_) {
        static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | 
                                             ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings;

        if (ImGui::BeginTable("Measurements", 2, table_flags)) {
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("delta X");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.6f", cursor_tag_[0] - cursor_tag_[1]);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("delta Y");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.6f", cursor_tag_[2] - cursor_tag_[3]);
            ImGui::EndTable();
        }
    }
    ImGui::PopID();
}

void gui_oscilloscope::channel::update_sample_rate(int sample_rate) {  
    for (int i=0; i<x_.size(); i++) {
        x_[i] = (float)i / (float)sample_rate;
    }    
}

int gui_oscilloscope::write_channels_to_file() {
    // Choose file to write to
    if (ImGui::Button("Write channel data to file")) {
        IGFD::FileDialogConfig config;
        config.path = ".";
        ImGuiFileDialog::Instance()->OpenDialog("choose_dir_key", "Choose File", ".csv", config);
    }
    // display
    if (ImGuiFileDialog::Instance()->Display("choose_dir_key"))  {
        if (ImGuiFileDialog::Instance()->IsOk()) {
            if (emit_data(ImGuiFileDialog::Instance()->GetFilePathName()) != jcs::RET_OK) {
                return jcs::RET_ERROR;
            }

        }
        ImGuiFileDialog::Instance()->Close();
    }
    return jcs::RET_OK;
}

int gui_oscilloscope::emit_data(std::string const& path_and_file) {
    std::cout << "Writing to: " << path_and_file << "\n";

    std::ofstream config_file(path_and_file); 

    // Write out header
    config_file << "t,";
    for (int i=0; i<channels_.size()-1; i++) {
        config_file << channels_[i]->source_ << ",";
    }
    config_file << channels_[channels_.size()-1]->source_ << "\n";
    // Write out data
    for (int i=0; i<channels_[0]->data_.size(); i++) {
        config_file << channels_[0]->x_[i] << ",";
        for (int ch=0; ch<channels_.size()-1; ch++) {
            config_file << channels_[ch]->data_[i] << ",";
        }
        config_file << channels_[channels_.size()-1]->data_[i] << "\n";
    }
    return jcs::RET_OK;
}