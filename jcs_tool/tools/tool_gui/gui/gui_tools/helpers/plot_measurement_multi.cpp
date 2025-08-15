// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "plot_measurement_multi.h"

#include "imgui_stdlib.h"
#include <iostream>
#include <algorithm>
#include "implot.h"
#include "ImGuiFileDialog.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Multi-channel plotting class implementation
plot_measurement_multi::plot_measurement_multi(std::string const& name,
                                               std::string const& x_name, std::string const& y_name,
                                               int const size) :
    name_(name), x_name_(x_name), y_name_(y_name), size_(size), plot_cursors_(false)
{
    x_.resize(size_);
    for (int i=0; i<4; i++) {
        cursor_tag_[i] = 0.0;
    }
}

plot_measurement_multi::plot_measurement_multi(std::string const& name, 
                                               std::string const& x_name, std::string const& y_name, 
                                               int const sample_time_s, int const sample_rate_hz) :
    plot_measurement_multi(name, x_name, y_name, sample_time_s * sample_rate_hz)
{
    update_sample_rate(sample_rate_hz);
}

void plot_measurement_multi::add_channel(std::string const& channel_name, ImVec4 color) {
    channels_.emplace_back(channel_name, size_, color);
}

void plot_measurement_multi::remove_channel(std::string const& channel_name) {
    channels_.erase(
        std::remove_if(channels_.begin(), channels_.end(),
            [&channel_name](const channel& ch) { return ch.name_ == channel_name; }),
        channels_.end()
    );
}

void plot_measurement_multi::clear_channels() {
    channels_.clear();
}

plot_measurement_multi::channel* plot_measurement_multi::get_channel(std::string const& channel_name) {
    for (std::vector<channel>::iterator it = channels_.begin(); it != channels_.end(); ++it) {
        if (it->name_ == channel_name) {
            return &(*it);
        }
    }
    return nullptr;
}

plot_measurement_multi::channel* plot_measurement_multi::get_channel(int index) {
    if (index >= 0 && index < channels_.size()) {
        return &channels_[index];
    }
    return nullptr;
}

void plot_measurement_multi::plot() {
    ImGui::PushID(name_.c_str());

    // Channel visibility controls
    if (channels_.size() > 0) {
        ImGui::Text("Channels:");
        ImGui::SameLine();
        // for (std::vector<channel>::iterator it = channels_.begin(); it != channels_.end(); ++it) {
        //     ImGui::Checkbox(it->name_.c_str(), &it->visible_);
        //     ImGui::SameLine();
        // }
        for (int i=0; i<channels_.size(); i++) {
            ImGui::Checkbox((channels_[i].name_ + "##" + std::to_string(i)).c_str(), &channels_[i].visible_);
            ImGui::SameLine();
        }
        ImGui::NewLine();
    }
    
    ImGui::Checkbox("Show measurement cursors", &plot_cursors_);
    ImGui::SameLine();
    ImGui::Text("(Double click plot to zoom to extents. Right click plot for menu.)");

    if (ImPlot::BeginPlot(name_.c_str())) {
        ImPlot::SetupAxes(x_name_.c_str(), y_name_.c_str());

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
        
        // Plot all visible channels
        for (std::vector<channel>::iterator it = channels_.begin(); it != channels_.end(); ++it) {
            if (it->visible_ && !it->y_.empty()) {
                ImPlot::PushStyleColor(ImPlotCol_Line, it->color_);
                ImPlot::PlotLine(it->name_.c_str(), &x_[0], &it->y_[0], x_.size());
                ImPlot::PopStyleColor();
            }
        }
        
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
            ImGui::Text("delta %s", x_name_.c_str());
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.6f", cursor_tag_[0] - cursor_tag_[1]);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("delta %s", y_name_.c_str());
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.6f", cursor_tag_[2] - cursor_tag_[3]);
            
            // Show cursor Y values for each visible channel
            if (channels_.size() > 0) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("Channel values at X0:");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text(" ");
                
                for (std::vector<channel>::iterator it = channels_.begin(); it != channels_.end(); ++it) {
                    if (it->visible_) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("  %s", it->name_.c_str());
                        ImGui::TableSetColumnIndex(1);
                        
                        // Find closest sample to cursor X0
                        int idx = 0;
                        float min_dist = std::abs(x_[0] - cursor_tag_[0]);
                        for (int i = 1; i < x_.size(); i++) {
                            float dist = std::abs(x_[i] - cursor_tag_[0]);
                            if (dist < min_dist) {
                                min_dist = dist;
                                idx = i;
                            }
                        }
                        ImGui::Text("%.6f", it->y_[idx]);
                    }
                }
            }
            ImGui::EndTable();
        }
    }
    
    // File export button and dialog - call this at bottom of plot()
    write_channels_to_file();
    
    ImGui::PopID();
}

int plot_measurement_multi::write_channels_to_file() {
    // Choose file to write to
    if (ImGui::Button("Write channel data to file")) {
        IGFD::FileDialogConfig config;
        config.path = ".";
        ImGuiFileDialog::Instance()->OpenDialog("choose_dir_key", "Choose File", ".csv", config);
    }
    
    // Display file dialog
    if (ImGuiFileDialog::Instance()->Display("choose_dir_key")) {
        if (ImGuiFileDialog::Instance()->IsOk()) {
            if (emit_data(ImGuiFileDialog::Instance()->GetFilePathName()) != jcs::RET_OK) {
                return jcs::RET_ERROR;
            }
        }
        ImGuiFileDialog::Instance()->Close();
    }
    return jcs::RET_OK;
}

int plot_measurement_multi::emit_data(std::string const& path_and_file) {
    std::cout << "Writing to: " << path_and_file << "\n";
    
    std::ofstream config_file(path_and_file);
    if (!config_file.is_open()) {
        std::cerr << "Failed to open file: " << path_and_file << "\n";
        return jcs::RET_ERROR;
    }
    
    // Write header row with x_name and all channel names
    config_file << x_name_;
    for (std::vector<channel>::const_iterator it = channels_.begin(); it != channels_.end(); ++it) {
        config_file << "," << it->name_;
    }
    config_file << "\n";
    
    // Write data rows
    for (int i = 0; i < x_.size(); i++) {
        config_file << x_[i];
        for (std::vector<channel>::const_iterator it = channels_.begin(); it != channels_.end(); ++it) {
            if (i < it->y_.size()) {
                config_file << "," << it->y_[i];
            } else {
                config_file << ",";  // Empty value if channel is shorter
            }
        }
        config_file << "\n";
    }
    
    config_file.close();
    std::cout << "Successfully wrote " << x_.size() << " samples to file\n";
    
    return jcs::RET_OK;
}

void plot_measurement_multi::update_sample_rate(int sample_rate_hz) {
    for (int i=0; i<x_.size(); i++) {
        x_[i] = (float)i / (float)sample_rate_hz;
    }
}

void plot_measurement_multi::update_storage_length(int sample_time_s, int sample_rate_hz) {
    int new_sample_length = sample_time_s * sample_rate_hz;
    size_ = new_sample_length;
    x_.resize(new_sample_length);
    
    // Resize all channel data
    for (std::vector<channel>::iterator it = channels_.begin(); it != channels_.end(); ++it) {
        it->y_.resize(new_sample_length);
    }
    
    update_sample_rate(sample_rate_hz);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Example usage:
/*
    // Create multi-channel plot
    plot_measurement_multi multi_plot("Multi-Channel Signal", "Time (s)", "Amplitude", 10, 1000);
    
    // Add channels with custom colors
    multi_plot.add_channel("Channel 1", ImVec4(1.0f, 0.0f, 0.0f, 1.0f));  // Red
    multi_plot.add_channel("Channel 2", ImVec4(0.0f, 1.0f, 0.0f, 1.0f));  // Green
    multi_plot.add_channel("Channel 3", ImVec4(0.0f, 0.0f, 1.0f, 1.0f));  // Blue
    
    // Access channel data for updating
    plot_measurement_multi::channel* ch1 = multi_plot.get_channel("Channel 1");
    if (ch1) {
        for (int i = 0; i < ch1->y_.size(); i++) {
            ch1->y_[i] = sin(2 * M_PI * i / 100.0f);
        }
    }
    
    // Or by index
    plot_measurement_multi::channel* ch2 = multi_plot.get_channel(1);
    if (ch2) {
        for (int i = 0; i < ch2->y_.size(); i++) {
            ch2->y_[i] = cos(2 * M_PI * i / 100.0f);
        }
    }
    
    // Plot in your render loop - includes file export button
    multi_plot.plot();
*/