// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_host_logger.h"
#include "imgui_stdlib.h"
#include "implot.h"
#include <cmath>
#include <iostream>
#include "ImGuiFileDialog.h"
#include "jcs_user_external.h"

gui_host_logger::gui_host_logger(jcs::jcs_host* host, gui_interface* gui_if, std::string const& target_device) : 
    gui_type_base("Host logger", host, gui_if, target_device)
{
    sampler_state_ = sampler_state::off_s;
    storage_count_ = 0;
    storage_length_ = 0;
    display_points_ = 1000;
    // Default sample time is 10 sec
    sample_time_ = 10;
    for (int i=0; i<channels_.size(); i++) {
        channels_[i] = new channel("Channel " + std::to_string(i),
                                   "None",
                                   sample_time_,
                                   1000);
    }
}

int gui_host_logger::startup() {
    // Resize base rate float storage vector
    f32_osignal_store_.resize(host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0));

    storage_length_ = sample_time_ * host_->base_frequency_get();

    // Update storage for new configured base rate
    // and set default sources
    for (int ch=0; ch<channels_.size(); ch++) {
        channels_[ch]->source_combo_idx_ = 0;
        channels_[ch]->source_ = gui_if_->get_f32_output_signal_names()->at(0);
        channels_[ch]->update_sample_rate(host_->base_frequency_get());
        channels_[ch]->update_storage_length(sample_time_);
    }

    // Sample the start time for the plots.
    // Otherwise times get huuuuuuuge
    t_start_ns_ = (double)jcs::external::time_now_ns();

    return jcs::RET_OK;
}

int gui_host_logger::step_rt() {
    return jcs::RET_OK;
}

// int gui_host_logger::step_rt_special() {
int gui_host_logger::step_rt_always() {
    // Only currently supporting base rates
    host_->sig_output_get_rt(0, &f32_osignal_store_);

    switch (sampler_state_) {
        default:
        case sampler_state::off_s:
            break;

        case sampler_state::sampling_s:
            {
                t_s_ = (float)(((double)jcs::external::time_now_ns() - t_start_ns_)*1e-9);
                // Sampling
                for (int i=0; i<channels_.size(); i++) {
                    channels_[i]->buffer_.add_point(t_s_, f32_osignal_store_[channels_[i]->source_combo_idx_]);
                }
            }
            break;
    }

    return jcs::RET_OK;
}

int gui_host_logger::render() {
    render_status();
    ImGui::Separator();
    if (render_interface() != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }
    return render_plot();
}

void gui_host_logger::render_status() {
    ImGui::Text("logger State: ");
    ImGui::SameLine();
    switch (sampler_state_) {
        default:
        case sampler_state::off_s:
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.0f, 1.0f), "Off");
            break;
        case sampler_state::sampling_s:
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "Sampling");
            break;
    }
}

int gui_host_logger::render_plot() {
    for (int i=0; i<channels_.size(); i++) {
        channels_[i]->plot(); ImGui::Separator();
    }
    return jcs::RET_OK;
}

int gui_host_logger::render_interface() {
    ImGui::Text("logger Settings");

    ImGuiInputTextFlags input_text_flags = ImGuiInputTextFlags_EscapeClearsAll;
    // Storage length
    ImGui::Text("Base sample frequency:  %u", host_->base_frequency_get());
    ImGui::Text("Total sampled points:   %u", host_->base_frequency_get() * sample_time_);
    ImGui::Text("Total displayed points: %u", display_points_);
    {
        int sample_temp = sample_time_;
        if (ImGui::InputInt("Sample time (s)", &sample_temp, 1, 100, input_text_flags)) {
            sample_time_ = sample_temp;
            // Must stop sampling before updating sample time
            sampler_state_ = sampler_state::off_s;
            storage_length_ = sample_time_ * host_->base_frequency_get();
            for (int ch=0; ch<channels_.size(); ch++) {
                channels_[ch]->update_storage_length(sample_time_);
            }
        }
    }
    // Channel Source
    for (int i=0; i<channels_.size(); i++) {
        helpers::combo_select("Channel " + std::to_string(i) + " Source", gui_if_->get_f32_output_signal_names(), &channels_[i]->source_combo_idx_, &channels_[i]->source_);
    }
    // Display sample points
    {
        int display_temp = display_points_;
        if (ImGui::InputInt("Display down sample points", &display_temp, 1, 100, input_text_flags)) {
            display_points_ = display_temp;
            for (int ch=0; ch<channels_.size(); ch++) {
                channels_[ch]->update_display_downsample_points(display_points_);
            }
        }
    }
    // Controls
    if (ImGui::Button("Start sampling")) {
        for (int ch=0; ch<channels_.size(); ch++) {
            channels_[ch]->clear();
        }
        sampler_state_ = sampler_state::sampling_s;
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop sampling")) {
        sampler_state_ = sampler_state::off_s;
    }
    // ImGui::SameLine();
    // if (ImGui::Button("Clear channels")) {
    //     for (int ch=0; ch<channels_.size(); ch++) {
    //         channels_[ch]->clear();
    //     }
    // }

    ImGui::Separator();
    // write_channels_to_file();
    return jcs::RET_OK;
}

gui_host_logger::channel::channel(std::string const& name, std::string const& source, int const sample_time_s, int const sample_rate_hz) :
    name_(name),
    source_(source),
    source_combo_idx_(0),
    buffer_(sample_time_s * sample_rate_hz),
    plot_cursors_(false),
    fit_y_(true), fit_x_(true),
    sample_time_s_(sample_time_s),
    sample_rate_(sample_rate_hz),
    downsample_points_(1000)
{
    for (int c=0; c<4; c++) {
        cursor_tag_[c] = 0.0;
    }
}

void gui_host_logger::channel::clear() {
    for (int i=0; i<buffer_.data_.size(); i++) {
        buffer_.erase();
    }
}

void gui_host_logger::channel::update_sample_rate(int sample_rate_hz) {
    sample_rate_ = sample_rate_hz;
}

void gui_host_logger::channel::update_storage_length(int sample_time_s) {
    int new_sample_length = sample_time_s * sample_rate_;
    buffer_.update_size(new_sample_length);

    // buffer_.add_point(0.0, 0.0);
    sample_time_s_ = sample_time_s;
}

void gui_host_logger::channel::update_display_downsample_points(int downsample_points) {
    downsample_points_ = downsample_points;
}

void gui_host_logger::channel::plot() {
    // Imgui internally hashes the text in button to generate an id
    // But! It needs a unique id. Generating heaps of "internal" labelled buttons
    // will generate a lot of not unique ids. Push "this" and use that to generate a hash
    ImGui::PushID(name_.c_str());

    bool buffer_is_empty = buffer_.data_.empty();

    if (buffer_is_empty) {
        ImGui::Text("Waiting on buffer data");
        ImGui::BeginDisabled(true);
    }

    // ImGui::Checkbox("Show measurement cursors", &plot_cursors_);
    // ImGui::SameLine();
    ImGui::Checkbox("Fit T axis", &fit_x_);
    ImGui::SameLine();
    ImGui::Checkbox("Fit Y axis", &fit_y_);

    if (ImPlot::BeginPlot(name_.c_str())) {

        ImPlotAxisFlags x_flags = fit_x_ ? ImPlotAxisFlags_AutoFit : 0;
        ImPlotAxisFlags y_flags = fit_y_ ? ImPlotAxisFlags_AutoFit : 0;
        
        ImPlot::SetupAxes("t", "y", x_flags, y_flags);


        double t_min = buffer_.last_point().x - sample_time_s_;
        double t_max = buffer_.last_point().x;
        // With ImGuiCond_Always the plot is nice, but I can't move the T axis around
        // With ImGuiCond_Once I see some rendering artifacts, but I can move the T axis around....
        ImPlot::SetupAxisLimits(ImAxis_X1, t_min, t_max, ImGuiCond_Once);

        // Cursors are not working properly...
        // if (plot_cursors_) {
        //     // Move x axis with time
        //     cursor_tag_[0] += (double)(buffer_.last_point().x - sample_time_s_);
        //     cursor_tag_[1] += (double)(buffer_.last_point().x - sample_time_s_);
        //     // X0
        //     ImPlot::DragLineX(0, &cursor_tag_[0], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
        //     ImPlot::TagX(cursor_tag_[0], ImVec4(1,0,0,1), "%.3f", cursor_tag_[0]);
        //     // X1
        //     ImPlot::DragLineX(1, &cursor_tag_[1], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
        //     ImPlot::TagX(cursor_tag_[1], ImVec4(1,0,0,1), "%.3f", cursor_tag_[1]);
        //     // Y0
        //     ImPlot::DragLineY(2, &cursor_tag_[2], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
        //     ImPlot::TagY(cursor_tag_[2], ImVec4(1,0,0,1), "%.3f", cursor_tag_[2]);
        //     // Y1
        //     ImPlot::DragLineY(3, &cursor_tag_[3], ImVec4(1,1,1,1), 1, ImPlotDragToolFlags_NoFit);
        //     ImPlot::TagY(cursor_tag_[3], ImVec4(1,0,0,1), "%.3f", cursor_tag_[3]);
        // }
        // Hackity hack - Plot a single point when the buffer is empty.
        // Point 0 is non-existant otherwise
        if (buffer_is_empty) {
            ImVec2 dummy(0.0f, 0.0f);
            ImPlot::PlotLine(source_.c_str(), &dummy.x, &dummy.y, 1, 0, 0, 2*sizeof(float));
        } else {
            // Downsample the number of points displayed to the screen, otherwise performance can suffer on
            // slower machines when trying to display large amounts of data.
            // We just adjust the buffer stride and number of points to display.
            // In effect, we just skip buffer entries - this is not true downsampling, however the data stored in 
            // the buffer remains uneffected.
            int downsample_stride = 1;
            int displayed_points = (int)(ImPlot::GetPlotLimits().X.Size() * (double)sample_rate_);
            if (displayed_points > downsample_points_) {
                downsample_stride = displayed_points / downsample_points_;
            }

            int n_points = buffer_.data_.size() / downsample_stride;
            ImPlot::PlotLine(source_.c_str(), &buffer_.data_[0].x, &buffer_.data_[0].y, n_points , 0, buffer_.offset_, downsample_stride*2*sizeof(float));
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

    if (buffer_is_empty) {
        ImGui::EndDisabled();
    }

    ImGui::PopID();
}

int gui_host_logger::write_channels_to_file() {
    // Choose file to write to
    if (ImGui::Button("Write channel data to file")) {
        IGFD::FileDialogConfig config;
        config.path = ".";
        ImGuiFileDialog::Instance()->OpenDialog("choose_dir_key", "Choose File", ".csv", config);
    }
    // Display
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

int gui_host_logger::emit_data(std::string const& path_and_file) {
    std::cout << "NOT IMPLEMENTED YET\n";

    // TODO: Will need to add some circular buffer-esq functionality to the 
    // scrolling buffer to get this to work


    // std::cout << "Writing to: " << path_and_file << "\n";

    // std::ofstream config_file(path_and_file); 

    // config_file << "t,";
    // for (int i=0; i<channels_.size()-1; i++) {
    //     config_file << channels_[i]->source_ << ",";
    // }
    // config_file << channels_[channels_.size()-1]->source_ << "\n";



    // for (int i=0; i<channels_[0]->data_.size(); i++) {
    //     config_file << channels_[0]->x_[i] << ",";
    //     config_file << channels_[0]->data_[i] << ",";
    //     config_file << channels_[1]->data_[i] << ",";
    //     config_file << channels_[2]->data_[i] << ",";
    //     config_file << channels_[3]->data_[i] << "\n";
    // }
    return jcs::RET_OK;
}

