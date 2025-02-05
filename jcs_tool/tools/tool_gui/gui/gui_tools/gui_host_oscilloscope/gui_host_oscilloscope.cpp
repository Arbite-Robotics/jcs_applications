// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_host_oscilloscope.h"
#include "imgui_stdlib.h"
#include "implot.h"
#include <cmath>
#include <iostream>
#include "helpers.h"
#include "ImGuiFileDialog.h"

gui_host_oscilloscope::gui_host_oscilloscope(jcs::jcs_host* host, std::string const& target_device) : 
    gui_type_base("Host Oscilloscope", host, target_device)
{
    sampler_state_ = sampler_state::off_s;
    storage_count_ = 0;
    storage_length_ = 0;
    // Default sample time is 1 sec
    sample_time_ = 1;
    input_stim_combo_idx_ = 0;
    trigger_.reset();
    for (int i=0; i<channels_.size(); i++) {
        channels_[i] = new channel("Channel " + std::to_string(i),
                                   "None",
                                   1,
                                   1000);
    }
}

int gui_host_oscilloscope::startup() {
    // Resize base rate float storage vector
    f32_osignal_store_.resize(host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0));
    f32_isignal_store_.resize(host_->sig_input_sz_unsafe_rt(jcs::signal_type::float32_s, 0));

    storage_length_ = sample_time_ * host_->base_frequency_get();

    // Build a list of all available output float type, base rate signals
    for (int i=0; i<host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0); i++) {
        std::string node_name;
        if (host_->sig_output_node_name_get(jcs::signal_type::float32_s, 0, i, &node_name) != jcs::RET_OK) {
            std::cout << "gui_host_oscilloscope: Error getting node name for output signal at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        std::string name;
        if (host_->sig_output_name_get(jcs::signal_type::float32_s, 0, i, &name) != jcs::RET_OK) {
            std::cout << "gui_host_oscilloscope: Error getting output signal name at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        // Name will be node name + signal name
        f32_output_signal_names_.push_back(node_name + "::" + name);
    }

    // Update storage for new configured base rate
    for (int ch=0; ch<channels_.size(); ch++) {
        channels_[ch]->update_storage_length(sample_time_, host_->base_frequency_get());
    }

    // Configure input stimulus
    input_stimulus_ = new gui_stimulus(static_cast<double>(host_->base_frequency_get()));
    input_stim_combo_idx_ = 0;

    // Build a list of all available input float type, base rate signals
    for (int i=0; i<host_->sig_input_sz_unsafe_rt(jcs::signal_type::float32_s, 0); i++) {
        std::string node_name;
        if (host_->sig_input_node_name_get(jcs::signal_type::float32_s, 0, i, &node_name) != jcs::RET_OK) {
            std::cout << "gui_host_oscilloscope: Error getting node name for input signal at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        std::string name;
        if (host_->sig_input_name_get(jcs::signal_type::float32_s, 0, i, &name) != jcs::RET_OK) {
            std::cout << "gui_host_oscilloscope: Error getting input signal name at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        // Name will be node name + signal name
        f32_input_signal_names_.push_back(node_name + "::" + name);
    }

    return jcs::RET_OK;
}

int gui_host_oscilloscope::step_rt() {

    // Only currently supporting base rates
    host_->sig_output_get_rt(0, &f32_osignal_store_);

    switch (sampler_state_) {
        default:
        case sampler_state::off_s:
            break;

        case sampler_state::waiting_trigger_s:
            // if (fabsf(f32_signal_store_[trigger_source_combo_idx_]) < trigger_level_) {
            //     break;
            // }
            if (!step_rt_has_trigger()) {
                break;
            }
            // Reset
            storage_count_ = 0;
            storage_length_ = sample_time_ * (int)host_->base_frequency_get();
            sampler_state_ = sampler_state::sampling_s;

            // Fall through
        case sampler_state::sampling_s:
            step_rt_sources();
            // Sampling
            for (int i=0; i<channels_.size(); i++) {
                channels_[i]->data_[storage_count_] = f32_osignal_store_[channels_[i]->source_combo_idx_];
            }

            storage_count_++;
            if (storage_count_ >= storage_length_) {
                storage_count_ = 0;
                sampler_state_ = sampler_state::done_s;
            }

            // Output depending on the trigger type
            switch (trigger_.type) {
                default:
                case control_type::output_signal_trigger_s:
                // case control_type::input_signal_trigger_s:
                case control_type::start_manual_s:
                    break;

                case control_type::input_stimulus_s:
                    host_->sig_input_set_rt(0, f32_isignal_store_);
                    break;
            }
            break;

        case sampler_state::done_s:
            break;
    }


    return jcs::RET_OK;
}

int gui_host_oscilloscope::step_rt_always() {
    return jcs::RET_OK;
}

int gui_host_oscilloscope::render() {
    render_status();
    ImGui::Separator();
    if (render_interface() != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }
    ImGui::Separator();
    return render_plot();
}

void gui_host_oscilloscope::render_status() {
    ImGui::Text("Oscilloscope State: ");
    ImGui::SameLine();
    switch (sampler_state_) {
        default:
        case sampler_state::off_s:
        case sampler_state::done_s:
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.0f, 1.0f), "Off");
            break;

        case sampler_state::waiting_trigger_s:
            ImGui::TextColored(ImVec4(0.0f, 0.5f, 0.0f, 1.0f), "Waiting Trigger");
            break;

        case sampler_state::sampling_s:
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "Sampling");
            break;
    }
}

int gui_host_oscilloscope::render_plot() {
    for (int i=0; i<channels_.size(); i++) {
        channels_[i]->plot(); ImGui::Separator();
    }
    return jcs::RET_OK;
}

int gui_host_oscilloscope::render_interface() {
    ImGui::Text("Oscilloscope Settings");

    ImGuiInputTextFlags input_text_flags = ImGuiInputTextFlags_EscapeClearsAll;

    // Storage length
    ImGui::Text("Base sample frequency: %u", host_->base_frequency_get());
    {
        int sample_temp = sample_time_;
        if (ImGui::InputInt("Sample time (s)", &sample_temp, 1, 100, input_text_flags)) {
            sample_time_ = sample_temp;
            storage_length_ = sample_time_ * host_->base_frequency_get();
            for (int ch=0; ch<channels_.size(); ch++) {
                channels_[ch]->update_storage_length(sample_time_, host_->base_frequency_get());
            }
        }
    }

    // Channel Source
    for (int i=0; i<channels_.size(); i++) {
        helpers::combo_select("Channel " + std::to_string(i) + " Source", &f32_output_signal_names_, &channels_[i]->source_combo_idx_, &channels_[i]->source_);
    }

    ImGui::Separator();

    ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
    if (ImGui::BeginTabBar("Control_sources", tab_bar_flags)) {
        ImGuiInputTextFlags input_text_flags = ImGuiInputTextFlags_EscapeClearsAll;
        
        if (ImGui::BeginTabItem("Trigger Output Signal")) {
            helpers::combo_select("Trigger Source", &f32_output_signal_names_, &trigger_.osig_source_combo_idx, nullptr);
            {
                float trigger_temp = trigger_.level;
                if (ImGui::InputFloat("Trigger Level", &trigger_temp, 0.1f, 1.0f, "%.3f", input_text_flags)) {
                    trigger_.level = trigger_temp;
                }
            }
            ImGui::Text("Note: Trigger will trigger off absolute value of trigger source.");
            trigger_.type = control_type::output_signal_trigger_s;
            if (ImGui::Button("Wait For Trigger")) {
                sampler_state_ = sampler_state::waiting_trigger_s;
            }
            ImGui::SameLine();
            if (ImGui::Button("Abort")) {
                sampler_state_ = sampler_state::off_s;
            }
            ImGui::EndTabItem();
        }
        
        // if (ImGui::BeginTabItem("Trigger Input Signal")) {
        //     helpers::combo_select("Trigger Source", &f32_input_signal_names_, &trigger_.isig_source_combo_idx, nullptr);
        //     {
        //         float trigger_temp = trigger_.level;
        //         if (ImGui::InputFloat("Trigger Level", &trigger_temp, 0.1f, 1.0f, "%.3f", input_text_flags)) {
        //             trigger_.level = trigger_temp;
        //         }
        //     }
        //     ImGui::Text("Note: Trigger will trigger off absolute value of trigger source.");
        //     trigger_.type = control_type::input_signal_trigger_s;
        //     if (ImGui::Button("Wait For Trigger")) {
        //         sampler_state_ = sampler_state::waiting_trigger_s;
        //     }
        //     ImGui::SameLine();
        //     if (ImGui::Button("Abort")) {
        //         sampler_state_ = sampler_state::off_s;
        //     }
        //     ImGui::EndTabItem();
        // }

        if (ImGui::BeginTabItem("Input Signal Stimulus")) {
            input_stimulus_->render_parameters();
            helpers::combo_select("Stimulus signal input", &f32_input_signal_names_, &input_stim_combo_idx_, nullptr);
            trigger_.type = control_type::input_stimulus_s;
            if (ImGui::Button("Start stimulus")) {
                sampler_state_ = sampler_state::waiting_trigger_s;
            }
            ImGui::SameLine();
            if (ImGui::Button("Abort")) {
                sampler_state_ = sampler_state::off_s;
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Manual")) {
            trigger_.type = control_type::start_manual_s;
            if (ImGui::Button("Start sampling")) {
                sampler_state_ = sampler_state::waiting_trigger_s;
            }
            ImGui::SameLine();
            if (ImGui::Button("Abort")) {
                sampler_state_ = sampler_state::off_s;
            }
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }

    ImGui::Separator();
    write_channels_to_file();
    return jcs::RET_OK;
}

gui_host_oscilloscope::channel::channel(std::string const& name, std::string const& source, int const sample_time_s, int const sample_rate_hz) :
    name_(name),
    source_(source),
    source_combo_idx_(0),
    data_(sample_time_s * sample_rate_hz),
    x_(sample_time_s * sample_rate_hz),
    plot_cursors_(false)
{
    for (int c=0; c<4; c++) {
        cursor_tag_[c] = 0.0;
    }
    update_sample_rate(sample_time_s, sample_rate_hz);
}

void gui_host_oscilloscope::channel::update_storage_length(int sample_time_s, int sample_rate_hz) {
    int new_sample_length = sample_time_s * sample_rate_hz;
    data_.resize(new_sample_length);
    x_.resize(new_sample_length);
    update_sample_rate(sample_time_s, sample_rate_hz);
}


void gui_host_oscilloscope::channel::plot() {
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

void gui_host_oscilloscope::channel::update_sample_rate(int sample_time_s, int sample_rate_hz) {
    (void)sample_time_s;
    for (int i=0; i<x_.size(); i++) {
        x_[i] = (float)i / (float)sample_rate_hz;
    }
}

int gui_host_oscilloscope::write_channels_to_file() {
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

int gui_host_oscilloscope::emit_data(std::string const& path_and_file) {
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

gui_host_oscilloscope::trigger::trigger() {
    osig_source_combo_idx = 0;
    isig_source_combo_idx = 0;
}

void gui_host_oscilloscope::trigger::reset() {
    type = control_type::output_signal_trigger_s;
    level = 0.0;
}

bool gui_host_oscilloscope::step_rt_has_trigger() {
    switch (trigger_.type) {
        default:
        case control_type::output_signal_trigger_s:
            if (fabsf(f32_osignal_store_[trigger_.osig_source_combo_idx]) < trigger_.level) {
                return true;
            }
            break;
        // case control_type::input_signal_trigger_s:
        //     if (fabsf(f32_isignal_store_[trigger_.osig_source_combo_idx]) < trigger_.level) {
        //         return true;
        //     }
        //     break;
        case control_type::input_stimulus_s:
            input_stimulus_->start();
            return true;
        case control_type::start_manual_s:
            return true;
    }
    // Not triggered
    return false;
}


void gui_host_oscilloscope::step_rt_sources() {
    switch (trigger_.type) {
        default:
        case control_type::output_signal_trigger_s:
        // case control_type::input_signal_trigger_s:
        case control_type::start_manual_s:
            break;

        case control_type::input_stimulus_s:
            input_stimulus_->step_rt();
            f32_isignal_store_[input_stim_combo_idx_] = input_stimulus_->value_get();
            break;
    }
}