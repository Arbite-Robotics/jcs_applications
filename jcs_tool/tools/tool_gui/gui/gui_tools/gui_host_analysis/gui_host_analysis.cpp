// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_host_analysis.h"
#include "imgui_stdlib.h"
#include "implot.h"
#include <cmath>
#include <iostream>
#include "helpers.h"
#include "ImGuiFileDialog.h"

#include <ETFE.hpp>

gui_host_analysis::gui_host_analysis(jcs::jcs_host* host, std::string const& target_device) : 
    gui_type_base("Host analysis", host, target_device),
    sample_time_(1),
    sample_rate_(1000),
    plotter_(sample_time_, sample_rate_)
{
    sampler_state_ = sampler_state::off_s;
    storage_count_ = 0;
    storage_length_ = 0;
}

int gui_host_analysis::startup() {
    // Resize base rate float storage vector
    f32_osignal_store_.resize(host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0));
    f32_isignal_store_.resize(host_->sig_input_sz_unsafe_rt(jcs::signal_type::float32_s, 0));

    storage_length_ = sample_time_ * host_->base_frequency_get();
    sample_rate_ = host_->base_frequency_get();

    // Build a list of all available output float type, base rate signals
    for (int i=0; i<host_->sig_output_sz_unsafe_rt(jcs::signal_type::float32_s, 0); i++) {
        std::string node_name;
        if (host_->sig_output_node_name_get(jcs::signal_type::float32_s, 0, i, &node_name) != jcs::RET_OK) {
            std::cout << "gui_host_analysis: Error getting node name for output signal at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        std::string name;
        if (host_->sig_output_name_get(jcs::signal_type::float32_s, 0, i, &name) != jcs::RET_OK) {
            std::cout << "gui_host_analysis: Error getting output signal name at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        // Name will be node name + signal name
        f32_output_signal_names_.push_back(node_name + "::" + name);
    }

    // Update storage for new configured base rate
    plotter_.update_storage_length(sample_time_, host_->base_frequency_get());

    // Configure input stimulus
    input_stimulus_ = new gui_stimulus(static_cast<double>(host_->base_frequency_get()));

    // Build a list of all available input float type, base rate signals
    for (int i=0; i<host_->sig_input_sz_unsafe_rt(jcs::signal_type::float32_s, 0); i++) {
        std::string node_name;
        if (host_->sig_input_node_name_get(jcs::signal_type::float32_s, 0, i, &node_name) != jcs::RET_OK) {
            std::cout << "gui_host_analysis: Error getting node name for input signal at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        std::string name;
        if (host_->sig_input_name_get(jcs::signal_type::float32_s, 0, i, &name) != jcs::RET_OK) {
            std::cout << "gui_host_analysis: Error getting input signal name at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        // Name will be node name + signal name
        f32_input_signal_names_.push_back(node_name + "::" + name);
    }
    return jcs::RET_OK;
}

int gui_host_analysis::step_rt() {
    // Only currently supporting base rates
    host_->sig_output_get_rt(0, &f32_osignal_store_);

    switch (sampler_state_) {
        default:
        case sampler_state::off_s:
            break;

        case sampler_state::init_s:
            input_stimulus_->start();
            // Reset
            storage_count_ = 0;
            storage_length_ = sample_time_ * (int)host_->base_frequency_get();
            sampler_state_ = sampler_state::sampling_s;

            // Fall through
        case sampler_state::sampling_s:
            input_stimulus_->step_rt();

            // Channel 0 is input signal
            f32_isignal_store_[plotter_.source_combo_idx_y0_] = input_stimulus_->value_get();
            plotter_.y0_[storage_count_] = static_cast<double>(f32_isignal_store_[plotter_.source_combo_idx_y0_]);
            // Channel 1 is output signal
            plotter_.y1_[storage_count_] = static_cast<double>(f32_osignal_store_[plotter_.source_combo_idx_y1_]);

            storage_count_++;
            if (storage_count_ >= storage_length_) {
                storage_count_ = 0;
                sampler_state_ = sampler_state::done_s;
            }
            host_->sig_input_set_rt(0, f32_isignal_store_);
            break;

        case sampler_state::done_s:
            sampler_state_ = sampler_state::off_s;
            break;
    }
    return jcs::RET_OK;
}

int gui_host_analysis::render() {
    render_status();
    ImGui::Separator();
    if (render_interface() != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }
    ImGui::Separator();
    render_plot();
    render_analysis();
    return jcs::RET_OK;
}

void gui_host_analysis::render_status() {
    ImGui::Text("Analysis State: ");
    ImGui::SameLine();
    switch (sampler_state_) {
        default:
        case sampler_state::off_s:
        case sampler_state::done_s:
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.0f, 1.0f), "Off");
            break;

        case sampler_state::init_s:
        case sampler_state::sampling_s:
            ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.0f, 1.0f), "Sampling");
            break;
    }
}

int gui_host_analysis::render_plot() {
    plotter_.plot();
    return jcs::RET_OK;
}

int gui_host_analysis::render_interface() {
    ImGui::Text("Analysis Settings");

    ImGuiInputTextFlags input_text_flags = ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll;

    // Storage length
    ImGui::Text("Base sample frequency: %u", host_->base_frequency_get());
    {
        int sample_temp = sample_time_;
        if (ImGui::InputInt("Sample time (s)", &sample_temp, 1, 100, input_text_flags)) {
            sample_time_ = sample_temp;
            storage_length_ = sample_time_ * host_->base_frequency_get();
            plotter_.update_storage_length(sample_time_, host_->base_frequency_get());
        }
    }

    // Channel Source
    helpers::combo_select("Input Source",  &f32_input_signal_names_,  &plotter_.source_combo_idx_y0_, &plotter_.source_y0_);
    helpers::combo_select("Output Source", &f32_output_signal_names_, &plotter_.source_combo_idx_y1_, &plotter_.source_y1_);

    ImGui::Separator();

    input_stimulus_->render_parameters();

    if (ImGui::Button("Start stimulus")) {
        sampler_state_ = sampler_state::init_s;
    }
    ImGui::SameLine();
    if (ImGui::Button("Abort")) {
        sampler_state_ = sampler_state::off_s;
    }

    ImGui::Separator();
    // Write data out
    write_channels_to_file();

    return jcs::RET_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 2 channel plot
gui_host_analysis::plotter::plotter(int const sample_time_s, int const sample_rate_hz) :
    source_combo_idx_y0_(0),
    source_combo_idx_y1_(0),
    plot_cursors_(false)
{
    int buffer_size = sample_time_s * sample_rate_hz;
    x_.resize(buffer_size);
    y0_.resize(buffer_size);
    y1_.resize(buffer_size);
    for (int i=0; i<4; i++) {
        cursor_tag_[i] = 0.0;
    }
    update_sample_rate(sample_rate_hz);
}

void gui_host_analysis::plotter::plot() {
    ImGui::PushID("plotter2ch##");

    ImGui::Checkbox("Show measurement cursors", &plot_cursors_);
    ImGui::SameLine();
    ImGui::Text("(Double click plot to zoom to extents. Right click plot for menu.)");

    if (ImPlot::BeginPlot("Input/Output##")) {
        ImPlot::SetupAxes("Time (s)", "Signal");

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
        ImPlot::PlotLine("Input", &x_[0], &y0_[0], x_.size());
        ImPlot::PlotLine("Output", &x_[0], &y1_[0], x_.size());
        ImPlot::EndPlot();
    }

    if (plot_cursors_) {
        static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | 
                                             ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings;

        if (ImGui::BeginTable("Measurements", 4, table_flags)) {
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("delta T");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.6f", cursor_tag_[0] - cursor_tag_[1]);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("Hz");
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.6f", 1.0f/(cursor_tag_[0] - cursor_tag_[1]));

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("delta Signal");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.6f", cursor_tag_[2] - cursor_tag_[3]);
            ImGui::TableSetColumnIndex(2);
            ImGui::TableSetColumnIndex(3);
            ImGui::EndTable();
        }
    }
    ImGui::PopID();
}

void gui_host_analysis::plotter::update_sample_rate(int sample_rate_hz) {
    for (int i=0; i<x_.size(); i++) {
        x_[i] = (float)i / (float)sample_rate_hz;
    }
}

void gui_host_analysis::plotter::update_storage_length(int sample_time_s, int sample_rate_hz) {
    int new_sample_length = sample_time_s * sample_rate_hz;
    x_.resize(new_sample_length);
    y0_.resize(new_sample_length);
    y1_.resize(new_sample_length);
    update_sample_rate(sample_rate_hz);
}

int gui_host_analysis::write_channels_to_file() {
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

int gui_host_analysis::emit_data(std::string const& path_and_file) {
    std::cout << "Writing to: " << path_and_file << "\n";

    std::ofstream config_file(path_and_file); 

    config_file << "t,";
    config_file << plotter_.source_y0_ << ",";
    config_file << plotter_.source_y1_ << "\n";

    for (int i=0; i<plotter_.x_.size(); i++) {
        config_file << plotter_.x_[i] << ",";
        config_file << plotter_.y0_[i] << ",";
        config_file << plotter_.y1_[i] << "\n";
    }
    return jcs::RET_OK;
}

void gui_host_analysis::render_analysis() {

    // This is swiped almost verbatim from implot_demos:
    // https://github.com/epezent/implot_demos

    // perform ETFE
    int N = plotter_.y0_.size();
    double Fs = static_cast<double>(sample_rate_);
    int fs_on_2 = sample_rate_ / 2;
    static int window         = 0;
    static int inwindow        = 4;
    static int nwindow_opts[] = {100, 200, 500, 1000, 2000, 5000, 10000};
    static int infft          = 4;
    static int nfft_opts[]    = {100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000};
    static float overlap      = 0.5f;

    static double Fc[] = {100,100};
    static bool etfe_need_update = true;

    static etfe::ETFE etfe(N, Fs, etfe::hamming(nwindow_opts[inwindow]), nwindow_opts[inwindow]/2, nfft_opts[infft]);        

    ImGui::Text("Frequency Response");
    ImGui::Separator();
    if (ImGui::Combo("FFT Size", &infft, "100\0""200\0""500\0""1000\0""2000\0""5000\0""10000\0""20000\0""50000\0")) {
        inwindow = infft < inwindow ? infft : inwindow;
        etfe_need_update = true;
    }
    if (ImGui::Combo("Window Type", &window,"hamming\0hann\0winrect\0")) {
        etfe_need_update = true;
    }
    if (ImGui::Combo("Window Size", &inwindow,"100\0""200\0""500\0""1000\0""2000\0""5000\0""10000\0")) {
        etfe_need_update = true;
    }
    if (ImGui::SliderFloat("Window Overlap",&overlap,0,1,"%.2f")) {
        etfe_need_update = true;
    }

    if (ImGui::Button("Recompute")) {
        etfe_need_update = true;
    }

    ImGui::NewLine();
    
    if (etfe_need_update) {
        infft = inwindow > infft ? inwindow : infft;
        int nwindow  = nwindow_opts[inwindow];
        int noverlap = (int)(nwindow * overlap);
        int nfft     = nfft_opts[infft];
        etfe.setup(N,Fs,window == 0 ? etfe::hamming(nwindow) : window == 1 ? etfe::hann(nwindow) : etfe::winrect(nwindow), noverlap, nfft);
    }

    if (etfe_need_update || sampler_state_ == sampler_state::sampling_s) {
        etfe.estimate(&plotter_.y0_[0], &plotter_.y1_[0]);
        etfe_need_update = false;
    }        

    // Hackity hacks...
    const etfe::ETFE::Result& result = etfe.getResult();

    if (ImGui::BeginTabBar("Plots")) {
        if (ImGui::BeginTabItem("Magnitude")) {
            static const double co = -3;
            if (ImPlot::BeginPlot("##Bode1",ImVec2(-1,-1))) {
                ImPlot::SetupAxesLimits(1, fs_on_2, -100, 10);
                ImPlot::SetupAxes("Frequency [Hz]","Magnitude [dB]");
                ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
                ImPlot::SetNextLineStyle({1,1,1,1});
                ImPlot::PlotInfLines("##3dB",&co,1,ImPlotInfLinesFlags_Horizontal);
                ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.250f);
                ImPlot::PlotShaded("##Mag1",result.f.data(),result.mag.data(),(int)result.f.size(),-INFINITY);
                ImPlot::PlotLine("##Mag2",result.f.data(),result.mag.data(),(int)result.f.size());
                ImPlot::Annotation(Fc[0],-3,ImVec4(0.15f,0.15f,0.15f,1),ImVec2(5,-5),true,"Half-Power Point");
                if (ImPlot::DragLineX(148884,&Fc[0],ImVec4(0.15f,0.15f,0.15f,1))) {
                    // filt_need_update = true;
                }
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }            
        if (ImGui::BeginTabItem("Phase")) {
            if (ImPlot::BeginPlot("##Bode2",ImVec2(-1,-1))) {  
                ImPlot::SetupAxesLimits(1, fs_on_2, -180, 10);
                ImPlot::SetupAxes("Frequency [Hz]","Phase Angle [deg]");
                ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
                ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.250f);
                ImPlot::PlotShaded("##Phase1",result.f.data(),result.phase.data(),(int)result.f.size(),-INFINITY);
                ImPlot::PlotLine("##Phase2",result.f.data(),result.phase.data(),(int)result.f.size());
                if (ImPlot::DragLineX(439829,&Fc[0],ImVec4(0.15f,0.15f,0.15f,1))) {
                    // filt_need_update = true;
                }
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }   
        if (ImGui::BeginTabItem("Amplitude")) {
            if (ImPlot::BeginPlot("##Amp",ImVec2(-1,-1))) {
                ImPlot::SetupAxesLimits(1, fs_on_2, 0, 1.0);
                ImPlot::SetupAxes("Frequency [Hz]","Amplitude [dB]");
                ImPlot::SetupLegend(ImPlotLocation_NorthEast);
                ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.25f);
                ImPlot::PlotShaded("x(f)",result.f.data(),result.ampx.data(),(int)result.f.size(),-INFINITY);
                ImPlot::PlotLine("x(f)",result.f.data(),result.ampx.data(),(int)result.f.size());
                ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.25f);
                ImPlot::PlotShaded("y(f)",result.f.data(),result.ampy.data(),(int)result.f.size(),-INFINITY);
                ImPlot::PlotLine("y(f)",result.f.data(),result.ampy.data(),(int)result.f.size());
                if (ImPlot::DragLineX(397391,&Fc[0],ImVec4(0.15f,0.15f,0.15f,1))) {
                    // filt_need_update = true;
                }

                // if (ImPlot::DragLineY(939031,&a[0],ImVec4(0.15f,0.15f,0.15f,1)))
                    // signal_need_update = true;                    
                // if (ImPlot::DragLineY(183853,&a[1],ImVec4(0.15f,0.15f,0.15f,1)))
                    // signal_need_update = true;
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Power")) {
            static std::vector<double> pxx10, pyy10;
            pxx10.resize(result.pxx.size());
            pyy10.resize(result.pyy.size());
            for (int i = 0; i < pxx10.size(); ++i) {
                pxx10[i] = 10*std::log10(result.pxx[i]);
                pyy10[i] = 10*std::log10(result.pyy[i]);
            }
            if (ImPlot::BeginPlot("##Power",ImVec2(-1,-1))) {
                ImPlot::SetupAxesLimits(1, fs_on_2, -100, 0);
                ImPlot::SetupAxes("Frequency [Hz]","Power Spectral Density (dB/Hz)");
                ImPlot::SetupLegend(ImPlotLocation_NorthEast);
                ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.25f);
                ImPlot::PlotShaded("x(f)",result.f.data(),pxx10.data(),(int)result.f.size(),-INFINITY);
                ImPlot::PlotLine("x(f)",result.f.data(),pxx10.data(),(int)result.f.size());
                ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.25f);
                ImPlot::PlotShaded("y(f)",result.f.data(),pyy10.data(),(int)result.f.size(),-INFINITY);
                ImPlot::PlotLine("y(f)",result.f.data(),pyy10.data(),(int)result.f.size());

                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

