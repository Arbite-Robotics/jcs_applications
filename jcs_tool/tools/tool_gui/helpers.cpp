// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "helpers.h"
#include "jcs_user_external.h"
#include <cmath>

// Helper to display a little (?) mark which shows a tooltip when hovered.
// In your own code you may want to display an actual icon if you are using a merged icon fonts (see docs/FONTS.md)
void helpers::HelpMarker(const char* desc) {
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip())
    {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void helpers::combo_select(std::string const& name, std::vector<std::string> const* sources, int* current_idx, std::string* dest) {
    const char* combo_preview_value = sources->at(*current_idx).c_str();

    ImGui::PushID(name.c_str());
    if (ImGui::BeginCombo(name.c_str(), combo_preview_value, 0)) {
        for (int i = 0; i < sources->size(); ++i) {
            const bool is_selected = (*current_idx == i);
            if (ImGui::Selectable(sources->at(i).c_str(), is_selected)) {
                *current_idx = i;
            }
            // Set the initial focus when opening the combo
            // (scrolling + keyboard navigation focus)
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        if (dest != nullptr) {
            *dest = sources->at(*current_idx);
        }
        // std::cout << "Got " << name << " " << *dest << "\n";
        ImGui::EndCombo();
    }
    ImGui::PopID();
}

// Normalise [-pi, pi]
double helpers::angle_norm_pipi(double angle) {
    double angle_norm = angle;
    while (angle_norm > M_PI)  { angle_norm -= 2.0*M_PI; }
    while (angle_norm < -M_PI) { angle_norm += 2.0*M_PI; }
    return angle_norm;
}

// Normalise [0, 2pi]
double helpers::angle_norm_2pi(double angle) {
    double angle_norm = angle;
    while (angle_norm > 2.0*M_PI){ angle_norm -= 2.0*M_PI; }
    while (angle_norm < 0.0)     { angle_norm += 2.0*M_PI; }
    return angle_norm;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void helpers::sleep_ms(long ms) {
    jcs::external::sleep_us(ms * 1e3);
}

long int helpers::time_now_ms() {
    return jcs::external::time_now_ns() / 1e6;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Nice plotting helper
helpers::plot_measurement::plot_measurement(std::string const& name, std::string const& x_name, std::string const& y_name, int const size) :
    name_(name), x_name_(x_name), y_name_(y_name), size_(size), plot_cursors_(false)
{
    x_.resize(size_);
    y_.resize(size_);
    for (int i=0; i<4; i++) {
        cursor_tag_[i] = 0.0;
    }
}
helpers::plot_measurement::plot_measurement(std::string const& name, std::string const& x_name, std::string const& y_name, int const sample_time_s, int const sample_rate_hz) :
    plot_measurement(name, x_name, y_name, sample_time_s * sample_rate_hz)
{
    update_sample_rate(sample_rate_hz);
}

void helpers::plot_measurement::plot() {
    ImGui::PushID(name_.c_str());

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
        ImPlot::PlotLine(name_.c_str(), &x_[0], &y_[0], x_.size());
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
            ImGui::EndTable();
        }
    }
    ImGui::PopID();
}

void helpers::plot_measurement::update_sample_rate(int sample_rate_hz) {
    for (int i=0; i<x_.size(); i++) {
        x_[i] = (float)i / (float)sample_rate_hz;
    }
}

void helpers::plot_measurement::update_storage_length(int sample_time_s, int sample_rate_hz) {
    int new_sample_length = sample_time_s * sample_rate_hz;
    x_.resize(new_sample_length);
    y_.resize(new_sample_length);
    update_sample_rate(sample_rate_hz);
}