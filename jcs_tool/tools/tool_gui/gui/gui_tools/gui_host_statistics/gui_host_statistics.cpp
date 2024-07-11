
// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_host_statistics.h"
#include "imgui_stdlib.h"
#include "implot.h"
#include <cmath>
#include <iostream>
#include "helpers.h"

#include "jcs_dev_motor_controller.h"

gui_host_statistics::gui_host_statistics(jcs::jcs_host* host, std::string const& target_device) :
    gui_type_base("Statistics", host, target_device),
    to_mean_buffer_(2000),
    to_variance_buffer_(2000),
    cycle_buffer_(2000),
    data_exchange_buffer_(2000)
{
    t_ = 0.0f;
}

int gui_host_statistics::startup() {
    return jcs::RET_OK;
}

int gui_host_statistics::step_rt() {
    // Get stats
    timing_ = host_->statistics_timing_get();
    health_ = host_->statistics_health_get();
    return jcs::RET_OK;
}

int gui_host_statistics::render() {

    static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings;

    t_ += ImGui::GetIO().DeltaTime;

    ImGui::Text("Times");
    if (ImGui::BeginTable("Times", 2, table_flags)) {
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Cycle Time (us)");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%7.3f", (float)timing_.total_cycle_time_ns/1000.0f);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Data Exchange Time (us)");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%7.3f", (float)timing_.data_exchange_time_ns/1000.0f);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Cycle - Data Exchange Time difference (us)");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%7.3f", fabsf((float)(timing_.data_exchange_time_ns-timing_.total_cycle_time_ns)/1000.0f));

        ImGui::EndTable();
    }

    // Pretty plots
    cycle_buffer_.add_point(t_, (float)timing_.total_cycle_time_ns/1000.0f);
    data_exchange_buffer_.add_point(t_, (float)timing_.data_exchange_time_ns/1000.0f);

    static ImPlotAxisFlags plot_flags = ImPlotFlags_NoFrame | ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
    float const history = 30.0f;

    if (ImPlot::BeginPlot("Times plots")) {
        ImPlot::SetupAxes(nullptr, nullptr, plot_flags, plot_flags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_ - history, t_, ImGuiCond_Always);
        // ImPlot::SetupAxisLimits(ImAxis_X1, t_, t_, ImGuiCond_Always);
        // ImPlot::SetupAxisLimits(ImAxis_Y1, -10.0, 10.0);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        ImPlot::PlotLine("Cycle Time", &cycle_buffer_.data_[0].x, &cycle_buffer_.data_[0].y, cycle_buffer_.data_.size(), 0, cycle_buffer_.offset_, 2*sizeof(float));
        ImPlot::PlotLine("Data Exchange Time", &data_exchange_buffer_.data_[0].x, &data_exchange_buffer_.data_[0].y, data_exchange_buffer_.data_.size(), 0, data_exchange_buffer_.offset_, 2*sizeof(float));

        ImPlot::EndPlot();
    }

    ImGui::Text("Pending Sequences Overrun");
    if (ImGui::BeginTable("Pending", 4, table_flags)) {
        ImGui::TableSetupColumn("Rate",                ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Overrun count",       ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Last timestamp (us)", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Counts percentage",   ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        // Full rate
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Full Rate");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%u", (int)health_.pending_sequences_fullrate.overrun_count);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%7.3fus", (float)health_.pending_sequences_fullrate.last_timestamp_ns/1000.0f);
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%7.3f", (float)health_.pending_sequences_fullrate.counts_percent);
        // Subrate 0
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Sub rate - 0");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%u", (int)health_.pending_sequences_subrate_0.overrun_count);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%7.3fus", (float)health_.pending_sequences_subrate_0.last_timestamp_ns/1000.0f);
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%7.3f", (float)health_.pending_sequences_subrate_0.counts_percent);
        // Subrate 1
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Sub rate - 1");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%u", (int)health_.pending_sequences_subrate_1.overrun_count);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%7.3fus", (float)health_.pending_sequences_subrate_1.last_timestamp_ns/1000.0f);
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%7.3f", (float)health_.pending_sequences_subrate_1.counts_percent);
        ImGui::EndTable();
    }

    ImGui::Text("Transport Overrun");
    if (ImGui::BeginTable("Transport", 3, table_flags)) {
        ImGui::TableSetupColumn("Overrun count",       ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Last timestamp (us)", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Counts percentage",   ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("%u", (int)health_.transport.overrun_count);
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%7.3f", (float)health_.transport.last_timestamp_ns/1000.0f);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%7.3f", (float)health_.transport.counts_percent);
        ImGui::EndTable();
    }

    ImGui::Text("Thread Offset Controller Overrun");
    if (ImGui::BeginTable("Thread overrun", 3, table_flags)) {
        ImGui::TableSetupColumn("Overrun count",       ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Last timestamp (us)", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Counts percentage",   ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("%u", (int)health_.thread_offset.overrun_count);
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%7.3f", (float)health_.thread_offset.last_timestamp_ns/1000.0f);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%7.3f", (float)health_.thread_offset.counts_percent);
        ImGui::EndTable();
    }
    ImGui::Text("Thread Offset Controller Statistics");
    if (ImGui::BeginTable("Thread stats", 2, table_flags)) {
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Mean (ns)");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%7.3f", (float)health_.thread_offset.mean);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Variance (ns)");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%7.3f", (float)health_.thread_offset.variance);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);        
        ImGui::Text("Variance %% of mean");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%3.6f", ((float)health_.thread_offset.variance/(float)health_.thread_offset.mean)*100.0f);
        ImGui::EndTable();
    }

    // More pretty plots
    if (ImPlot::BeginPlot("Thread offset mean/variance")) {
        ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);

        ImPlot::SetupAxes(nullptr, nullptr, plot_flags, plot_flags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_ - history, t_, ImGuiCond_Always);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);

        to_mean_buffer_.add_point(t_, (float)health_.thread_offset.mean);

        float half_var = (float)health_.thread_offset.variance /2.0f;
        to_variance_buffer_.add_point((float)health_.thread_offset.mean-half_var, (float)health_.thread_offset.mean+half_var);

        ImPlot::PlotLine("Mean", &to_mean_buffer_.data_[0].x, &to_mean_buffer_.data_[0].y, to_mean_buffer_.data_.size(), 0, to_mean_buffer_.offset_, 2*sizeof(float));
        ImPlot::PlotShaded("Variance", &to_mean_buffer_.data_[0].x, &to_variance_buffer_.data_[0].x, &to_variance_buffer_.data_[0].y, to_variance_buffer_.data_.size(), 0, to_variance_buffer_.offset_, 2*sizeof(float));

        ImPlot::PopStyleVar();
        ImPlot::EndPlot();
    }

    return jcs::RET_OK;
}
