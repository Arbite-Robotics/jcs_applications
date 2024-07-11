// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef PLOT_SINK_PLOT_H_
#define PLOT_SINK_PLOT_H_

#include "plot_sink.h"
#include "helpers.h"

class plot_sink_plot : public plot_sink {
public:
    plot_sink_plot(std::string const& node_name, std::string const& name, std::string const& units, float* val_ptr);
    ~plot_sink_plot() {}

    void update();
private:
    std::string units_;
    float* val_ptr_;

    helpers::scrolling_buffer buffer_;
    float history_;
    float t_;

    float min_;
    float max_;
    // bool do_reset_y_axis_;
    // Averaging
    float ave_time_;
    float average_;
    int average_count_;
    ImPlotRange average_range_;
};

#endif