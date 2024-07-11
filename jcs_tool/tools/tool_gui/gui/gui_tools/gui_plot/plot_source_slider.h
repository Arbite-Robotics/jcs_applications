// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef PLOT_SOURCE_SLIDER_H_
#define PLOT_SOURCE_SLIDER_H_

#include "plot_source.h"

class plot_source_slider : public plot_source {
public:
    plot_source_slider(std::string const& node_name, std::string const& name, std::string const& units, float const max, float const min, float* val_ptr) :
        plot_source(node_name, name), units_(units), max_(max), min_(min), val_ptr_(val_ptr) {}
    ~plot_source_slider() {}

    void update();
private:
    std::string units_;
    float* val_ptr_;
    float min_;
    float max_;
};


#endif