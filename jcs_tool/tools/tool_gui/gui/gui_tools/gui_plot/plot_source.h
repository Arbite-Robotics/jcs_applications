// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef PLOT_SOURCE_H_
#define PLOT_SOURCE_H_

#include <string>

class plot_source {
public:
    plot_source(std::string const& node_name, std::string const& name) : node_name_(node_name), name_(name) {}
    ~plot_source() {}

    virtual void update() = 0;

// private:
    std::string node_name_;
    std::string name_;
};

#endif