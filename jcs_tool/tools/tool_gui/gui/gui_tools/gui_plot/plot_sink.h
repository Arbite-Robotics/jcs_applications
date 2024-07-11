// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//

#ifndef PLOT_SINK_H_
#define PLOT_SINK_H_

#include <string>

class plot_sink {
public:
    plot_sink(std::string node_name, std::string name) : node_name_(node_name), name_(name) {}
    ~plot_sink() {}

    virtual void update() = 0;
// private:
    std::string node_name_;
    std::string name_;
};

#endif