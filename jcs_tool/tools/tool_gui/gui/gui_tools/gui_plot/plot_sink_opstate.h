// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef PLOT_SINK_OPSTATE_H_
#define PLOT_SINK_OPSTATE_H_

#include "plot_sink.h"
#include <cstdint>

class plot_sink_opstate : public plot_sink {
public:
    plot_sink_opstate(std::string const& node_name, std::string const& name, uint8_t* val_ptr);
    ~plot_sink_opstate() {}

    void update();
private:
    uint8_t* val_ptr_;
};


#endif