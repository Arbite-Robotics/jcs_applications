// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef PLOT_SINK_int_H_
#define PLOT_SINK_int_H_

#include "plot_sink.h"
#include "helpers.h"
#include <cstdint>

class plot_sink_int : public plot_sink {
public:
    enum width {
        uint_8,
        uint_16,
        uint_32,
    };

    plot_sink_int(std::string const& node_name, std::string const& name, uint32_t* val_ptr);
    plot_sink_int(std::string const& node_name, std::string const& name, uint16_t* val_ptr);
    plot_sink_int(std::string const& node_name, std::string const& name, uint8_t* val_ptr);

    ~plot_sink_int() {}

    void update();
private:
    void reset();
    width bit_width_;
    uint32_t* val_prt_u32_;
    uint16_t* val_prt_u16_;
    uint8_t*  val_prt_u8_;

    helpers::scrolling_buffer buffer_;
    float history_;
    float t_;

    float min_;
    float max_;
};

#endif