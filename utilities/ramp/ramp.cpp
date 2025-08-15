// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "ramp.h"
#include <math.h>

ramp::ramp(double dt) :
    state_(state::off_s),
    ramp_start_(0.0),
    ramp_final_(0.0),
    ramp_value_(0.0),
    ramp_time_(0.0),
    ramp_elapsed_(0.0),
    dwell_time_(0.0),
    dwell_elapsed_(0.0),
    settle_time_(0.0),
    settle_elapsed_(0.0),
    dt_(dt)
{}

ramp::~ramp() {}

void ramp::start(double ramp_start, double ramp_final, double ramp_time, double settle_time, double dwell_time) {
    ramp_start_ = ramp_start;
    ramp_final_ = ramp_final;
    ramp_time_  = ramp_time;
    ramp_value_ = ramp_start;
    settle_time_ = settle_time;
    dwell_time_  = dwell_time;

    ramp_elapsed_ = 0.0;
    dwell_elapsed_ = 0.0;
    settle_elapsed_ = 0.0;
    state_ = state::ramp_s;
}

float ramp::step() {
    switch (state_) {
        case state::off_s:
            break;

        case state::ramp_s:
            ramp_elapsed_ += dt_;

            if (ramp_elapsed_ >= ramp_time_) {
                // Ramp done - force final value
                ramp_value_ = ramp_final_;
                ramp_elapsed_ = ramp_time_;
                settle_elapsed_ = 0.0;
                state_ = state::settle_s;
            } else {
                // Linear interpolate
                double progress = ramp_elapsed_ / ramp_time_;
                ramp_value_ = ramp_start_ + (ramp_final_ - ramp_start_) * progress;
            }
            break;

        case state::settle_s:
            settle_elapsed_ += dt_;
            if (settle_elapsed_ >= settle_time_) {
                dwell_elapsed_ = 0.0;
                state_ = state::dwell_s;
            }
            break;

        case state::dwell_s:
            dwell_elapsed_ += dt_;
            if (dwell_elapsed_ >= dwell_time_) {
                state_ = state::off_s;
            }
            break;
    }
    return static_cast<float>(ramp_value_);
}

bool ramp::is_done() {
    return (state_ == state::off_s);
}
bool ramp::in_dwell() {
    return (state_ == state::dwell_s);
}