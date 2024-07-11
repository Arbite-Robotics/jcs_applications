// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "dwell.h"

dwell::dwell(float dt) {
    state_ = state::off_s;
    dt_ = dt;
    dwell_time_ = 0.0f;
    dwell_tick_ = 0.0f;
    value_      = 0.0f;
}

dwell::~dwell() {}

void dwell::start(float value, float settle_time, float dwell_time) {
    value_ = value;
    settle_time_ = settle_time;
    settle_tick_ = 0.0f;
    dwell_time_ = dwell_time;
    dwell_tick_ = 0.0f;

    state_ = state::settle_s;
}

float dwell::step() {
    switch (state_) {
        case state::off_s:
            break;

        case state::settle_s:
            settle_tick_ += dt_;
            if (settle_tick_ >= settle_time_) {
                dwell_tick_ = 0.0f;
                state_ = state::dwell_s;
            }
            break;

        case state::dwell_s:
            dwell_tick_ += dt_;
            if (dwell_tick_ >= dwell_time_) {
                state_ = state::off_s;
            }
            break;
    }

    return value_;
}

bool dwell::is_done() {
    return (state_ == state::off_s);
}
bool dwell::in_dwell() {
    return (state_ == state::dwell_s);
}