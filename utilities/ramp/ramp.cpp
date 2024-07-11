// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "ramp.h"

ramp::ramp(double dt) {
    state_ = state::off_s;
    dt_ = dt;
    ramp_final_ = 0.0;
    ramp_increment_ = 0.0;
    dwell_time_ = 0.0;
    settle_time_ = 0.0;

    ramp_value_ = 0.0;
    dwell_tick_ = 0.0;
    settle_tick_ = 0.0;
}

ramp::~ramp() {}

void ramp::start(double ramp_start, double ramp_final, double ramp_time, double settle_time, double dwell_time) {
    ramp_value_  = ramp_start;
    ramp_final_  = ramp_final;
    ramp_increment_ = (ramp_final_ / ramp_time) * dt_;
    settle_time_ = settle_time;
    dwell_time_  = dwell_time;

    dwell_tick_ = 0.0;
    settle_tick_ = 0.0;
    state_ = state::ramp_s;
}

float ramp::step() {
    switch (state_) {
        case state::off_s:
            break;

        case state::ramp_s:
            {
                bool ramp_done = false;
                if (ramp_value_ < ramp_final_) {
                    // Ramp up from start to final
                    ramp_value_ += ramp_increment_;
                    if (ramp_value_ >= ramp_final_) {
                        ramp_done = true;
                    }
                } else {
                    // Ramp down from start to final
                    ramp_value_ -= ramp_increment_;
                    if (ramp_value_ <= ramp_final_) {
                        ramp_done = true;
                    }
                }

                if (ramp_done) {
                    settle_tick_ = 0.0;
                    state_ = state::settle_s;
                }
            }
            break;

        case state::settle_s:
            settle_tick_ += dt_;
            if (settle_tick_ >= settle_time_) {
                dwell_tick_ = 0.0;
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

    return static_cast<float>(ramp_value_);
}

bool ramp::is_done() {
    return (state_ == state::off_s);
}
bool ramp::in_dwell() {
    return (state_ == state::dwell_s);
}