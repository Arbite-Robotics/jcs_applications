// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
//
#include "rotate.h"
#include <cmath>

rotate::rotate(double dt) {
    state_= state::off_s;
    dt_ = dt;
    theta_total_ = 0.0;
    theta_increment_ = 0.0;
    direction_ = true;
    theta_ = 0.0;
}

rotate::~rotate() {}

void rotate::start(double theta_start, double rotations, bool direction, double time_s) {
    theta_ = theta_start;
    theta_total_ = theta_start + (2.0 * M_PI * rotations);
    theta_increment_ = rotations * ((2.0 * M_PI) / time_s) * dt_;
    omega_ = ((2.0 * M_PI) / time_s);
    state_ = state::rotating_s;
}

float rotate::step() {
    double theta_out = 0.0f;

    switch (state_) {
        default:
        case state::off_s:
            break;

        case state::rotating_s:
            if (direction_) {
                theta_ += theta_increment_;
            } else {
                theta_ -= theta_increment_;
            }

            if (fabs(theta_) >= theta_total_) {
                state_ = state::off_s;
            }
            
            theta_out = theta_;
            // Wrap [-pi, pi]
            while (theta_out > M_PI) {
                theta_out -= (2.0 * M_PI);
            }
            while (theta_out < -M_PI) {
                theta_out += (2.0 * M_PI);
            }
            break;
    }

    return static_cast<float>(theta_out);
}

bool rotate::is_done() {
    return (state_ == state::off_s);
}

float rotate::omega() {
    return static_cast<float>(omega_);
}