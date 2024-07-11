// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef ROTATE_HELPER_H_
#define ROTATE_HELPER_H_

#include <stdint.h>

class rotate {
public: 
    rotate(double dt);
    ~rotate();

    // theta_start: Starting angle
    // rotations:   Number of rotations to complete
    // direction:   Positive - incrementing theta, Negative - decrementing theta
    // precision:   Steps per rotation
    void start(double theta_start, double rotations, bool direction, double time_s);

    // Return rotational angle
    float step();

    // Return rotational speed
    float omega();

    bool is_done();

private:
    enum class state {
        off_s,
        rotating_s
    };

    state state_;
    double dt_;
    double theta_total_;
    double theta_increment_;
    double theta_;
    double omega_;
    bool direction_;
};

#endif