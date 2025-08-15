// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef RAMP_HELPER_
#define RAMP_HELPER_

// Ramp with settle and dwell
// Settle to allow eg a motor rotor to settle into a position
// Dwell to eg hold at that position while collecting data
class ramp {
public:
    ramp(double dt);
    ~ramp();

    void start(double ramp_start, double ramp_final, double ramp_time, double settle_time, double dwell_time);
    float step();

    bool is_done();
    bool in_dwell();
private:
    enum class state {
        off_s,      // Not doing anything
        ramp_s,     // Ramping from ramp_start to ramp_final over ramp_time
        settle_s,   // Holding for settle_time 
        dwell_s     // Holding for dwell_time
    };

    state state_;
    double ramp_start_;
    double ramp_final_;
    double ramp_value_;
    double ramp_time_;
    double ramp_elapsed_;
    double dwell_time_;
    double dwell_elapsed_;
    double settle_time_;
    double settle_elapsed_;
    double dt_;
};

#endif