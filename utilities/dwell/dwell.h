// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef DWELL_HELPER_
#define DWELL_HELPER_

// Dwell at some value for some time
class dwell {
public:
    dwell(float dt);
    ~dwell();

    void start(float value, float settle_time, float dwell_time);
    float step();

    bool is_done();
    bool in_dwell();
private:
    enum class state {
        off_s,      // Not doing anything
        settle_s,   // Holding for settle_time 
        dwell_s     // Holding for dwell_time
    };

    state state_;
    float value_;
    float dwell_time_;
    float dwell_tick_;
    float settle_time_;
    float settle_tick_;
    float dt_;
};

#endif