// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//

#ifndef STIMULUS_H_
#define STIMULUS_H_

#include <string>

////////////////////////////////////////////////////////////////////////////
class stimulus {
public:
    stimulus(std::string const& name, double sample_rate_hz);

    std::string const& name_get() { return name_; }

    virtual void step_rt() = 0;
    virtual void render_parameters() = 0;

    void start();
    void stop();
    bool is_running();
    float value_get();

protected:
    enum class state {
        off_s,
        init_s,
        running_s
    };
    state state_;
    double dt_;
    double value_;
    std::string name_;
    bool zero_at_off_;
    bool do_recompute_;
};

////////////////////////////////////////////////////////////////////////////
class stimulus_ramp : public stimulus {
public:
    stimulus_ramp(double sample_rate_hz);
    ~stimulus_ramp() {}

    void step_rt();
    void render_parameters();

private:
    double ramp_end_;
    double ramp_start_;
    double ramp_increment_;
    double ramp_time_s_;
};

////////////////////////////////////////////////////////////////////////////
class stimulus_chirp : public stimulus {
public:
    stimulus_chirp(double sample_rate_hz);
    ~stimulus_chirp() {}

    void step_rt();
    void render_parameters();

private:
    double w0_;
    double w1_;
    double amplitude_;
    double phase_;
    double t_;
    double chirp_time_s_;
    double frequency_start_hz_;
    double frequency_end_hz_;
};

////////////////////////////////////////////////////////////////////////////
class stimulus_step : public stimulus {
public:
    stimulus_step(double sample_rate_hz);
    ~stimulus_step() {}

    void step_rt();
    void render_parameters();
private:
    enum class step_state {
        start_s,
        step_s,
        end_s
    };
    step_state step_state_;
    double start_time_s_;
    double end_time_s_;
    double t_;
    double test_value_;
};

#endif