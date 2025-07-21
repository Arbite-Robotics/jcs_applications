// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "stimulus.h"
#include "imgui.h"
#include "helpers.h"
#include <math.h>

#include <iostream>

////////////////////////////////////////////////////////////////////////////
stimulus::stimulus(std::string const& name, double sample_rate_hz) :
    name_(name)
{
    state_ = state::off_s;
    value_ = 0.0;
    zero_at_off_ = true;
    dt_ = 1.0 / sample_rate_hz;
    do_recompute_ = true;
}

void stimulus::start() {
    state_ = state::init_s;
}
void stimulus::stop() {
    // Call zero at off here too.
    // Higher level may transition not call step_rt once stop has been called....
    if (zero_at_off_) {
        value_ = 0.0;
    }
    state_ = state::off_s;
}
bool stimulus::is_running() {
    return (state_ == state::running_s) || (state_ == state::init_s);
}
float stimulus::value_get() {
    return static_cast<float>(value_);
}

////////////////////////////////////////////////////////////////////////////
stimulus_ramp::stimulus_ramp(double sample_rate_hz) : 
    stimulus("Ramp", sample_rate_hz)
{
    ramp_end_ = 0.0;
    ramp_start_ = 0.0;
    ramp_increment_ = 0.0;
    ramp_time_s_ = 0.0;
}

void stimulus_ramp::render_parameters() {
    const double f64_one = 1.0;

    ImGui::PushID(this);

    if (ImGui::InputScalar("Ramp start value", ImGuiDataType_Double, &ramp_start_, &f64_one)) { do_recompute_ = true; }
    if (ImGui::InputScalar("Ramp end value",   ImGuiDataType_Double, &ramp_end_, &f64_one))   { do_recompute_ = true; }

    if (ImGui::InputScalar("Ramp time (s)",    ImGuiDataType_Double, &ramp_time_s_, &f64_one)) { do_recompute_ = true;}

    ImGui::Checkbox("Zero at ramp off", &zero_at_off_);

    ImGui::PopID();

    if (do_recompute_) {
        ramp_increment_ = ( (ramp_end_ - ramp_start_) / ramp_time_s_) * dt_;
        do_recompute_ = false;
    }
}

void stimulus_ramp::step_rt() {
    switch (state_) {
        default:
        case state::off_s:
            if (zero_at_off_) {
                value_ = 0.0;
            }
            break;

        case state::init_s:
            value_ = ramp_start_;
            state_ = state::running_s;
            break;

        case state::running_s:
            if (value_ < ramp_end_) {
                // Ramping up to final value
                value_ += ramp_increment_;
                if (value_ >= ramp_end_) {
                    // Done
                    state_ = state::off_s;
                }
            } else {
                // Ramping down to final value
                value_ -= ramp_increment_;
                if (value_ <= ramp_end_) {
                    // Done
                    state_ = state::off_s;
                }
            }
            break;
    }
}

////////////////////////////////////////////////////////////////////////////
// Some inspiration
// https://github.com/libsndfile/sndfile-tools/blob/master/src/generate-chirp.c
// https://electronics.stackexchange.com/questions/151664/how-can-i-implement-chirp-signal-using-c
stimulus_chirp::stimulus_chirp(double sample_rate_hz) : 
    stimulus("Chirp", sample_rate_hz)
{
    w0_ = 0.0;
    w1_ = 0.0;
    amplitude_ = 1.0;
    t_ = 0.0;
    chirp_time_s_ = 1.0;
    // dt_ = chirp_time_s_ / sample_rate_hz;
    phase_ = 0.0;

    frequency_start_hz_ = 0.0;
    frequency_end_hz_ = 10.0;
}

void stimulus_chirp::render_parameters() {
    const double f64_one = 1.0;

    ImGui::PushID(this);

    if (ImGui::InputScalar("Frequency start (Hz)", ImGuiDataType_Double, &frequency_start_hz_, &f64_one)) { do_recompute_ = true; }
    if (ImGui::InputScalar("Frequency end (Hz)",   ImGuiDataType_Double, &frequency_end_hz_, &f64_one))   { do_recompute_ = true; }

    ImGui::InputScalar("Amplitude",                ImGuiDataType_Double, &amplitude_, &f64_one);

    if (ImGui::InputScalar("Chirp time (s)",       ImGuiDataType_Double, &chirp_time_s_, &f64_one)) { do_recompute_ = true;}

    ImGui::Checkbox("Zero at chirp off", &zero_at_off_);

    ImGui::PopID();

    if (do_recompute_) {
        w0_ = 2.0 * M_PI * frequency_start_hz_ * dt_;
        w1_ = 2.0 * M_PI * frequency_end_hz_ * dt_;
        do_recompute_ = false;
    }
}

void stimulus_chirp::step_rt() {
    switch (state_) {
        default:
        case state::off_s:
            if (zero_at_off_) {
                value_ = 0.0;
            }
            break;

        case state::init_s:
            value_ = 0.0;
            t_ = 0.0;
            phase_ = 0.0;
            state_ = state::running_s;
            break;

        case state::running_s:
            if (t_ >= chirp_time_s_) {
                // Done
                state_ = state::off_s;
                break;
            }
            value_ = amplitude_ * sin(phase_);
            // Find next instantaneous frequency - Linear model
            double wi = w0_ + (w1_ - w0_) * (t_ / chirp_time_s_);
            phase_ = fmod(phase_ + wi, 2.0*M_PI);
            // phase_ = helpers::angle_norm_pipi(phase_ + wi);
            // Advance time
            t_ += dt_;
            break;
    }
}

////////////////////////////////////////////////////////////////////////////
stimulus_step::stimulus_step(double sample_rate_hz) : 
    stimulus("Step", sample_rate_hz)
{
    step_state_ = step_state::start_s;
    start_time_s_ = 0.0;
    end_time_s_ = 1.0;
    test_value_ = 0.0;
    t_ = 0.0;
}

void stimulus_step::render_parameters() {
    const double f64_one = 1.0;

    ImGui::PushID(this);

    if (ImGui::InputScalar("Step value", ImGuiDataType_Double, &test_value_, &f64_one)) { do_recompute_ = true; }

    if (ImGui::InputScalar("Step start time (s)", ImGuiDataType_Double, &start_time_s_, &f64_one)) { do_recompute_ = true;}
    if (ImGui::InputScalar("Step end time (s)",   ImGuiDataType_Double, &end_time_s_, &f64_one)) { do_recompute_ = true;}

    ImGui::Checkbox("Zero at step off", &zero_at_off_);

    ImGui::PopID();

    if (do_recompute_) {
        do_recompute_ = false;
    }
}

void stimulus_step::step_rt() {
    switch (state_) {
        default:
        case state::off_s:
            if (zero_at_off_) {
                value_ = 0.0;
            }
            break;

        case state::init_s:
            step_state_ = step_state::start_s;
            state_ = state::running_s;
            t_ = 0.0;
            value_ = 0.0;
            break;

        case state::running_s:
            t_ += dt_;
            switch (step_state_) {
                default:
                case step_state::start_s:
                    if (t_ < start_time_s_) {
                        break;
                    }
                    step_state_ = step_state::step_s;
                    value_ = test_value_;
                    // Fall through
                case step_state::step_s:
                    if (t_ < end_time_s_) {
                        break;
                    }
                    step_state_ = step_state::end_s;
                    value_ = 0.0;
                    // Fall through
                case step_state::end_s:
                    break;
            }
            break;
    }
}

////////////////////////////////////////////////////////////////////////////
stimulus_const::stimulus_const(double sample_rate_hz) :
    stimulus("Constant", sample_rate_hz)
{
    const_value_ = 0.0;
    const_time_s_ = 0.0;
}

void stimulus_const::render_parameters() {
    const double f64_one = 1.0;

    ImGui::PushID(this);

    if (ImGui::InputScalar("const value",    ImGuiDataType_Double, &const_value_, &f64_one)) { do_recompute_ = true; }
    if (ImGui::InputScalar("const time (s)", ImGuiDataType_Double, &const_time_s_, &f64_one)) { do_recompute_ = true;}

    ImGui::Checkbox("Zero at const off", &zero_at_off_);

    ImGui::PopID();

    if (do_recompute_) {
        do_recompute_ = false;
    }
}

void stimulus_const::step_rt() {
    switch (state_) {
        default:
        case state::off_s:
            if (zero_at_off_) {
                value_ = 0.0;
            }
            break;

        case state::init_s:
            value_ = const_value_;
            t_ = 0.0;
            state_ = state::running_s;
            break;

        case state::running_s:
            t_ += dt_;
            if (t_ >= const_time_s_) {
                state_ = state::off_s;
            }
            break;
    }
}