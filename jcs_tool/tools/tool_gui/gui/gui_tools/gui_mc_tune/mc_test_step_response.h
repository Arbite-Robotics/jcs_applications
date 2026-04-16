// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef MC_TEST_STEP_RESPONSE_H_
#define MC_TEST_STEP_RESPONSE_H_

#include "jcs_host.h"
#include "plot_measurement_multi.h"
#include <string>
#include <vector>

/////////////////////////////////////////////////////////////////////////////////////////////
class mc_test_step_response {
public:
    // Parameters
    float amplitude;
    int   time_ms;

    // Axis selection: 0 = d, 1 = q
    int axis_index;
    static const std::vector<std::string> axis_names;

    // Mode selection: 0 = current, 1 = torque
    int mode_index;
    static const std::vector<std::string> mode_names;

    // Plot
    plot_measurement_multi plot;

    mc_test_step_response();
    void render_ui();
    void render_plot();
    int execute(jcs::jcs_host* host, std::string const& target);

private:
    // Enum values sent to device
    static const std::vector<std::string> axis_enum_values;
    static const std::vector<std::string> mode_enum_values;
};

#endif