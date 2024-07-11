// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_BC_TUNE_H_
#define GUI_BC_TUNE_H_

#include "jcs_host.h"
#include "imgui.h"
#include <array>
#include <string>
#include "gui_type_base.h"

//////////////////////////////////////////////////////////////////////
class gui_bc_tune : public gui_type_base {
public:
    gui_bc_tune(jcs::jcs_host* host, std::string const& target_device);
    ~gui_bc_tune() {}

    int startup();
    int step_rt();
    int render();

private:
    std::vector<std::string> const channels_ = {"controller_0", "controller_1"};

    int channel_combo_idx_;
    std::string active_channel_;

    // R/L test parameters
    struct test_r {
        float amplitude;
        int   time_ms;
        float result;
        std::string original_mode;
        test_r() : amplitude(2.0f), time_ms(3000), result(0.0f) {}
    };
    std::array<test_r, 2> test_r_parameters_;

    struct test_l {
        float bias;
        float amplitude;
        float frequency;
        int   time_ms;
        float result;
        std::string original_mode;
        test_l() : bias(2.5f), amplitude(2.0f), frequency(1000.0f), time_ms(3000), result(0.0f) {}
    };
    std::array<test_l, 2> test_l_parameters_;


    void test_r_get_parameters(std::string const& channel, test_r* storage);
    void test_l_get_parameters(std::string const& channel, test_l* storage);
    int do_test_r(test_r* storage, std::string const& channel);
    int do_test_l(test_l* storage, std::string const& channel);    
};

#endif