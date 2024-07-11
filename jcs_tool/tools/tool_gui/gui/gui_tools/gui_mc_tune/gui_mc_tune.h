// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_MC_TUNE_H_
#define GUI_MC_TUNE_H_

#include "jcs_host.h"
#include "imgui.h"
#include <array>
#include <string>
#include "gui_type_base.h"

//////////////////////////////////////////////////////////////////////
class gui_mc_tune : public gui_type_base {
public:
    gui_mc_tune(jcs::jcs_host* host, std::string const& target_device);
    ~gui_mc_tune() {}

    int startup();
    int step_rt();
    int render();

private:
    bool is_ready_;

    uint32_t pole_pairs_;

    // D/Q axis measurements
    struct test_r {
        float amplitude;
        int   time_ms;
        float result;
        test_r() : amplitude(1.0f), time_ms(3000), result(0.0f) {}
    };
    std::array<test_r, 2> test_r_parameters_;

    struct test_l {
        float bias;
        float amplitude;
        float frequency;
        int   time_ms;
        float result;
        test_l() : bias(1.0f), amplitude(0.5f), frequency(1000.0f), time_ms(3000), result(0.0f) {}
    };
    std::array<test_l, 2> test_l_parameters_;

    void test_r_get_parameters(std::string const& axis, test_r* storage);
    void test_l_get_parameters(std::string const& axis, test_l* storage);
    int do_test_r(test_r* storage, std::string const& axis);
    int do_test_l(test_l* storage, std::string const& axis);

    // Comnpute controller gains
    float i_ctl_bw_hz_;
    float i_ctl_bw_rads_;
    struct controller_gains {
        float kp;
        float ki;
        controller_gains() : kp(0.0f), ki(0.0f) {}
    };
    std::array<controller_gains, 2> controller_gains_;
    void compute_controller_gains(std::string const& axis, test_r* r_store, test_l* l_store, controller_gains* gains);

    // Step response test
    struct test_step_response {
        std::string axis_;
        float amplitude_;
        int   time_ms_;
        std::vector<float> data_;
        std::vector<float> x_;
        bool plot_cursors_;
        double cursor_tag_[4];
        test_step_response(std::string const& axis);
        void plot();
    };
    int step_response_do_test(test_step_response* test);
    void step_response_get_parameters(test_step_response* test);
    std::array<test_step_response*, 2> test_step_response_;

    int ready_test();
    
};

#endif