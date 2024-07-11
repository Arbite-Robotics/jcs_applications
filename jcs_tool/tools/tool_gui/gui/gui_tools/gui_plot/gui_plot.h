// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef GUI_PLOT_H_
#define GUI_PLOT_H_

#include "jcs_host.h"
#include "gui_type_base.h"
#include <vector>
#include "plot_sink.h"
#include "plot_sink_opstate.h"
#include "plot_source.h"
#include "imgui.h"

class gui_plot : public gui_type_base {
public:
    gui_plot(jcs::jcs_host* host, std::string const& target_device) :
        gui_type_base("Signal Plot", host, target_device) {
            signals_in_active_ = true;
        }
    ~gui_plot() {}

    int startup();
    int step_rt();
    int render();

private:

    int configure_storage(jcs::signal_type const type, std::vector<std::vector<plot_sink*>*>* sink,
                                                       std::vector<std::vector<plot_source*>*>* source);

    int render_source();
    int render_sink();

    // Single sample storage
    std::vector<std::vector<float>>         sink_f32_store_;
    std::vector<std::vector<float>>         source_f32_store_;
    std::vector<std::vector<uint32_t>>      sink_u32_store_;
    std::vector<std::vector<uint32_t>>      source_u32_store_;
    std::vector<std::vector<uint16_t>>      sink_u16_store_;
    std::vector<std::vector<uint16_t>>      source_u16_store_;
    std::vector<std::vector<uint8_t>>       sink_u8_store_;
    std::vector<std::vector<uint8_t>>       source_u8_store_;

    std::vector<uint8_t> sink_opstate_store_;

    // Plot source and sink
    std::vector<std::vector<plot_sink*>*>   sink_f32_;
    std::vector<std::vector<plot_source*>*> source_f32_;
    std::vector<std::vector<plot_sink*>*>   sink_u32_;
    std::vector<std::vector<plot_source*>*> source_u32_;
    std::vector<std::vector<plot_sink*>*>   sink_u16_;
    std::vector<std::vector<plot_source*>*> source_u16_;
    std::vector<std::vector<plot_sink*>*>   sink_u8_;
    std::vector<std::vector<plot_source*>*> source_u8_;

    std::vector<plot_sink_opstate*>         sink_opstate_;

    bool signals_in_active_;
};

#endif
