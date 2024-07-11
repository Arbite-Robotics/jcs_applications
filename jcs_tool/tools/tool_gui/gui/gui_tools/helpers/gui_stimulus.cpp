// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_stimulus.h"
#include "helpers.h"

gui_stimulus::gui_stimulus(double sample_rate_hz) {
    active_idx_ = 0;

    stimulus_source_.push_back(new stimulus_ramp(sample_rate_hz));
    stimulus_source_.push_back(new stimulus_chirp(sample_rate_hz));
    stimulus_source_.push_back(new stimulus_step(sample_rate_hz));

    for (int i=0; i<stimulus_source_.size(); i++) {
        source_names_.push_back(stimulus_source_[i]->name_get());
    }
}

void gui_stimulus::start() {
    stimulus_source_[active_idx_]->start();
}
void gui_stimulus::stop() {
    stimulus_source_[active_idx_]->stop();
}
bool gui_stimulus::is_running() {
    return stimulus_source_[active_idx_]->is_running();
}
float gui_stimulus::value_get() {
    return stimulus_source_[active_idx_]->value_get();
}

void gui_stimulus::step_rt() {
    stimulus_source_[active_idx_]->step_rt();
}

void gui_stimulus::render_parameters() {

    ImGui::PushID(this);
    helpers::combo_select("Stimulus Source", &source_names_, &active_idx_, nullptr);
    stimulus_source_[active_idx_]->render_parameters();
    ImGui::PopID();
}