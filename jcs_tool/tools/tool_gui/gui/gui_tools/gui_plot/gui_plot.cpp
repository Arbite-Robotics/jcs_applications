
// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "gui_plot.h"
#include "plot_sink_int.h"
#include "plot_sink_opstate.h"
#include "plot_sink_plot.h"
#include "plot_source_slider.h"
#include "tool_gui_settings.h"
#include <string>
#include <iostream>
#include "imgui.h"

int gui_plot::startup() {

    if (configure_storage(jcs::signal_type::float32_s, &sink_f32_, &source_f32_) == jcs::RET_ERROR) { return jcs::RET_ERROR; }
    if (configure_storage(jcs::signal_type::uint32_s,  &sink_u32_, &source_u32_) == jcs::RET_ERROR) { return jcs::RET_ERROR; }
    if (configure_storage(jcs::signal_type::uint16_s,  &sink_u16_, &source_u16_) == jcs::RET_ERROR) { return jcs::RET_ERROR; }
    if (configure_storage(jcs::signal_type::uint8_s,   &sink_u8_,  &source_u8_) == jcs::RET_ERROR) { return jcs::RET_ERROR; }

    // Configure opstate
    sink_opstate_store_.resize(host_->sig_opstate_output_sz_rt());
    for (int i=0; i<host_->sig_opstate_output_sz_rt(); i++) {
        std::string node_name;
        if (host_->sig_opstate_output_node_name_get(i, &node_name) != jcs::RET_OK) {
            std::cout << "gui_plot: Error getting opstate node name for output signal at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        std::string name;
        if (host_->sig_opstate_output_name_get(i, &name) != jcs::RET_OK) {
            std::cout << "gui_plot: Error getting opstate signal name at index " << i << "\n";
            return jcs::RET_ERROR;
        }
        sink_opstate_.push_back(new plot_sink_opstate(node_name, name, &sink_opstate_store_[i]));
    }

    return jcs::RET_OK;
}

int gui_plot::configure_storage(jcs::signal_type const type, std::vector<std::vector<plot_sink*>*>* sink,
                                                        std::vector<std::vector<plot_source*>*>* source)
{
    // Sinks - from jcs_host to gui plots
    // 
    // Resize rate storage vector
    switch (type) {
        default:
            return jcs::RET_ERROR;
        case jcs::signal_type::float32_s:
            sink_f32_store_.resize(host_->sig_output_rate_sz_rt(type));
            break;
        case jcs::signal_type::uint32_s:
            sink_u32_store_.resize(host_->sig_output_rate_sz_rt(type));
            break;
        case jcs::signal_type::uint16_s:
            sink_u16_store_.resize(host_->sig_output_rate_sz_rt(type));
            break;
        case jcs::signal_type::uint8_s:
            sink_u8_store_.resize(host_->sig_output_rate_sz_rt(type));
            break;
    }
    for (int r=0; r<host_->sig_output_rate_sz_rt(type); r++) {
        // Resize signal storage vector
        switch (type) {
            default:
                return jcs::RET_ERROR;
            case jcs::signal_type::float32_s:
                sink_f32_store_[r].resize(host_->sig_output_sz_unsafe_rt(type, r));
                break;
            case jcs::signal_type::uint32_s:
                sink_u32_store_[r].resize(host_->sig_output_sz_unsafe_rt(type, r));
                break;
            case jcs::signal_type::uint16_s:
                sink_u16_store_[r].resize(host_->sig_output_sz_unsafe_rt(type, r));
                break;
            case jcs::signal_type::uint8_s:
                sink_u8_store_[r].resize(host_->sig_output_sz_unsafe_rt(type, r));
                break;
        }
        // Configure sink
        std::vector<plot_sink*>* sink_tmp = new std::vector<plot_sink*>;

        for (int i=0; i<host_->sig_output_sz_unsafe_rt(type, r); i++) {
            std::string node_name;
            if (host_->sig_output_node_name_get(type, r, i, &node_name) != jcs::RET_OK) {
                std::cout << "gui_plot: Error getting node name for output signal at index " << i << "\n";
                return jcs::RET_ERROR;
            }
            std::string name;
            if (host_->sig_output_name_get(type, r, i, &name) != jcs::RET_OK) {
                std::cout << "gui_plot: Error getting output signal name at index " << i << "\n";
                return jcs::RET_ERROR;
            }

            switch (type) {
                default:
                    return jcs::RET_ERROR;

                case jcs::signal_type::float32_s:
                    {
                        // Float gets units
                        std::string units;
                        if (host_->sig_output_units_get(type, r, i, &units) != jcs::RET_OK) {
                            std::cout << "gui_plot: Error getting output signal units at index " << i << "\n";
                            return jcs::RET_ERROR;
                        }
                        sink_tmp->push_back(new plot_sink_plot(node_name, name, units, &sink_f32_store_[r][i]));
                    }
                    break;
                case jcs::signal_type::uint32_s:
                    sink_tmp->push_back(new plot_sink_int(node_name, name, &sink_u32_store_[r][i]));
                    break;
                case jcs::signal_type::uint16_s:
                    sink_tmp->push_back(new plot_sink_int(node_name, name, &sink_u16_store_[r][i]));
                    break;
                case jcs::signal_type::uint8_s:
                    sink_tmp->push_back(new plot_sink_int(node_name, name, &sink_u8_store_[r][i]));
                    break;
            }
       }
       sink->push_back(sink_tmp);
    }

    // Sources - from gui inputs into jcs_host
    // Resize rate storage vector
    switch (type) {
        default:
            return jcs::RET_ERROR;
        case jcs::signal_type::float32_s:
            source_f32_store_.resize(host_->sig_input_rate_sz_rt(type));
            break;
        case jcs::signal_type::uint32_s:
            source_u32_store_.resize(host_->sig_input_rate_sz_rt(type));
            break;
        case jcs::signal_type::uint16_s:
            source_u16_store_.resize(host_->sig_input_rate_sz_rt(type));
            break;
        case jcs::signal_type::uint8_s:
            source_u8_store_.resize(host_->sig_input_rate_sz_rt(type));
            break;
    }
    for (int r=0; r<host_->sig_input_rate_sz_rt(type); r++) {
        // Resize signal storage vector
        switch (type) {
            default:
                return jcs::RET_ERROR;
            case jcs::signal_type::float32_s:
                source_f32_store_[r].resize(host_->sig_input_sz_unsafe_rt(type, r));
                break;
            case jcs::signal_type::uint32_s:
                source_u32_store_[r].resize(host_->sig_input_sz_unsafe_rt(type, r));
                break;
            case jcs::signal_type::uint16_s:
                source_u16_store_[r].resize(host_->sig_input_sz_unsafe_rt(type, r));
                break;
            case jcs::signal_type::uint8_s:
                source_u8_store_[r].resize(host_->sig_input_sz_unsafe_rt(type, r));
                break;
        }
        // Configure source
        std::vector<plot_source*>* source_tmp = new std::vector<plot_source*>;

        for (int i=0; i<host_->sig_input_sz_unsafe_rt(type, r); i++) {
            std::string node_name;
            if (host_->sig_input_node_name_get(type, r, i, &node_name) != jcs::RET_OK) {
                std::cout << "gui_plot: Error getting node name for input signal at index " << i << "\n";
                return jcs::RET_ERROR;
            }
            std::string name;
            if (host_->sig_input_name_get(type, r, i, &name) != jcs::RET_OK) {
                std::cout << "gui_plot: Error getting input signal name at index " << i << "\n";
                return jcs::RET_ERROR;
            }

            switch (type) {
                default:
                    return jcs::RET_ERROR;

                case jcs::signal_type::float32_s:
                    {
                        // Float gets units and limits
                        std::string units;
                        if (host_->sig_input_units_get(type, r, i, &units) != jcs::RET_OK) {
                            std::cout << "gui_plot: Error getting output signal units at index " << i << "\n";
                            return jcs::RET_ERROR;
                        }
                        float limit_h;
                        float limit_l;
                        if (host_->sig_input_limits_get_by_name(name, &limit_h, &limit_l) != jcs::RET_OK) {
                            std::cout << "gui_plot: Error getting input signal limits at index " << i << "\n";
                            return jcs::RET_ERROR;
                        }
                        source_tmp->push_back(new plot_source_slider(node_name, name, units, limit_h, limit_l, &source_f32_store_[r][i]));
                    }
                    break;
                case jcs::signal_type::uint32_s:
                    // Not implemented yet
                    delete(source_tmp);
                    return jcs::RET_OK; 
                    break;
                case jcs::signal_type::uint16_s:
                    // Not implemented yet
                    delete(source_tmp);
                    return jcs::RET_OK; 
                    break;
                case jcs::signal_type::uint8_s:
                    // Not implemented yet
                    delete(source_tmp);
                    return jcs::RET_OK; 
                    break;
            }
       }
       source->push_back(source_tmp);
    }
    return jcs::RET_OK;
}


int gui_plot::render_sink() {
    ImVec2 sink_size = ImGui::GetContentRegionAvail();
    sink_size.y *= 0.8f; 

    ImGui::BeginChild("Sink", sink_size, ImGuiChildFlags_None);

    static ImGuiTableFlags flags = ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable;
                                   
    if (ImGui::BeginTable("##table", 2, flags, ImVec2(-1,0))) {
        ImGui::TableSetupColumn("Plot", ImGuiTableColumnFlags_WidthFixed, sink_size.x * 0.80f);
        ImGui::TableSetupColumn("Info", ImGuiTableColumnFlags_WidthFixed, sink_size.x * 0.20f);

        for (unsigned int r=0; r<sink_f32_.size(); r++) {
            for (unsigned int i=0; i<sink_f32_[r]->size(); i++) {
                sink_f32_[r]->at(i)->update();
            }
        }
        for (unsigned int r=0; r<sink_u32_.size(); r++) {
            for (unsigned int i=0; i<sink_u32_[r]->size(); i++) {
                sink_u32_[r]->at(i)->update();
            }
        }
        for (unsigned int r=0; r<sink_u16_.size(); r++) {
            for (unsigned int i=0; i<sink_u16_[r]->size(); i++) {
                sink_u16_[r]->at(i)->update();
            }
        }
        for (unsigned int r=0; r<sink_u8_.size(); r++) {
            for (unsigned int i=0; i<sink_u8_[r]->size(); i++) {
                sink_u8_[r]->at(i)->update();
            }
        }
        for (unsigned int i=0; i<sink_opstate_.size(); i++) {
            sink_opstate_[i]->update();
        }

        ImGui::EndTable();
    }

    ImGui::EndChild();

    return jcs::RET_OK;
}

int gui_plot::render_source() {

    ImGui::BeginChild("Source", ImGui::GetContentRegionAvail(), ImGuiChildFlags_None);

    ImGui::PushItemWidth(-250); 

    ImGui::Checkbox("Input signals active", &signals_in_active_);

    if (!signals_in_active_) {
        ImGui::BeginDisabled();
    }
    for (unsigned int r=0; r<source_f32_.size(); r++) {
        for (unsigned int i=0; i<source_f32_[r]->size(); i++) {
            source_f32_[r]->at(i)->update();
        }
    }
    if (!signals_in_active_) {
        ImGui::EndDisabled();
    }
    ImGui::PopItemWidth();
    ImGui::EndChild();

    return jcs::RET_OK;
}

int gui_plot::render() {
    // Plot the actual stuff
    if (render_sink() != jcs::RET_OK) {
        return jcs::RET_ERROR;
    }
    ImGui::Separator();
    return render_source();
}

int gui_plot::step_rt() {
    // Perform data exchange with RT system

    // dev_host -> gui
    // Base rate is always ticked
    host_->sig_output_get_rt(0, &sink_f32_store_[0]);
    host_->sig_output_get_rt(0, &sink_u32_store_[0]);
    host_->sig_output_get_rt(0, &sink_u16_store_[0]);
    host_->sig_output_get_rt(0, &sink_u8_store_[0]);
    // Opstate
    host_->sig_opstate_output_get_rt(&sink_opstate_store_);    
    // Tick sub rates if they are valid and not stale
    for (int r=1; r<host_->sig_output_rate_sz_rt(jcs::signal_type::float32_s); r++) {
        if ( host_->sig_output_is_valid_unsafe_rt(jcs::signal_type::float32_s, r) &&
            !host_->sig_output_is_stale_unsafe_rt(jcs::signal_type::float32_s, r) )
        {
            host_->sig_output_get_rt(r, &sink_f32_store_[r]);
        }
    }
    for (int r=1; r<host_->sig_output_rate_sz_rt(jcs::signal_type::uint32_s); r++) {
        if ( host_->sig_output_is_valid_unsafe_rt(jcs::signal_type::uint32_s, r) &&
            !host_->sig_output_is_stale_unsafe_rt(jcs::signal_type::uint32_s, r) )
        {
            host_->sig_output_get_rt(r, &sink_u32_store_[r]);
        }
    }
    for (int r=1; r<host_->sig_output_rate_sz_rt(jcs::signal_type::uint16_s); r++) {
        if ( host_->sig_output_is_valid_unsafe_rt(jcs::signal_type::uint16_s, r) &&
            !host_->sig_output_is_stale_unsafe_rt(jcs::signal_type::uint16_s, r) )
        {
            host_->sig_output_get_rt(r, &sink_u16_store_[r]);
        }
    }
    for (int r=1; r<host_->sig_output_rate_sz_rt(jcs::signal_type::uint8_s); r++) {
        if ( host_->sig_output_is_valid_unsafe_rt(jcs::signal_type::uint8_s, r) &&
            !host_->sig_output_is_stale_unsafe_rt(jcs::signal_type::uint8_s, r) )
        {
            host_->sig_output_get_rt(r, &sink_u8_store_[r]);
        }
    }
    // gui -> dev_host
    // Setting the input value validates it for jcs_host
    if (signals_in_active_) {
        for (int r=0; r<host_->sig_input_rate_sz_rt(jcs::signal_type::float32_s); r++) {
            host_->sig_input_set_rt(r, source_f32_store_[r]);
            // host_->sig_input_set_rt(r, &source_u32_store_[r]);
            // host_->sig_input_set_rt(r, &source_u16_store_[r]);
            // host_->sig_input_set_rt(r, &source_u8_store_[r]);
        }
    }
    return jcs::RET_OK;    
}

int gui_plot::step_rt_always() {
    return jcs::RET_OK;
}