// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef PLOT_MEASUREMENT_MULTI_H_
#define PLOT_MEASUREMENT_MULTI_H_

#include <vector>
#include <string>
#include "imgui.h"
#include "jcs_host_types.h"

class plot_measurement_multi {
public:
    // Inner channel struct
    struct channel {
        std::string name_;
        std::vector<double> y_;
        ImVec4 color_;
        bool visible_;
        
        channel(std::string const& name, int size, ImVec4 color = ImVec4(1,1,1,1)) : 
            name_(name), color_(color), visible_(true) {
            y_.resize(size);
        }
    };
    
    // Constructors
    plot_measurement_multi(std::string const& name,
                           std::string const& x_name, std::string const& y_name,
                           int const size);
    plot_measurement_multi(std::string const& name,
                           std::string const& x_name, std::string const& y_name,
                          int const sample_time_s, int const sample_rate_hz);
    
    // Destructor
    ~plot_measurement_multi() = default;
    
    // Channel management
    void add_channel(std::string const& channel_name, ImVec4 color = ImVec4(1,1,1,1));
    void remove_channel(std::string const& channel_name);
    void clear_channels();
    channel* get_channel(std::string const& channel_name);
    channel* get_channel(int index);
    
    // Plotting
    void plot();
    
    // Sample rate based
    void update_sample_rate(int sample_rate_hz);
    void update_storage_length(int sample_time_s, int sample_rate_hz);

    std::vector<double> x_;
    std::vector<channel> channels_;

private:
    // File I/O
    int write_channels_to_file();
    int emit_data(std::string const& path_and_file);
    
    // Member variables
    std::string name_;
    std::string x_name_;
    std::string y_name_;
    int size_;

    bool plot_cursors_;
    double cursor_tag_[4];
};

#endif