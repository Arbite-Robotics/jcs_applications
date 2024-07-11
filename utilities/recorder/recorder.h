// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef RECORDER_HELPER_H_
#define RECORDER_HELPER_H_

#include <stdint.h>
#include <vector>
#include <string>

class recorder {
public:
    recorder(int n_record, int n_points, std::string filename);
    ~recorder();

    void reset();
    int add(std::vector<float>& values);

    void write_to_file(std::vector<std::string>& header);


private:
    std::vector<std::vector<float>>* data_;
    int point_idx_;
    int n_points_;
    int n_records_;
    std::string filename_;
};

#endif