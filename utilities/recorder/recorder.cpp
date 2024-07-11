// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "recorder.h"
#include <fstream>
#include <iomanip>
#include "jcs_host.h"

recorder::recorder(int n_records, int n_points, std::string filename) {
    data_ = new std::vector<std::vector<float>>;
    data_->resize(n_points, std::vector<float>(n_records));

    n_points_ = n_points;
    n_records_ = n_records;
    point_idx_ = 0;

    filename_ = filename;
}

recorder::~recorder() {
    delete data_;
}

int recorder::add(std::vector<float>& values) {
    if (data_->empty()) {
        return jcs::RET_ERROR;
    }

    // std::vector<float> data_point(n_records_);
    // if (values.size() != data_points.size()) {
    //     return RET_ERROR;
    // }

    if (point_idx_ >= n_points_) {
        // Full
        return jcs::RET_ERROR;
    }

    data_->at(point_idx_) = values;
    point_idx_++;
    return jcs::RET_OK;
}

void recorder::reset() {
    point_idx_ = 0;
}

void recorder::write_to_file(std::vector<std::string>& header) {
    std::ofstream ofs;
    ofs.open(filename_);

    if (header.size() > 0) {
        int h = 0;
        while (h < (header.size()-1)) {
            ofs << header[h] << ",";
            h++;
        }
        ofs << header[h] << "\n";
    }

    for (int i=0; i<data_->size(); i++) {
        std::vector<float>* data_point = &data_->at(i);
        
        volatile int p = 0;
        while (p<(data_point->size()-1)) {
            ofs << std::fixed << std::setprecision(8) << data_point->at(p) << ",";
            p++;
        }
        ofs << std::fixed << std::setprecision(8) << data_point->at(p) << "\n";
    }

    ofs.close();
}