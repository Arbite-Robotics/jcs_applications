// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef MC_ENCODER_CORRECTOR_H_
#define MC_ENCODER_CORRECTOR_H_

#include <vector>

namespace mc_encoder_corrector {
    
    // Calibrator
    struct correction_table {
        std::vector<float> corrections;
        std::vector<float> corrections_increment;
        int table_size;
        double rms_error;
        double rms_error_corrected;
    };

    int build_correction_table(correction_table* result,
                               const std::vector<double>& reference_positions,
                               const std::vector<double>& recorded_positions,
                               int table_size);

    float apply_correction(float encoder_angle, const std::vector<float>& correction_table);

    double interpolate_correction(double query_angle, const std::vector<std::pair<double, double>>& sorted_data);

    double calculate_RMS_error(const std::vector<double>& reference,const std::vector<double>& recorded);
    double validate_correction(const std::vector<double>& reference,
                               const std::vector<double>& recorded,
                               const std::vector<float>& correction_table);

    void clear_correction_table(correction_table* correction);
    void initialise_correction_table(correction_table* correction, int table_size);

}; // End namespace mc_encoder_corrector;
#endif