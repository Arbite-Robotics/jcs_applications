// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "mc_encoder_corrector.h"
#include <algorithm>
#include <limits>
#include "helpers.h"
#include "jcs_host_types.h"


int mc_encoder_corrector::build_correction_table(
    correction_table* result,
    const std::vector<double>& reference_positions,
    const std::vector<double>& recorded_positions,
    int table_size)
{
    if (result == nullptr) {
        return jcs::RET_ERROR;
    }

    // result->corrections.clear();
    result->table_size = 0;
    result->rms_error = 0.0;
    result->rms_error_corrected = 0.0;

    if (reference_positions.size() != recorded_positions.size()) {
        return jcs::RET_ERROR;
    }
    if (reference_positions.empty()) {
        return jcs::RET_ERROR;
    }
    if (table_size < 2) {
        return jcs::RET_ERROR;
    }
    if (result->corrections.size() != table_size) {
        return jcs::RET_ERROR;
    }
    if (result->corrections_increment.size() != table_size) {
        return jcs::RET_ERROR;
    }

    result->table_size = table_size;

    // Sort data by recorded positions for interpolation
    std::vector<std::pair<double, double>> sorted_data;
    sorted_data.reserve(reference_positions.size());
    
    for (size_t i = 0; i < reference_positions.size(); ++i) {
        double error = reference_positions[i] - recorded_positions[i];
        // Handle wrap-around
        error = helpers::angle_norm_pipi(error);
        sorted_data.push_back({recorded_positions[i], error});
    }
    
    std::sort(sorted_data.begin(), sorted_data.end());
    
    result->rms_error = calculate_RMS_error(reference_positions, recorded_positions);
        
    for (int i = 0; i < table_size; ++i) {
        double table_angle = (M_TWO_PI * i) / table_size;
        result->corrections_increment[i] = static_cast<float>(interpolate_correction(table_angle, sorted_data));
    }
    
    // Accumulate corrections
    for (int i = 0; i < table_size; ++i) {
        result->corrections[i] += result->corrections_increment[i];
    }
    
    // Compute the new RMS error off the increment.
    // The existing recorded positions may already have corrections applied
    result->rms_error_corrected = validate_correction(reference_positions, recorded_positions, result->corrections_increment);
    
    return jcs::RET_OK;
}

float mc_encoder_corrector::apply_correction(float encoder_angle, const std::vector<float>& correction_table) {
    if (correction_table.empty()) {
        return encoder_angle;
    }
    
    int table_size = correction_table.size();
    
    // Normalize angle to [0, 2π]
    encoder_angle = helpers::angle_norm_2pi(encoder_angle);

    float table_position = (encoder_angle * table_size) / M_TWO_PI;
    int index_low = static_cast<int>(table_position);
    int index_high = (index_low + 1) % table_size;
    
    float fraction = table_position - index_low;
    float correction = (1.0f - fraction) * correction_table[index_low] + fraction * correction_table[index_high];
    
    float corrected = encoder_angle + correction;
    
    // Normalize result
    corrected = helpers::angle_norm_2pi(corrected);
    
    return corrected;
}

double mc_encoder_corrector::interpolate_correction(double query_angle, const std::vector<std::pair<double, double>>& sorted_data) {
    if (sorted_data.empty()) {
        return 0.0;
    }
    if (sorted_data.size() == 1) {
        return sorted_data[0].second;
    }
    
    double first_angle = sorted_data.front().first;
    double last_angle = sorted_data.back().first;
    
    // Handle wrap-around region
    if (query_angle < first_angle || query_angle > last_angle) {
        double angle_diff = (first_angle + M_TWO_PI) - last_angle;
        double query_offset = query_angle < first_angle ? 
                              query_angle + M_TWO_PI - last_angle :
                              query_angle - last_angle;
        
        if (angle_diff < 1e-10) {
            return sorted_data.back().second;
        }
        
        double fraction = query_offset / angle_diff;
        return sorted_data.back().second * (1.0 - fraction) + sorted_data.front().second * fraction;
    }
    
    // Find bracketing points
    auto it = std::lower_bound(sorted_data.begin(), sorted_data.end(), std::make_pair(query_angle, 0.0));
    
    if (it == sorted_data.begin()) {
        return sorted_data.front().second;
    }
    if (it == sorted_data.end()) {
        return sorted_data.back().second;
    }
    
    auto prev_it = std::prev(it);
    double angle1 = prev_it->first;
    double angle2 = it->first;
    double error1 = prev_it->second;
    double error2 = it->second;
    
    double angle_diff = angle2 - angle1;
    if (angle_diff < 1e-10) {
        return error1;
    }
    
    double fraction = (query_angle - angle1) / angle_diff;
    return error1 * (1.0 - fraction) + error2 * fraction;
}

double mc_encoder_corrector::calculate_RMS_error(
    const std::vector<double>& reference,
    const std::vector<double>& recorded)
{
    double sum_squared_error = 0.0;
    
    for (size_t i = 0; i < reference.size(); ++i) {
        double error = reference[i] - recorded[i];
        error = helpers::angle_norm_pipi(error);
        sum_squared_error += error * error;
    }
    return std::sqrt(sum_squared_error / reference.size());
}

double mc_encoder_corrector::validate_correction(
    const std::vector<double>& reference,
    const std::vector<double>& recorded,
    const std::vector<float>& correction_table)
{
    double sum_squared_error = 0.0;
    
    for (size_t i = 0; i < reference.size(); ++i) {
        float corrected = apply_correction(recorded[i], correction_table);
        double error = reference[i] - corrected;
        error = helpers::angle_norm_pipi(error);
        sum_squared_error += error * error;
    }
    return std::sqrt(sum_squared_error / reference.size());
}

void mc_encoder_corrector::clear_correction_table(correction_table* correction) {
    std::fill(correction->corrections.begin(), correction->corrections.end(), 0.0f);
    std::fill(correction->corrections_increment.begin(), correction->corrections_increment.end(), 0.0f);
    correction->rms_error = 0.0f;
    correction->rms_error_corrected = 0.0f;
}

void mc_encoder_corrector::initialise_correction_table(correction_table* correction, int table_size) {
    correction->table_size = table_size;
    correction->corrections.resize(table_size);
    correction->corrections_increment.resize(table_size);
}