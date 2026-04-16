// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
// Thermal model parameter identification via Levenberg-Marquardt.
// Fits R1, C1 (1st order) or R1, C1, R2, C2 (2nd order) to recorded
// time-series data of power dissipated, winding temperature (from V/I),
// and housing temperature (from thermistor).
//
// Dependencies: Eigen3, lsqcpp (header-only, on top of Eigen)
//
#ifndef MC_THERMAL_FITTER_H_
#define MC_THERMAL_FITTER_H_

#include <vector>
#include <string>

namespace mc_thermal_fitter {

    // ---------------------------------------------------------------
    // Input data for the fitter — recorded during the commissioning test
    // ---------------------------------------------------------------
    struct recorded_data {
        std::vector<double> time_s;         // Timestamps [s], monotonically increasing
        std::vector<double> power_w;        // Instantaneous power dissipated P(t) [W]
        std::vector<double> t1_measured;    // Winding temperature T1(t) from R(t) [degC]
        std::vector<double> t2_measured;    // Housing temperature T2(t) from thermistor [degC]
        double t_initial;                   // Initial equilibrium temperature [degC]
        double t_ambient;                   // Ambient temperature (= t_initial for short tests) [degC]
    };

    // ---------------------------------------------------------------
    // Fit result
    // ---------------------------------------------------------------
    struct fit_result {
        // Fitted parameters
        double r1;          // Thermal resistance, winding to housing [K/W]
        double c1;          // Thermal capacitance, winding [J/K]
        double r2;          // Thermal resistance, housing to ambient [K/W] (2nd order only)
        double c2;          // Thermal capacitance, housing [J/K] (2nd order only)

        // Quality metrics
        double rms_error_t1;        // RMS error on T1 fit [K]
        double rms_error_t2;        // RMS error on T2 fit [K] (2nd order only)
        int    iterations;          // Number of LM iterations
        bool   converged;           // Did the optimiser converge?
        std::string info;           // Human-readable status string

        // Validation traces — predicted T1, T2 using the fitted parameters
        std::vector<double> t1_predicted;
        std::vector<double> t2_predicted;
    };

    // ---------------------------------------------------------------
    // Initial guess estimation from data
    // ---------------------------------------------------------------
    struct initial_guess {
        double r1;
        double c1;
        double r2;
        double c2;
    };

    // Estimate reasonable starting parameters from the recorded data.
    // Uses heuristics: C1 from initial heating rate, R1 from steady-state
    // temperature difference, R2/C2 from housing dynamics.
    initial_guess estimate_initial_params(const recorded_data& data, bool second_order);

    // ---------------------------------------------------------------
    // Fit functions
    // ---------------------------------------------------------------

    // Fit a 1st order thermal model (R1, C1 only).
    // T2 is treated as measured (thermistor).
    // dT1/dt = (1/C1) * (P - (T1 - T2_measured) / R1)
    fit_result fit_1st_order(const recorded_data& data,
                             const initial_guess& guess);

    // Fit a 2nd order thermal model (R1, C1, R2, C2).
    // dT1/dt = (1/C1) * (P - (T1 - T2) / R1)
    // dT2/dt = (1/C2) * ((T1 - T2) / R1 - (T2 - Ta) / R2)
    fit_result fit_2nd_order(const recorded_data& data,
                             const initial_guess& guess);

    // ---------------------------------------------------------------
    // Simulation — for validation and plotting
    // ---------------------------------------------------------------

    // Simulate the 1st order model forward given parameters and input data.
    // Returns predicted T1.
    void simulate_1st_order(const recorded_data& data,
                            double r1, double c1,
                            std::vector<double>* t1_out);

    // Simulate the 2nd order model forward given parameters and input data.
    // Returns predicted T1 and T2.
    void simulate_2nd_order(const recorded_data& data,
                            double r1, double c1, double r2, double c2,
                            std::vector<double>* t1_out,
                            std::vector<double>* t2_out);

    // ---------------------------------------------------------------
    // Data derivation helpers — compute derived signals from raw sampler data
    // ---------------------------------------------------------------
    struct derived_data_config {
        double rs_ref;              // Reference winding resistance [Ohm] (from test_dq_r)
        double rs_t_ref;            // Temperature at which rs_ref was measured [degC]
        double rs_alpha;            // Temperature coefficient of resistance [1/degC] (0.00393 for copper)
        double i_d_min_threshold;   // Minimum |i_d| to accept for R computation [A]
        double skip_time_s;         // Discard data before this elapsed time [s] (skip initial ramp/alignment)
    };

    // From raw v_d, i_d, t_housing samples, compute power, winding temperature, etc.
    // Samples where |i_d| < threshold are interpolated over.
    // Returns 0 on success.
    int derive_signals(const std::vector<float>& time_s_in,
                       const std::vector<float>& v_d_in,
                       const std::vector<float>& i_d_in,
                       const std::vector<float>& t_housing_in,
                       const derived_data_config& config,
                       recorded_data* out);

}; // namespace mc_thermal_fitter

#endif


// // Copyright (c) 2024 Arbite Robotics Pty Ltd
// // https://arbite.io
// //
// // Thermal model parameter identification via Levenberg-Marquardt.
// // Fits R1, C1 (1st order) or R1, C1, R2, C2 (2nd order) to recorded
// // time-series data of power dissipated, winding temperature (from V/I),
// // and housing temperature (from thermistor).
// //
// // Dependencies: Eigen3, lsqcpp (header-only, on top of Eigen)
// //
// #ifndef MC_THERMAL_FITTER_H_
// #define MC_THERMAL_FITTER_H_

// #include <vector>
// #include <string>

// namespace mc_thermal_fitter {

//     // ---------------------------------------------------------------
//     // Input data for the fitter — recorded during the commissioning test
//     // ---------------------------------------------------------------
//     struct recorded_data {
//         std::vector<double> time_s;         // Timestamps [s], monotonically increasing
//         std::vector<double> power_w;        // Instantaneous power dissipated P(t) [W]
//         std::vector<double> t1_measured;    // Winding temperature T1(t) from R(t) [degC]
//         std::vector<double> t2_measured;    // Housing temperature T2(t) from thermistor [degC]
//         double t_initial;                   // Initial equilibrium temperature [degC]
//         double t_ambient;                   // Ambient temperature (= t_initial for short tests) [degC]
//     };

//     // ---------------------------------------------------------------
//     // Fit result
//     // ---------------------------------------------------------------
//     struct fit_result {
//         // Fitted parameters
//         double r1;          // Thermal resistance, winding to housing [K/W]
//         double c1;          // Thermal capacitance, winding [J/K]
//         double r2;          // Thermal resistance, housing to ambient [K/W] (2nd order only)
//         double c2;          // Thermal capacitance, housing [J/K] (2nd order only)

//         // Quality metrics
//         double rms_error_t1;        // RMS error on T1 fit [K]
//         double rms_error_t2;        // RMS error on T2 fit [K] (2nd order only)
//         int    iterations;          // Number of LM iterations
//         bool   converged;           // Did the optimiser converge?
//         std::string info;           // Human-readable status string

//         // Validation traces — predicted T1, T2 using the fitted parameters
//         std::vector<double> t1_predicted;
//         std::vector<double> t2_predicted;
//     };

//     // ---------------------------------------------------------------
//     // Initial guess estimation from data
//     // ---------------------------------------------------------------
//     struct initial_guess {
//         double r1;
//         double c1;
//         double r2;
//         double c2;
//     };

//     // Estimate reasonable starting parameters from the recorded data.
//     // Uses heuristics: C1 from initial heating rate, R1 from steady-state
//     // temperature difference, R2/C2 from housing dynamics.
//     initial_guess estimate_initial_params(const recorded_data& data, bool second_order);

//     // ---------------------------------------------------------------
//     // Fit functions
//     // ---------------------------------------------------------------

//     // Fit a 1st order thermal model (R1, C1 only).
//     // T2 is treated as measured (thermistor).
//     // dT1/dt = (1/C1) * (P - (T1 - T2_measured) / R1)
//     fit_result fit_1st_order(const recorded_data& data,
//                              const initial_guess& guess);

//     // Fit a 2nd order thermal model (R1, C1, R2, C2).
//     // dT1/dt = (1/C1) * (P - (T1 - T2) / R1)
//     // dT2/dt = (1/C2) * ((T1 - T2) / R1 - (T2 - Ta) / R2)
//     fit_result fit_2nd_order(const recorded_data& data,
//                              const initial_guess& guess);

//     // ---------------------------------------------------------------
//     // Simulation — for validation and plotting
//     // ---------------------------------------------------------------

//     // Simulate the 1st order model forward given parameters and input data.
//     // Returns predicted T1.
//     void simulate_1st_order(const recorded_data& data,
//                             double r1, double c1,
//                             std::vector<double>* t1_out);

//     // Simulate the 2nd order model forward given parameters and input data.
//     // Returns predicted T1 and T2.
//     void simulate_2nd_order(const recorded_data& data,
//                             double r1, double c1, double r2, double c2,
//                             std::vector<double>* t1_out,
//                             std::vector<double>* t2_out);

//     // ---------------------------------------------------------------
//     // Data derivation helpers — compute derived signals from raw sampler data
//     // ---------------------------------------------------------------
//     struct derived_data_config {
//         double rs_ref;          // Reference winding resistance [Ohm] (from test_dq_r)
//         double rs_t_ref;        // Temperature at which rs_ref was measured [degC]
//         double rs_alpha;        // Temperature coefficient of resistance [1/degC] (0.00393 for copper)
//         double i_d_min_threshold;   // Minimum |i_d| to accept for R computation [A]
//     };

//     // From raw v_d, i_d, t_housing samples, compute power, winding temperature, etc.
//     // Samples where |i_d| < threshold are interpolated over.
//     // Returns 0 on success.
//     int derive_signals(const std::vector<float>& time_s_in,
//                        const std::vector<float>& v_d_in,
//                        const std::vector<float>& i_d_in,
//                        const std::vector<float>& t_housing_in,
//                        const derived_data_config& config,
//                        recorded_data* out);

// }; // namespace mc_thermal_fitter

// #endif