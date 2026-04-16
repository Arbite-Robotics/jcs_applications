// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "mc_thermal_fitter.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <sstream>

#include <Eigen/Dense>
#include <unsupported/Eigen/NumericalDiff>
#include <unsupported/Eigen/LevenbergMarquardt>

using namespace Eigen;

// ===================================================================
// Internal helpers
// ===================================================================
namespace {

// Forward Euler simulation of 1st order thermal model.
// T2 is taken from measured data (thermistor).
void sim_1st_order(const mc_thermal_fitter::recorded_data& data,
                   double r1, double c1,
                   std::vector<double>& t1_out)
{
    const size_t n = data.time_s.size();
    t1_out.resize(n);

    double t1 = data.t_initial;
    t1_out[0] = t1;

    for (size_t i = 1; i < n; ++i) {
        double dt = data.time_s[i] - data.time_s[i - 1];
        double p  = data.power_w[i - 1];
        double t2 = data.t2_measured[i - 1];

        double dT1 = (1.0 / c1) * (p - (t1 - t2) / r1) * dt;
        t1 += dT1;
        t1_out[i] = t1;
    }
}

// Forward Euler simulation of 2nd order thermal model.
void sim_2nd_order(const mc_thermal_fitter::recorded_data& data,
                   double r1, double c1, double r2, double c2,
                   std::vector<double>& t1_out,
                   std::vector<double>& t2_out)
{
    const size_t n = data.time_s.size();
    t1_out.resize(n);
    t2_out.resize(n);

    double t1 = data.t_initial;
    double t2 = data.t_initial;
    t1_out[0] = t1;
    t2_out[0] = t2;

    double ta = data.t_ambient;

    for (size_t i = 1; i < n; ++i) {
        double dt = data.time_s[i] - data.time_s[i - 1];
        double p  = data.power_w[i - 1];

        double dT1 = (1.0 / c1) * (p - (t1 - t2) / r1) * dt;
        double dT2 = (1.0 / c2) * ((t1 - t2) / r1 - (t2 - ta) / r2) * dt;

        t1 += dT1;
        t2 += dT2;
        t1_out[i] = t1;
        t2_out[i] = t2;
    }
}

double compute_rms(const std::vector<double>& predicted, const std::vector<double>& measured)
{
    double sum_sq = 0.0;
    size_t n = std::min(predicted.size(), measured.size());
    for (size_t i = 0; i < n; ++i) {
        double e = predicted[i] - measured[i];
        sum_sq += e * e;
    }
    return std::sqrt(sum_sq / static_cast<double>(n));
}

// ===================================================================
// Eigen LM functors
//
// Positivity enforcement via log-parameterisation: we optimise over
// log(param), so the actual parameter is exp(x) which is always > 0.
// This also gives LM uniform sensitivity across orders of magnitude —
// a step of 0.1 in log-space is 10% regardless of the absolute value.
// The initial guess is passed as log(guess); results are exp()'d back.
// ===================================================================

// 1st order functor: 2 parameters [log(R1), log(C1)], n_samples residuals on T1
struct Functor1stOrder : Eigen::DenseFunctor<double> {
    const mc_thermal_fitter::recorded_data* data;

    Functor1stOrder(const mc_thermal_fitter::recorded_data* d)
        : DenseFunctor<double>(2, static_cast<int>(d->time_s.size())), data(d) {}

    int operator()(const InputType& x, ValueType& fvec) const
    {
        double r1 = std::exp(x(0));
        double c1 = std::exp(x(1));

        const int n = static_cast<int>(data->time_s.size());
        double t1 = data->t_initial;
        fvec(0) = t1 - data->t1_measured[0];

        for (int i = 1; i < n; ++i) {
            double dt = data->time_s[i] - data->time_s[i - 1];
            double p  = data->power_w[i - 1];
            double t2 = data->t2_measured[i - 1];

            double dT1 = (1.0 / c1) * (p - (t1 - t2) / r1) * dt;
            t1 += dT1;
            fvec(i) = t1 - data->t1_measured[i];
        }
        return 0;
    }
};

// 2nd order functor: 4 parameters [log(R1), log(C1), log(R2), log(C2)],
// 2*n_samples residuals (T1 and T2 interleaved)
struct Functor2ndOrder : Eigen::DenseFunctor<double> {
    const mc_thermal_fitter::recorded_data* data;

    Functor2ndOrder(const mc_thermal_fitter::recorded_data* d)
        : DenseFunctor<double>(4, static_cast<int>(d->time_s.size()) * 2), data(d) {}

    int operator()(const InputType& x, ValueType& fvec) const
    {
        double r1 = std::exp(x(0));
        double c1 = std::exp(x(1));
        double r2 = std::exp(x(2));
        double c2 = std::exp(x(3));

        const int n = static_cast<int>(data->time_s.size());
        double t1 = data->t_initial;
        double t2 = data->t_initial;
        double ta = data->t_ambient;

        fvec(0)     = t1 - data->t1_measured[0];
        fvec(0 + n) = t2 - data->t2_measured[0];

        for (int i = 1; i < n; ++i) {
            double dt = data->time_s[i] - data->time_s[i - 1];
            double p  = data->power_w[i - 1];

            double dT1 = (1.0 / c1) * (p - (t1 - t2) / r1) * dt;
            double dT2 = (1.0 / c2) * ((t1 - t2) / r1 - (t2 - ta) / r2) * dt;

            t1 += dT1;
            t2 += dT2;

            fvec(i)     = t1 - data->t1_measured[i];
            fvec(i + n) = t2 - data->t2_measured[i];
        }
        return 0;
    }
};

// Helper to interpret LM status
const char* lm_status_string(Eigen::LevenbergMarquardtSpace::Status s) {
    switch (s) {
        case Eigen::LevenbergMarquardtSpace::NotStarted:                        return "Not started";
        case Eigen::LevenbergMarquardtSpace::Running:                           return "Running";
        case Eigen::LevenbergMarquardtSpace::ImproperInputParameters:           return "Improper input parameters";
        case Eigen::LevenbergMarquardtSpace::RelativeReductionTooSmall:         return "Converged (relative reduction)";
        case Eigen::LevenbergMarquardtSpace::RelativeErrorTooSmall:             return "Converged (relative error)";
        case Eigen::LevenbergMarquardtSpace::RelativeErrorAndReductionTooSmall: return "Converged (error and reduction)";
        case Eigen::LevenbergMarquardtSpace::CosinusTooSmall:                   return "Converged (cosine)";
        case Eigen::LevenbergMarquardtSpace::TooManyFunctionEvaluation:         return "Too many function evaluations";
        case Eigen::LevenbergMarquardtSpace::FtolTooSmall:                      return "Converged (ftol)";
        case Eigen::LevenbergMarquardtSpace::XtolTooSmall:                      return "Converged (xtol)";
        case Eigen::LevenbergMarquardtSpace::GtolTooSmall:                      return "Converged (gtol)";
        case Eigen::LevenbergMarquardtSpace::UserAsked:                         return "User asked";
        default:                                                                 return "Unknown";
    }
}

bool lm_converged(Eigen::LevenbergMarquardtSpace::Status s) {
    return s >= Eigen::LevenbergMarquardtSpace::RelativeReductionTooSmall
        && s <= Eigen::LevenbergMarquardtSpace::GtolTooSmall;
}

} // anonymous namespace

// ===================================================================
// Public API implementation
// ===================================================================

mc_thermal_fitter::initial_guess mc_thermal_fitter::estimate_initial_params(
    const recorded_data& data, bool second_order)
{
    // Physical sanity bounds for small-to-medium servo motors:
    //   R1 (winding-to-housing thermal resistance): 0.05–20 K/W
    //     - Small motor with good thermal path to housing: ~0.5 K/W
    //     - Small motor with poor thermal path: ~5 K/W
    //     - Frameless gimbal motor with thin stator: ~10 K/W
    //   C1 (winding thermal capacitance): 0.5–200 J/K
    //     - Small motor (~100g) with maybe 25g copper: ~10 J/K
    //     - Medium servo: 50–100 J/K
    //     - Larger motors: up to 200 J/K
    //   R2, C2 (housing): larger still, depends entirely on mounting
    const double r1_min = 0.05, r1_max = 20.0;
    const double c1_min = 0.5,  c1_max = 200.0;
    const double r2_min = 0.05, r2_max = 50.0;
    const double c2_min = 1.0,  c2_max = 5000.0;

    // Sensible mid-range defaults (geometric mean of the bounds — reasonable
    // for "unknown small servo motor" and in the middle of log-space)
    initial_guess g;
    g.r1 = 1.0;   // K/W
    g.c1 = 10.0;  // J/K
    g.r2 = 5.0;   // K/W
    g.c2 = 100.0; // J/K

    if (data.time_s.size() < 10) {
        return g;
    }

    const size_t n = data.time_s.size();

    // -----------------------------------------------------------
    // C1 estimate: find the maximum sustained dT1/dt during the test.
    //
    // C1 = P / (dT1/dt) at a point where the heat outflow through R1 is
    // small compared to the power input. The fastest heating rate occurs
    // at the highest power level when the temperature difference (T1-T2)
    // hasn't yet built up enough to matter.
    //
    // We use a moving window to smooth numerical derivatives, and pick
    // the point where P is largest. The ratio P/dT1dt at that point is a
    // conservative (upper) bound on C1 — actual C1 is a bit smaller
    // because some heat does leave through R1 — but it's in the right
    // ballpark and clamps well.
    // -----------------------------------------------------------
    {
        const int window = 20; // Samples to smooth the derivative over
        double best_ratio = -1.0;

        for (size_t i = window; i + window < n; ++i) {
            double p = data.power_w[i];
            if (p < 1.0) { continue; }   // Ignore low-power regions

            double dt = data.time_s[i + window] - data.time_s[i - window];
            if (dt < 1e-6) { continue; }

            double dT1 = data.t1_measured[i + window] - data.t1_measured[i - window];
            double dT1_dt = dT1 / dt;

            // Only consider points where the winding is actively heating
            // (ignore steady-state, transitions, and cooldown).
            if (dT1_dt < 0.1) { continue; }  // K/s threshold

            double ratio = p / dT1_dt;  // This is ~C1 (upper bound)
            if (ratio > best_ratio) {
                best_ratio = ratio;
            }
        }

        if (best_ratio > 0.0) {
            g.c1 = best_ratio;
        }
    }

    // -----------------------------------------------------------
    // R1 estimate: pick the most settled (high power, small dT1/dt) sample
    // and compute R1 = (T1 - T2) / P.
    //
    // This corresponds to near-steady-state where heat in ≈ heat out:
    // P ≈ (T1 - T2) / R1  =>  R1 = (T1 - T2) / P
    //
    // We look for high-power samples where T1 has stopped rising quickly
    // (small |dT1/dt|), which means the winding is close to thermal
    // equilibrium with the housing at that operating point.
    // -----------------------------------------------------------
    {
        const int window = 20;
        double best_dT = 0.0;
        double best_p  = 0.0;
        double best_stability = 1e9;  // Lower dT1/dt = more settled

        for (size_t i = window; i + window < n; ++i) {
            double p = data.power_w[i];
            if (p < 1.0) { continue; }

            double dt = data.time_s[i + window] - data.time_s[i - window];
            if (dt < 1e-6) { continue; }

            double dT1_dt = std::abs(
                (data.t1_measured[i + window] - data.t1_measured[i - window]) / dt);

            // Prefer the most settled region at good power.
            // Score: combine stability with power — we want low dT1/dt and high P.
            double score = dT1_dt / p;

            if (score < best_stability) {
                best_stability = score;
                best_dT = data.t1_measured[i] - data.t2_measured[i];
                best_p = p;
            }
        }

        if (best_p > 1.0 && best_dT > 0.1) {
            g.r1 = best_dT / best_p;
        }
    }

    if (second_order) {
        // R2: from housing rise relative to ambient at peak power
        double t2_max = *std::max_element(data.t2_measured.begin(), data.t2_measured.end());
        double max_p = *std::max_element(data.power_w.begin(), data.power_w.end());
        double dt_housing = t2_max - data.t_ambient;
        if (dt_housing > 0.5 && max_p > 1.0) {
            g.r2 = dt_housing / max_p;
        }

        // C2: housing is typically much more massive than the winding.
        // Rule of thumb — housing thermal time constant is 5-20x winding,
        // so C2 ~= 10 * C1 gives a decent starting point.
        g.c2 = g.c1 * 10.0;
    }

    // -----------------------------------------------------------
    // Clamp to physical sanity ranges. If the heuristic produced
    // something outside these bounds, it's almost certainly garbage
    // from noisy data — fall back to the bound rather than blowing up.
    // -----------------------------------------------------------
    g.r1 = std::max(r1_min, std::min(g.r1, r1_max));
    g.c1 = std::max(c1_min, std::min(g.c1, c1_max));
    g.r2 = std::max(r2_min, std::min(g.r2, r2_max));
    g.c2 = std::max(c2_min, std::min(g.c2, c2_max));

    return g;
}


mc_thermal_fitter::fit_result mc_thermal_fitter::fit_1st_order(
    const recorded_data& data,
    const initial_guess& guess)
{
    fit_result result;
    result.r1 = guess.r1;
    result.c1 = guess.c1;
    result.r2 = 0.0;
    result.c2 = 0.0;
    result.rms_error_t1 = 0.0;
    result.rms_error_t2 = 0.0;
    result.iterations = 0;
    result.converged = false;
    result.info = "";

    if (data.time_s.size() < 10) {
        result.info = "Insufficient data (need at least 10 samples)";
        return result;
    }

    // Set up functor + NumericalDiff for automatic Jacobian
    Functor1stOrder functor(&data);
    Eigen::NumericalDiff<Functor1stOrder> num_diff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<Functor1stOrder>> lm(num_diff);

    lm.setMaxfev(1000);
    lm.setXtol(1.0e-10);
    lm.setFtol(1.0e-10);
    lm.setGtol(1.0e-10);

    // Initial parameters: log() because functor uses exp() for positivity
    VectorXd x(2);
    x(0) = std::log(guess.r1);
    x(1) = std::log(guess.c1);

    auto status = lm.minimize(x);

    // exp() back to get actual parameter values
    result.r1 = std::exp(x(0));
    result.c1 = std::exp(x(1));
    result.iterations = static_cast<int>(lm.iterations());
    result.converged = lm_converged(status);

    // Validation traces
    sim_1st_order(data, result.r1, result.c1, result.t1_predicted);
    result.t2_predicted = data.t2_measured;

    result.rms_error_t1 = compute_rms(result.t1_predicted, data.t1_measured);

    std::ostringstream ss;
    ss << lm_status_string(status)
       << " in " << result.iterations << " iterations. "
       << "RMS T1 error: " << result.rms_error_t1 << " K";
    result.info = ss.str();

    return result;
}


mc_thermal_fitter::fit_result mc_thermal_fitter::fit_2nd_order(
    const recorded_data& data,
    const initial_guess& guess)
{
    fit_result result;
    result.r1 = guess.r1;
    result.c1 = guess.c1;
    result.r2 = guess.r2;
    result.c2 = guess.c2;
    result.rms_error_t1 = 0.0;
    result.rms_error_t2 = 0.0;
    result.iterations = 0;
    result.converged = false;
    result.info = "";

    if (data.time_s.size() < 10) {
        result.info = "Insufficient data (need at least 10 samples)";
        return result;
    }

    Functor2ndOrder functor(&data);
    Eigen::NumericalDiff<Functor2ndOrder> num_diff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<Functor2ndOrder>> lm(num_diff);

    lm.setMaxfev(2000);
    lm.setXtol(1.0e-10);
    lm.setFtol(1.0e-10);
    lm.setGtol(1.0e-10);

    VectorXd x(4);
    x(0) = std::log(guess.r1);
    x(1) = std::log(guess.c1);
    x(2) = std::log(guess.r2);
    x(3) = std::log(guess.c2);

    auto status = lm.minimize(x);

    result.r1 = std::exp(x(0));
    result.c1 = std::exp(x(1));
    result.r2 = std::exp(x(2));
    result.c2 = std::exp(x(3));
    result.iterations = static_cast<int>(lm.iterations());
    result.converged = lm_converged(status);

    std::vector<double> t1_pred, t2_pred;
    sim_2nd_order(data, result.r1, result.c1, result.r2, result.c2, t1_pred, t2_pred);
    result.t1_predicted = std::move(t1_pred);
    result.t2_predicted = std::move(t2_pred);

    result.rms_error_t1 = compute_rms(result.t1_predicted, data.t1_measured);
    result.rms_error_t2 = compute_rms(result.t2_predicted, data.t2_measured);

    std::ostringstream ss;
    ss << lm_status_string(status)
       << " in " << result.iterations << " iterations. "
       << "RMS T1 error: " << result.rms_error_t1 << " K, "
       << "RMS T2 error: " << result.rms_error_t2 << " K";
    result.info = ss.str();

    return result;
}


void mc_thermal_fitter::simulate_1st_order(
    const recorded_data& data,
    double r1, double c1,
    std::vector<double>* t1_out)
{
    sim_1st_order(data, r1, c1, *t1_out);
}


void mc_thermal_fitter::simulate_2nd_order(
    const recorded_data& data,
    double r1, double c1, double r2, double c2,
    std::vector<double>* t1_out,
    std::vector<double>* t2_out)
{
    sim_2nd_order(data, r1, c1, r2, c2, *t1_out, *t2_out);
}


int mc_thermal_fitter::derive_signals(
    const std::vector<float>& time_s_in,
    const std::vector<float>& v_d_in,
    const std::vector<float>& i_d_in,
    const std::vector<float>& t_housing_in,
    const derived_data_config& config,
    recorded_data* out)
{
    if (out == nullptr) {
        return -1;
    }

    size_t n_raw = time_s_in.size();
    if (n_raw < 2 || v_d_in.size() != n_raw || i_d_in.size() != n_raw || t_housing_in.size() != n_raw) {
        return -1;
    }

    // Find start index — skip the initial ramp/alignment transient
    size_t i_start = 0;
    double t_origin = static_cast<double>(time_s_in[0]);
    for (size_t i = 0; i < n_raw; ++i) {
        if (static_cast<double>(time_s_in[i]) - t_origin >= config.skip_time_s) {
            i_start = i;
            break;
        }
    }

    size_t n = n_raw - i_start;
    if (n < 2) {
        return -1;
    }

    // Re-zero time origin to the start of usable data
    double t_zero = static_cast<double>(time_s_in[i_start]);

    out->time_s.resize(n);
    out->power_w.resize(n);
    out->t1_measured.resize(n);
    out->t2_measured.resize(n);

    double rs_ref = config.rs_ref;
    double t_ref  = config.rs_t_ref;
    double alpha  = config.rs_alpha;
    double i_min  = config.i_d_min_threshold;

    // First pass: compute R(t), T1(t), P(t) where i_d is valid
    std::vector<bool> valid(n, false);

    for (size_t i = 0; i < n; ++i) {
        size_t src = i + i_start;
        out->time_s[i]      = static_cast<double>(time_s_in[src]) - t_zero;
        out->t2_measured[i] = static_cast<double>(t_housing_in[src]);

        double v_d = static_cast<double>(v_d_in[src]);
        double i_d = static_cast<double>(i_d_in[src]);

        if (std::abs(i_d) >= i_min) {
            double r_inst = v_d / i_d;
            // T1 = T_ref + (R - Rs_ref) / (Rs_ref * alpha)
            double t1 = t_ref + (r_inst - rs_ref) / (rs_ref * alpha);
            out->t1_measured[i] = t1;
            out->power_w[i] = i_d * i_d * r_inst;
            valid[i] = true;
        }
    }

    // Second pass: interpolate over invalid samples (where i_d was too small)
    int first_valid = -1;
    int last_valid = -1;
    for (size_t i = 0; i < n; ++i) {
        if (valid[i]) {
            if (first_valid < 0) { first_valid = static_cast<int>(i); }
            last_valid = static_cast<int>(i);
        }
    }

    if (first_valid < 0) {
        return -1;
    }

    // Fill leading invalid samples with first valid value
    for (int i = 0; i < first_valid; ++i) {
        out->t1_measured[i] = out->t1_measured[first_valid];
        out->power_w[i] = 0.0;
    }

    // Fill trailing invalid samples with last valid value
    for (int i = last_valid + 1; i < static_cast<int>(n); ++i) {
        out->t1_measured[i] = out->t1_measured[last_valid];
        out->power_w[i] = 0.0;
    }

    // Linearly interpolate gaps in the middle
    int prev_valid = first_valid;
    for (size_t i = first_valid + 1; i <= static_cast<size_t>(last_valid); ++i) {
        if (valid[i]) {
            if (static_cast<int>(i) - prev_valid > 1) {
                double t_start  = out->time_s[prev_valid];
                double t_end    = out->time_s[i];
                double t1_start = out->t1_measured[prev_valid];
                double t1_end   = out->t1_measured[i];
                double p_start  = out->power_w[prev_valid];
                double p_end    = out->power_w[i];

                for (int j = prev_valid + 1; j < static_cast<int>(i); ++j) {
                    double frac = (out->time_s[j] - t_start) / (t_end - t_start);
                    out->t1_measured[j] = t1_start + frac * (t1_end - t1_start);
                    out->power_w[j]     = p_start + frac * (p_end - p_start);
                }
            }
            prev_valid = static_cast<int>(i);
        }
    }

    // Set initial conditions from first usable sample
    out->t_initial = out->t2_measured[0];
    out->t_ambient = out->t2_measured[0];

    return 0;
}
