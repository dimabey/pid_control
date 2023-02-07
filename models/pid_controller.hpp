#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "plant.hpp"
#include "sim_block.hpp"
#include <cmath>

class PidController : public SimBlock
{
public:
    PidController(double p_gain, double i_gain, double bc_gain, double d_gain,
                  double tau_lpf, double sim_step, double max_sat,
                  double min_sat)
        : SimBlock(sim_step)
    {
        proportional_term_.k = p_gain;
        integral_term_.k_f = i_gain;
        integral_term_.k_b = bc_gain;
        diff_term_.k = d_gain;
        diff_term_.tau = tau_lpf;
        sat_model_.max = max_sat;
        sat_model_.min = min_sat;
    };
    ~PidController() override = default;

    double update(double error) final
    {
        proportional_term_.value = calcNewProportionalState(error);
        integral_term_.value = calcNewIntegratorState(error);
        diff_term_.value = calcNewDifferentiatorState(error);

        double pid_output =
            proportional_term_.value + integral_term_.value + diff_term_.value;

        double sat_model_output = checkSaturation(pid_output);
        double saturator_overflow = sat_model_output - pid_output;

        previous_state_ = {error, saturator_overflow, sat_model_output};

        return sat_model_output;
    }

    void toggleWindup() { windup_on_ = !windup_on_; }

    double getIntegratorValue() const { return integral_term_.value; }

    double getControlSignal() const { return previous_state_.control; }

private:
    double calcNewProportionalState(double error) const
    {
        return proportional_term_.k * error;
    }

    double calcNewIntegratorState(double error) const
    {
        double res = integral_term_.k_f * T_ * error + integral_term_.value;
        if (windup_on_)
            res += integral_term_.k_b * previous_state_.bc_error * T_;
        return res;
    }

    double calcNewDifferentiatorState(double error) const
    {
        double filtered_error =
            (T_ / diff_term_.tau) * error +
            (1 - T_ / diff_term_.tau) * previous_state_.error;
        return diff_term_.k * (filtered_error - previous_state_.error) / T_;
    }

    double checkSaturation(double input) const
    {
        if (input > sat_model_.max)
            return sat_model_.max;
        if (input < sat_model_.min)
            return sat_model_.min;
        return input;
    }

    struct Proportional
    {
        double k;
        double value;
    };
    Proportional proportional_term_;

    struct Integral
    {
        double k_f;
        double k_b;
        double value;
    };
    Integral integral_term_;

    struct Differential
    {
        double k;
        double tau;
        double value;
    };
    Differential diff_term_;

    struct StateMemory
    {
        double error;
        double bc_error;
        double control;
    };
    StateMemory previous_state_;

    struct SaturationModel
    {
        double max;
        double min;
    };
    SaturationModel sat_model_;

    bool windup_on_ = false;
};

#endif
