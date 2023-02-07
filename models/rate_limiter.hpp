#ifndef RATE_LIMITTER_H
#define RATE_LIMITTER_H

#include "sim_block.hpp"

class RateLimiter : public SimBlock
{
public:
    RateLimiter(double init_state, double max_fall_rate, double max_rise_rate,
                double sim_step)
        : output_{init_state}, max_fall_rate_{max_fall_rate},
          max_rise_rate_(max_rise_rate), SimBlock(sim_step){};
    ~RateLimiter() override = default;

    double update(double input) final
    {
        double crnt_rate = (output_ - input) / T_;
        if (crnt_rate > max_rise_rate_)
            output_ += max_rise_rate_ * T_;
        if (crnt_rate < max_fall_rate_)
            output_ += max_fall_rate_ * T_;
        else
            output_ = input;
        return output_;
    };

    void setMaxFallRate(double max_fall_rate)
    {
        max_fall_rate_ = max_fall_rate;
    }

    void setMaxRiseRate(double max_rise_rate)
    {
        max_rise_rate_ = max_rise_rate;
    }

private:
    double output_;
    double max_fall_rate_, max_rise_rate_;
};

#endif