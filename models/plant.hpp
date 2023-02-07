#ifndef PLANT_H
#define PLANT_H

#include "sim_block.hpp"

class Plant : public SimBlock
{
public:
    Plant(double int_init_val, double max_val, double min_val, double sim_step)
        : integrator_{int_init_val}, max_limit_{max_val}, min_limit_{min_val},
          SimBlock(sim_step){};
    ~Plant() override = default;

    double update(double input) final
    {
        integrator_ += input * T_;
        if (integrator_ > max_limit_)
            integrator_ = max_limit_;
        if (integrator_ < min_limit_)
            integrator_ = min_limit_;
        return integrator_;
    };

private:
    double integrator_;
    double max_limit_, min_limit_;
};

#endif