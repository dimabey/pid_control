#ifndef SIM_BLOCK_H
#define SIM_BLOCK_H

class SimBlock
{
public:
    explicit SimBlock(double sim_time) : T_{sim_time} {};
    virtual ~SimBlock() = default;

    virtual double update(double input) = 0;

protected:
    double T_;
};

#endif
