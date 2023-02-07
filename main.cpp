#include "gnuplot-iostream/gnuplot-iostream.h"
#include "gnuplot_utils/gp_data_writer.hpp"
#include "models/pid_controller.hpp"
#include "models/plant.hpp"
#include "models/rate_limiter.hpp"
#include <iostream>

int main()
{
    std::vector<double> pts_time;
    std::vector<double> pts_output;
    std::vector<double> pts_integrator;
    std::vector<double> pts_control;

    double sim_step = 0.0001;
    double control_period = 0.001;
    double max_sat = 1;
    double min_sat = -max_sat;
    double target = 0.5;
    double sim_end = 10;

    Plant plant(0, max_sat, min_sat, sim_step);
    PidController controller(40, 15, 0.1, 20, 0.1, control_period, max_sat,
                             min_sat);
    controller.toggleWindup(); // Off by default
    RateLimiter limiter(0, 100, 100, sim_step);

    for (double sim_time = 0, output = 0, control = 0, control_timer = 0;
         sim_time < sim_end; sim_time += sim_step, control_timer += sim_step)
    {
        double setpoint = limiter.update(target);
        double error = setpoint - output;
        if (control_timer >= control_period)
        {
            control = controller.update(error);
            control_timer = 0;
        }
        output = plant.update(control);

        pts_integrator.push_back(controller.getIntegratorValue());
        pts_time.push_back(sim_time);
        pts_output.push_back(output);
        pts_control.push_back(controller.getControlSignal());
    }

    // Plot data in GNU Plot
    Gnuplot gp;
    char buff[200];
    std::snprintf(buff, sizeof(buff),
                  "set xrange [%d:%d]\nset yrange [%d:%d]\n", 0, int(sim_end),
                  -1, 2);
    gp << buff;
    std::snprintf(buff, sizeof(buff),
                  "plot '-' with lines title '%s', '-' with lines "
                  "title '%s', '-' with lines title '%s' \n",
                  "Model Output", "Integrator Signal", "Control Signal");
    gp << buff;
    gp.send1d(boost::make_tuple(pts_time, pts_output));
    gp.send1d(boost::make_tuple(pts_time, pts_integrator));
    gp.send1d(boost::make_tuple(pts_time, pts_control));

    // Write data to GNU Plot readable format
    GnuPlotDataWriter::writeDataDat(
        std::vector<std::string>(
            {"Simulation Time", "Model Output", "Controller State"}),
        std::vector<std::vector<double>>(
            {pts_time, pts_output, pts_integrator, pts_control}));

    return 0;
}
