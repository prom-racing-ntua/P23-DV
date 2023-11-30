#include "sim_steering_modelling.hpp"

namespace sim
{


class steeringActuation: public Actuation<double>
{
    private:
    SteeringState steering_model;
    double integration_frequency;
    bool simplified;
    public:
    steeringActuation(): Actuation<double>(){}
    void init(double delay, double kp, double kd, Constants a, bool simplified, double model_integration_frequency);
    void propagate(double dt, double time, double vx, double ax);
    void propagate(double dt, double time)override {exit(69);}
};

class brakeActuation: public Actuation<double>
{
    public:
    brakeActuation(): Actuation<double>(){}
    double get_command()const {return this->Actuation<double>::get_command();}
    void add_command(double command, double time) {this->Actuation<double>::add_command(command, time);}
    void propagate(double dt, double time) override;
};

class motorActuation: public Actuation<double>
{
    public:
    motorActuation(): Actuation<double>(){}
    void init(double delay);
    double get_command()const {return this->Actuation<double>::get_command();}
    void add_command(double command, double time) {this->Actuation<double>::add_command(command, time);}
    void propagate(double dt, double time) override;
};
} // namespace sim
