#ifndef PID_H
#define PID_H

#include <cmath>

class PID {
public:
    PID(float kp, float ki, float kd, float out_min, float out_max, float tau)
        : kp_(kp), ki_(ki), kd_(kd),
          out_min_(out_min), out_max_(out_max), tau_(tau),
          integrator_(0.0f), prev_measurement_(0.0f), d_term_filtered_(0.0f) {}

    float compute(float setpoint, float measurement, float dt) {
        if (dt <= 0.0f) return 0.0f;

        float error = setpoint - measurement;

        float proportional = kp_ * error;

        float measurement_rate = (measurement - prev_measurement_) / dt;
        float alpha = tau_ / (tau_ + dt);
        d_term_filtered_ = alpha * d_term_filtered_ + (1.0f - alpha) * measurement_rate;
        float derivative = -kd_ * d_term_filtered_;

        float pre_saturated_output = proportional + integrator_ + derivative;
        bool saturating_high = (pre_saturated_output >= out_max_ && error > 0);
        bool saturating_low  = (pre_saturated_output <= out_min_ && error < 0);

        if (!saturating_high && !saturating_low) {
            integrator_ += ki_ * error * dt;
        }

        float output = proportional + integrator_ + derivative;
        
        if (output > out_max_) output = out_max_;
        else if (output < out_min_) output = out_min_;

        prev_measurement_ = measurement;

        return output;
    }

    void reset() {
        integrator_ = 0.0f;
        prev_measurement_ = 0.0f;
        d_term_filtered_ = 0.0f;
    }

    void setGains(float kp, float ki, float kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

private:
    float kp_, ki_, kd_;
    float out_min_, out_max_;
    float tau_;

    float integrator_;
    float prev_measurement_;
    float d_term_filtered_;
};

#endif