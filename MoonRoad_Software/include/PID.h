#ifndef PID_H
#define PID_H
#include <cmath>

class PID {                                                                                  //Bundles variables and functions together
public:
    PID(float kp, float ki, float kd, float out_min, float out_max, float tau)               //Constructor calling
        : kp_(kp), ki_(ki), kd_(kd),
          out_min_(out_min), out_max_(out_max), tau_(tau),
          integrator_(0.0f), prev_measurement_(0.0f), d_term_filtered_(0.0f) {}

    float compute(float setpoint, float measurement, float dt) {                              //Control Logic target (setpoint), current sensor value (measurement), and time passed (dt)
        if (dt <= 0.0f) return 0.0f;

        float error = setpoint - measurement;

        float proportional = kp_ * error;

        float measurement_rate = (measurement - prev_measurement_) / dt;
        float alpha = tau_ / (tau_ + dt);                                                       //smoothing factor (alpha) based on filter time constant tau_ and time step dt.
        d_term_filtered_ = alpha * d_term_filtered_ + (1.0f - alpha) * measurement_rate;        //Low Pass filter
        float derivative = -kd_ * d_term_filtered_;

        float pre_saturated_output = proportional + integrator_ + derivative;                    //Antiwindup protection
        bool saturating_high = (pre_saturated_output >= out_max_ && error > 0);
        bool saturating_low  = (pre_saturated_output <= out_min_ && error < 0);

        if (!saturating_high && !saturating_low) {
            integrator_ += ki_ * error * dt;
        }

        float output = proportional + integrator_ + derivative;                                    //Sums up P, I, and D, clamps (limits) the final output so it never exceeds your servo's physical bounds, saves the current measurement for the next loop's derivative math, and returns the safe final angle command.
        
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

private:                                                                                            //private: label hides these variables so external code cannot accidentally mess with internal states.
    float kp_, ki_, kd_;
    float out_min_, out_max_;
    float tau_;

    float integrator_;
    float prev_measurement_;
    float d_term_filtered_;
};

#endif
