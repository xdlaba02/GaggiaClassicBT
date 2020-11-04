#ifndef PID_H
#define PID_H

#include <algorithm>

template <typename T>
T clamp(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}

class PID {
    double m_kP;
    double m_kI;
    double m_kD;

    double m_out_min;
    double m_out_max;

    unsigned long m_prev_time;

    double m_integral;

    double m_prev_output;

    double m_prevprev_error;
    double m_prev_error;

public:
    PID():
        m_kP{0.0},
        m_kI{0.0},
        m_kD{0.0},
        m_out_min{-std::numeric_limits<double>::infinity()},
        m_out_max{+std::numeric_limits<double>::infinity()},
        m_prev_time{millis()},
        m_integral{0.0},
        m_prev_output{0.0},
        m_prevprev_error{0.0},
        m_prev_error{0.0}
        {}

    void restart() {
        m_prev_time = millis();
        
        m_prev_output = 0.0;

        m_prevprev_error = 0.0;
        m_prev_error = 0.0;
    }

    void setTunings(double kP, double kI, double kD) {
        m_kP = kP;
        m_kI = kI;
        m_kD = kD;
    }

    void setOutputRange(double out_min, double out_max) {
        m_out_min = out_min;
        m_out_max = out_max;
    }

    double compute(double input, double target) {
        double p, i, d;
        return compute(input, target, p, i, d);
    }

    /*
    double compute(double input, double target, double &proportional, double &integral, double &derivative) {
        unsigned long now = millis();
        double delta = (now - m_prev_time) / 1000.0;

        double error = target - input;

        proportional = m_kP * (error - m_prev_error);
        integral     = m_kI * error * delta;
        derivative   = m_kD * (error - 2 * m_prev_error + m_prevprev_error) / delta;

        double output = clamp(m_prev_output + proportional + integral + derivative, m_out_min, m_out_max);

        m_prev_time = now;
        m_prev_output = output;
        m_prevprev_error = m_prev_error;
        m_prev_error = error;

        return output;
    }*/

    double compute(double input, double target, double &proportional, double &integral, double &derivative) {
        unsigned long now = millis();

        double delta = (now - m_prev_time) / 1000.0;
        m_prev_time = now;

        double error = target - input;

        proportional = m_kP * error;

        double integral_delta = m_kI * error * delta;
        
        derivative = m_kD * (error - m_prev_error) / delta;
        m_prev_error = error;

        double proposed_output = proportional + m_integral + integral_delta + derivative;

        // TODO
        if (std::signbit(proposed_output) == std::signbit(error)) {
            if (proposed_output > m_out_max) {
                if (proposed_output - integral_delta <= m_out_max) {
                    integral_delta -= proposed_output - m_out_max;
                }
                else {
                    integral_delta = 0;
                }
            }
            else if (proposed_output < m_out_min) {
                if (proposed_output - integral_delta >= m_out_min) {
                    integral_delta -= proposed_output - m_out_min;
                }
                else {
                    integral_delta = 0;
                }
            }
        }

        m_integral += integral_delta;

        integral = m_integral;
        
        return clamp(proportional + integral + derivative, m_out_min, m_out_max);
    }
};

#endif
