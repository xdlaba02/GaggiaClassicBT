#pragma once

#include <algorithm>

#include <Arduino.h>

class PID {
    float m_kP;
    float m_kI;
    float m_kD;

    float m_out_min;
    float m_out_max;

    unsigned long m_prev_time;

    float m_target;

    float m_proportional;
    float m_integral;
    float m_derivative;

    float m_prev_error;

public:
    PID():
        m_kP{0.f},
        m_kI{0.f},
        m_kD{0.f},
        m_out_min{-std::numeric_limits<float>::infinity()},
        m_out_max{+std::numeric_limits<float>::infinity()},
        m_prev_time{millis()},
        m_target{0.f},
        m_proportional{0.f},
        m_integral{0.f},
        m_derivative{0.f},
        m_prev_error{0.f}
        {}

    float proportional() const {
        return m_proportional;
    }

    float integral() const {
        return m_integral;
    }

    float derivative() const {
        return m_derivative;
    }

    void restart() {
        m_prev_time = millis();
        m_prev_error = 0.f;
        m_proportional = 0.f;
        m_integral = 0.f;
        m_derivative = 0.f;
    }

    void setTunings(float kP, float kI, float kD) {
        m_kP = kP;
        m_kI = kI;
        m_kD = kD;

        restart();
    }

    void setOutputRange(float out_min, float out_max) {
        m_out_min = out_min;
        m_out_max = out_max;
    }

    void setTarget(float target) {
        m_target = target;
    } 

    float compute(float input) {
        unsigned long now = millis();

        float delta = (now - m_prev_time) / 1000.0;
        m_prev_time = now;

        float error = m_target - input;

        m_proportional = m_kP * error;

        float integral_delta = m_kI * error * delta;
        
        m_derivative = m_kD * (error - m_prev_error) / delta;
        m_prev_error = error;

        float proposed_output = m_proportional + m_integral + integral_delta + m_derivative;

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

        return std::max(m_out_min, std::min(m_proportional + m_integral + m_derivative, m_out_max));
    }
};