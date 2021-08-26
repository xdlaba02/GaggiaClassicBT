#pragma once

#include <algorithm>

#include <Arduino.h>

class PIDAutotune {
    enum PeakType {
        NONE,
        MIN,
        MAX
    };

    float m_noise_band;
    float m_step;

    PeakType m_current_peak_type;

    float m_out_min;
    float m_out_max;

    float m_target;
    bool m_running;

    static const int m_prev_inputs_size = 20;
    float m_prev_inputs[m_prev_inputs_size + 1];
    int m_prev_inputs_count;

    unsigned long m_prev_peak_time;
    unsigned long m_current_peak_time;

    float m_prev_peak_max;
    float m_current_peak_min;
    float m_current_peak_max;

    float m_kU;
    float m_tU;
    
public:
    void start() {
        m_current_peak_type = NONE;
        m_prev_inputs_count = 0;
        m_prev_peak_time = 0;
        m_current_peak_time = 0;
        m_prev_peak_max = 0.f;
        m_current_peak_min = 0.f;
        m_current_peak_max = 0.f;
        m_kU = 0.f;
        m_running = true;
    }

    void stop() {
        m_running = false;
    }

    void setNoiseBand(float noise_band) {
        m_noise_band = noise_band;
    }

    void setStep(float step) {
        m_step = step;
    }

    void setOutputRange(float out_min, float out_max) {
        m_out_min = out_min;
        m_out_max = out_max;
    }

    void setTarget(float target) {
        m_target = target;
    } 

    void getPID(float &kP, float &kI, float &kD) {
        kP = 0.6f * m_kU;
        kI = 1.2f * m_kU / m_tU;
        kD = 3.f * m_kU * m_tU / 40.f;
    }

    float compute(float input) {
	    if (input > m_target + m_noise_band) {
		    m_kU -= m_step;
	    }
	    else if (input < m_target - m_noise_band) {
	        m_kU += m_step;
	    }

        float output = std::max(m_out_min, std::min(m_kU * (m_target - input), m_out_max));

  	    bool is_max = true;
	    bool is_min = true;

  	    for (int i = m_prev_inputs_count - 1; i >= 0; i--) {
			is_max = is_max && input > m_prev_inputs[i];
    	    is_min = is_min && input < m_prev_inputs[i];
    	    m_prev_inputs[i + 1] = m_prev_inputs[i];
  	    }

  	    m_prev_inputs[0] = input; 
	
  	    if (m_prev_inputs_count < m_prev_inputs_size) {  
            m_prev_inputs_count++;
		    return output;
	    }

        if (is_max) {
            if (m_current_peak_type == MIN) {
                m_prev_peak_time = m_current_peak_time;
                m_prev_peak_max = m_current_peak_max;
            }
                
            m_current_peak_type = MAX;
            m_current_peak_max = input;
            m_current_peak_time = millis();
        }
        else if (is_min) {
            if (m_current_peak_type == MAX) {
                if (m_prev_peak_time != 0) {
                    float average_separation = (m_current_peak_max + m_prev_peak_max) / 2.f - m_current_peak_min;
            
                    if (average_separation < 2.f * m_noise_band) {
                        m_tU = (m_current_peak_time - m_prev_peak_time) / 1000.f;
                        m_running = false;
                        return output;
                    }
                }
            }

            m_current_peak_type = MIN;
            m_current_peak_min = input;
        }

        return output;
    }

    bool running() {
        return m_running;
    }
};