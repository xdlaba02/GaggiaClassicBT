#ifndef PID_AUTOTUNE_H
#define PID_AUTOTUNE_H

#include <algorithm>
#include <Arduino.h>

class PIDAutotune {

    enum PeakType {
        NONE,
        MIN,
        MAX
    };

    double m_noise_band;
    double m_step;

    bool m_running;
    
    PeakType m_current_peak_type;

    double m_out_min;
    double m_out_max;

    static const int m_prev_inputs_size = 100;
    double m_prev_inputs[m_prev_inputs_size + 1];
    int m_prev_inputs_count;

    unsigned long m_prev_peak_time;
    unsigned long m_current_peak_time;

    double m_prev_peak_max;
    double m_current_peak_min;
    double m_current_peak_max;

    double m_kU;
    double m_tU;
    
public:
    void cancel() {
        m_running = false;
    }

    void setNoiseBand(double noise_band) {
        m_noise_band = noise_band;
    }

    void setStep(double step) {
        m_step = step;
    }

    void setOutputRange(double out_min, double out_max) {
        m_out_min = out_min;
        m_out_max = out_max;
    }

    void getPID(double &kP, double &kI, double &kD) {
        kP = 0.6 * m_kU;
        kI = 1.2 * m_kU / m_tU;
        kD = 3.0 * m_kU * m_tU / 40.0;
    }

    bool compute(double input, double target, double &output) {
        if (!m_running) {
		    m_running = true;
    
            m_current_peak_type = NONE;

            m_prev_inputs_count = 0;

            m_prev_peak_time = 0;
            m_current_peak_time = 0;

            m_prev_peak_max = 0.0;
            m_current_peak_min = 0.0;
            m_current_peak_max = 0.0;

            m_kU = 0.0;
        }

	    if (input > target + m_noise_band) {
		    m_kU = std::max(m_kU - m_step, m_out_min);
	    }
	    else if (input < target - m_noise_band) {
	        m_kU = std::min(m_kU + m_step, m_out_max);
	    }

        double error = target - input;
        output = m_kU * error; 

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
		    return false;
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
                    double average_separation = (std::abs(m_current_peak_max - m_current_peak_min) + std::abs(m_current_peak_min - m_prev_peak_max)) / 2.0;
            
                    if (average_separation < 2 + m_noise_band) {
                        m_tU = (m_current_peak_time - m_prev_peak_time) / 1000.0;
                        m_running = false;
                        return true;
                    }
                }
            }

            m_current_peak_type = MIN;
            m_current_peak_min = input;
        }

        return false;
    }
};

#endif