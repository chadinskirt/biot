#ifndef ANALYSIS
#define ANALYSIS
#include "packet.hpp"
#include "biot.hpp"
#include <algorithm>
namespace biot{
    struct feature_t{
        float accel_mag_mean;
        float accel_mag_var;

        float accel_jerk_mean;
        float jerk_var;

        float roll_mean;
        float roll_var;

        float pitch_mean;
        float pitch_var;

        float velocity_mean;

        float accel_peak;
        float jerk_peak;
        float roll_peak;
        float pitch_peak;
    };
    struct Welford{
        uint32_t count = 0;
        float mean = 0.0f;
        float M2 = 0.0f;
        void update(float x){
            count++;
            float delta1 = x- mean; // xk -Mk-1
            mean = delta1/count; //Mk

            float delta2 = x - mean; //xk-Mk
            M2+= delta2 * delta1; // sk = sk-1 (xk-mk)(xk-mk-1)
        }
        float variance(){
            if(count<2){
                return 0.0f; // too small iteration
            }
            return M2/(count-1);
        }
    };
    class Analyzer {
        public:
            void normalize(packet_t& p){
                float accel_stable = 9.8f;
                float accel_high = 30.0f;
                float jerk_stable = 0.0f;
                float jerk_high = 80.0f;
                float roll_high = 70.0f;
                float pitch_high = 35.0f;

                float accel_norm = (p.accel_mag-accel_stable)/(accel_high-accel_stable);
                float roll_norm = std::abs(p.roll) / roll_high;
                float pitch_norm = std::abs(p.pitch) / pitch_high;

                p.accel_jerk = p.accel_jerk/jerk_high;
                p.accel_mag = std::clamp(accel_norm, 0.0f, 1.0f);
                p.roll = std::clamp(roll_norm, 0.0f, 1.0f);
                p.pitch = std::clamp(pitch_norm, 0.0f, 1.0f);
            }
            feature_t extract(const biot::WindowView<biot::packet_t>& view){
                Welford accel;
                Welford jerk;
                Welford roll;
                Welford pitch;

                float accel_peak = 0.0f;
                float jerk_peak = 0.0f;
                float roll_peak = 0.0f;
                float pitch_peak = 0.0f;
                // go through all view[i] each are one packet_t (likely) and perform computation
                for (auto i = 0; i < view.size(); ++i){
                    const auto& p = view[i];
                    accel.update(p.accel_mag);
                    jerk.update(p.accel_jerk);
                    roll.update(p.roll);
                    pitch.update(p.pitch);

                    accel_peak = std::max(accel_peak,p.accel_mag);
                    jerk_peak = std::max(jerk_peak,p.accel_jerk);
                    roll_peak = std::max(roll_peak, p.roll);
                    pitch_peak = std::max(pitch_peak,p.pitch);
                }

                feature_t result;
                result.accel_peak = accel_peak;
                result.jerk_peak = jerk_peak;
                result.roll_peak = roll_peak;
                result.pitch_peak = pitch_peak;

                result.accel_mag_mean = accel.mean;
                result.accel_jerk_mean = jerk.mean;
                result.roll_mean = roll.mean;
                result.pitch_mean = pitch.mean;

                result.accel_mag_var = accel.variance();
                result.jerk_var = jerk.variance();
                result.roll_var = roll.variance();
                result.pitch_var = pitch.variance();

                return result;
            };
        };
}
#endif