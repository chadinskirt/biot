#ifndef statistic
#define statistic
#include "analysis.hpp"
#include <cmath>
namespace biot{
    struct belief_t{
        float crash;
        float stable;
        float unknow;
    };
    class ImpactEngine {
        public:
            belief_t evaluate(const feature_t& feature){
                belief_t b;
                float accel_score = 0.6f * feature.accel_mag_var + 0.4f * feature.accel_mag_mean;
                float jerk_score = 0.4f * feature.accel_jerk_mean + 0.6f * feature.jerk_var;

                float disagreement = std::abs(accel_score - jerk_score);
                if(feature.accel_peak - accel_score >= 0.5 || feature.jerk_peak - jerk_score >= 0.5){
                    b.unknow = 1.1f * disagreement;
                }
                
                if(feature.velocity_mean >= 15){
                    b.unknow = 1.1f * disagreement;
                }
                else{
                    b.unknow = 0.9f * disagreement;
                }
                //eg unknow 0.5 , crash : 0.7 -> crash: 0.35, stable: 0.15
                float crash_score = (0.6 * jerk_score + 0.4 * accel_score);
                float commited = 1 - b.unknow;
                b.crash = crash_score * commited;
                b.stable = (1-crash_score) * commited;
                return b;
            };
        };

    class OrientationEngine {
        public:
            belief_t evaluate(const feature_t& feature){
                belief_t b1;
                float roll_score = 0.7 * feature.roll_mean + 0.3* feature.roll_var;
                float pitch_score = 0.7 * feature.pitch_mean + 0.3* feature.pitch_var;
                
                b1.unknow = std::abs(roll_score-pitch_score);
                float crash_score = (0.6 * roll_score + 0.4 * pitch_score);
                float commited = 1 - b1.unknow;
                b1.crash = crash_score * commited;
                b1.stable = (1-crash_score) * commited;
                return b1;
            };
        };


    class FusionEngine {
        public:
            belief_t combine(
                const belief_t& impact,
                const belief_t& orientation
            ){
                // sigma of A intersect B give conflict/empty
                float K =
                    impact.crash  * orientation.stable +
                    impact.stable * orientation.crash;
                
                float denom = 1.0f - K;

                belief_t out;

                // sigma A intersect B give crash
                out.crash =
                    (impact.crash * orientation.crash +
                    impact.crash * orientation.unknow +
                    impact.unknow * orientation.crash) / denom;
                
                // sigma A intersect B give stable
                out.stable =
                    (impact.stable * orientation.stable +
                    impact.stable * orientation.unknow +
                    impact.unknow * orientation.stable) / denom;
                
                // sigma A intersect B give unknow only one case 
                out.unknow =
                    (impact.unknow * orientation.unknow) / denom;

                return out;
            };
    };
}
#endif