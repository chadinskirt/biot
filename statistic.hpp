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
                // variance after feeding normalize value give max 0.25 a prescaler * 4 brought it back to 0-1 
                float accel_score = std::clamp(0.4f * (feature.accel_mag_var * 4.0f) + 0.6f * feature.accel_mag_mean, 0.0f, 1.0f);
                float jerk_score = std::clamp(0.4f * feature.accel_jerk_mean + 0.6f * (feature.jerk_var * 4),0.0f,1.0f);

                b.unknow = std::clamp(std::abs(accel_score - jerk_score), 0.0f, 1.0f);
                float crash_score = std::clamp(0.6f * jerk_score + 0.4f * accel_score, 0.0f, 1.0f);

                if(feature.accel_peak >= 0.8f && feature.jerk_peak >= 0.85f){
                    crash_score = std::clamp(crash_score + 0.1f, 0.0f, 1.0f); // condition for modifier
                }
                else if(feature.accel_peak >= 0.8f || feature.jerk_peak >= 0.85f){
                    b.unknow += 0.1f;
                }
                
                if(feature.velocity_mean < 15 && crash_score > 0.5){
                    b.unknow += 0.1f;
                }
                //eg unknow 0.5 , crash : 0.7 -> crash: 0.35, stable: 0.15
                float commited = 1.0f - b.unknow;
                b.crash = std::clamp(crash_score * commited, 0.0f, 1.0f);
                b.stable = std::clamp((1.0f-crash_score) * commited, 0.0f, 1.0f);

//              std::cout<<"crash: "<< b.crash << ' ' << "stable: "<< b.stable<< ' '<< "unknow: " << b.unknow << ' ' <<'\n';//

                return b;
            };
        };

    class OrientationEngine {
        public:
            belief_t evaluate(const feature_t& feature){
                belief_t b1;
                float roll_score = std::clamp(0.7f * feature.roll_mean + 0.3f * (feature.roll_var * 4.0f), 0.0f, 1.0f);
                float pitch_score = std::clamp(0.7f * feature.pitch_mean + 0.3f * (feature.pitch_var * 4),0.0f,1.0f);
                
                b1.unknow = std::clamp(std::abs(roll_score-pitch_score), 0.0f, 1.0f);
                float crash_score = std::clamp((0.6f * roll_score + 0.4f * pitch_score), 0.0f, 1.0f);
                if(feature.roll_peak >= 0.8f && feature.jerk_peak >= 0.6f){
                    crash_score = std::clamp(crash_score + 0.1f, 0.0f, 1.0f);
                }
                else if(feature.roll_peak >= 0.8f || feature.pitch_peak >= 0.6f){
                    b1.unknow += 0.1f;
                }
                float commited = 1.0f - b1.unknow;
                b1.crash = std::clamp(crash_score * commited, 0.0f, 1.0f);
                b1.stable = std::clamp((1.0f-crash_score) * commited, 0.0f, 1.0f);

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
                    
                std::cout<< "unknow: " << out.unknow << '\n'<< "crash: " << out.crash << '\n' << "stable: " << out.stable << '\n';
                return out;
            };
    };
}
#endif