#ifndef LOCALIZER_HPP
#define LOCALIZER_HPP
#include <uWS/uWS.h>
#include <iostream>
#include <json.hpp>
#include <math.h>
#include <map.hpp>
#include <particle_filter.hpp>
#include <memory>
#include <vector>

struct InputLocalizationMsg
{
    double sense_x;
    double sense_y;
    double sense_theta;
    double previous_velocity;
    double previous_yawrate;
    std::vector<float> x_sense;
    std::vector<float> y_sense;
};

struct OutputLocalizationMsg
{
    double x;
    double y;
    double theta;
    std::string association;
    std::string sense_x;
    std::string sense_y;
};


class Localizer
{
private:
    double m_delta_t;
    double m_sensor_range;
    double m_sigma_pos[3];
    double m_sigma_landmark[2];
    std::vector<LandmarkObs> m_noisy_observations;

    std::unique_ptr<Map> m_map;
    std::unique_ptr<ParticleFilter> m_pf;

public:

    Localizer(double delta, double sensor_range, double sigma_pos[], double sigma_landmark[]);

    ~Localizer() {}

    OutputLocalizationMsg processLocation(InputLocalizationMsg& input);

    void updateAndResamplePF(InputLocalizationMsg& input);

    Particle getBestParticle();

};

#endif