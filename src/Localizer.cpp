#include <Localizer.hpp>

Localizer::Localizer(double delta, double sensor_range, double sigma_pos[], double sigma_landmark[]):
m_delta_t(delta),
m_sensor_range(sensor_range),
m_sigma_pos{sigma_pos[0],sigma_pos[1],sigma_pos[2]},
m_sigma_landmark{sigma_landmark[0],sigma_landmark[1]}
{

    m_map = std::make_unique<Map>();
    m_pf = std::make_unique<ParticleFilter>();

    std::string map_file = "../data/map_data.txt";
    if (!read_map_data(map_file, *m_map)) 
    {
        std::cout << "Error: Could not open map file" << std::endl;
        return ;
    }
}

OutputLocalizationMsg Localizer::processLocation(InputLocalizationMsg& input)
{
    if(! m_pf->initialized())
    {
        // Sense noisy position data from the simulator
        m_pf->init(input.sense_x, input.sense_y, input.sense_theta, m_sigma_pos);
    }
    else
    {
        // Predict the vehicle's next state from previous (noiseless control) data.
        m_pf->prediction(m_delta_t, m_sigma_pos, input.previous_velocity, input.previous_yawrate);
    }

    updateAndResamplePF(input);
    Particle best_particle = getBestParticle();

    // populate out msg
    OutputLocalizationMsg out_msg;
    out_msg.x = best_particle.x;
    out_msg.y = best_particle.y;
    out_msg.theta = best_particle.theta;

    out_msg.association = m_pf->getAssociations(best_particle);
    out_msg.sense_x = m_pf->getSenseX(best_particle);
    out_msg.sense_y = m_pf->getSenseY(best_particle);

    return out_msg;
}

void Localizer::updateAndResamplePF(InputLocalizationMsg& input)
{
    m_noisy_observations.clear();
    for(size_t i = 0; i < input.x_sense.size(); i++)
    {
        LandmarkObs obs;
        obs.x = input.x_sense[i];
        obs.y = input.y_sense[i];
        m_noisy_observations.push_back(obs);
    }

    // Update the weights and resample
    m_pf->updateWeights(m_sensor_range, m_sigma_landmark, m_noisy_observations, *m_map);
    m_pf->resample();
}

Particle Localizer::getBestParticle()
{
    // Calculate and output the average weighted error of the particle filter over all time steps so far.
    std::vector<Particle> particles = m_pf->m_particles;
    int num_particles = particles.size();
    double highest_weight = -1.0;
    Particle best_particle;
    double weight_sum = 0.0;

    for (int i = 0; i < num_particles; ++i) 
    {
        if (particles[i].weight > highest_weight) 
        {
            highest_weight = particles[i].weight;
            best_particle = particles[i];
        }
        weight_sum += particles[i].weight;
    }

    std::cout << "[highest w ]" << highest_weight << "\t";
    std::cout << "[average w ]" << weight_sum/num_particles << std::endl;

    return best_particle;
}