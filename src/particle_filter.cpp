/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"


#define getEuclideanDist(a,b) pow(a.x - b.x,2)+pow(a.y - b.y,2)
#define YAW_THRESHOLD 0.0001

using namespace std;

void
ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    default_random_engine gen;

    num_particles = 500;

    normal_distribution<double> dist_x(x,std[0]);
    normal_distribution<double> dist_y(y,std[1]);
    normal_distribution<double> dist_theta(theta,std[2]);

    // initialize all particles to first reading
    Particle p;
    for(int i =0; i< num_particles; i++)
    {
        p.x         = dist_x(gen);
        p.y         = dist_y(gen);
        p.theta     = dist_theta(gen);
        p.weight    = 1.0;

        particles.push_back(p);
        weights.push_back(p.weight);
    }
    // set initialization flag
    is_initialized = true;

}

void
ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    default_random_engine gen;
    normal_distribution<double> dist_x(0,std_pos[0]);
    normal_distribution<double> dist_y(0,std_pos[1]);
    normal_distribution<double> dist_theta(0,std_pos[2]);


    for(Particle &p:particles)
    {
        if(yaw_rate < YAW_THRESHOLD)
        {
            p.x  += velocity*cos(p.theta)*delta_t;
            p.y  += velocity*sin(p.theta)*delta_t;
        }
        else
        {
            p.x  += velocity/yaw_rate * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
            p.y  += velocity/yaw_rate * (-cos(p.theta + yaw_rate * delta_t) + cos(p.theta));
        }
        p.theta  += yaw_rate * delta_t;

        // add gausian noise
        p.x      += dist_x(gen);
        p.y      += dist_y(gen);
        p.theta  += dist_theta(gen);
    }


}

void
ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.


    // nearest neighbour

    for(LandmarkObs &obs:observations)
    {
        obs.id = predicted[0].id;
        double min_dist = getEuclideanDist(obs,predicted[0]);

        for(LandmarkObs &pred:predicted)
        {
            const double dist = getEuclideanDist(obs,pred);
            if(dist < min_dist)
            {
                min_dist    = dist;
                obs.id      = pred.id;
            }
        }
    }


}

void
ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
    std::vector<LandmarkObs> observations_map;
        observations_map = observations; //copy to get size and ids. values replaced below
        double sensor_range_sqrd = pow(sensor_range,2);

        for (int i = 0; i < num_particles; i++) {
            //First find all landmarks that are within range
            std::vector<LandmarkObs> landmarks_in_range;
            for (size_t j = 0; j < map_landmarks.landmark_list.size(); j++)
            {
                double dist_squared = pow(particles[i].x - map_landmarks.landmark_list[j].x_f,2) +
                                      pow(particles[i].y - map_landmarks.landmark_list[j].y_f,2);
                if (dist_squared < sensor_range_sqrd)
                {
                    LandmarkObs myLandmark;
                    myLandmark.id = j;
                    myLandmark.x = map_landmarks.landmark_list[j].x_f;
                    myLandmark.y = map_landmarks.landmark_list[j].y_f;
                    landmarks_in_range.push_back(myLandmark);
                }
            }
            if (landmarks_in_range.size()>0){
                // now transform observations to map frame (to match landmarks_in_range coordinate system)
                // assuming the observations were made from the i'th particle's perspective
                std::vector<LandmarkObs> observations_map(observations.size());
                for (size_t obs_i = 0; obs_i < observations.size(); obs_i++)
                {
                    observations_map[obs_i].x = cos(particles[i].theta)*observations[obs_i].x -
                                                sin(particles[i].theta)*observations[obs_i].y +
                                                particles[i].x;


                    observations_map[obs_i].y = sin(particles[i].theta)*observations[obs_i].x +
                                                cos(particles[i].theta)*observations[obs_i].y +
                                                particles[i].y;
                }


                //get associations
                dataAssociation(landmarks_in_range, observations_map);
                //update weights
                double w = 1.0;
                double prob_const = 1.0/2.0/M_PI/std_landmark[0]/std_landmark[1];
                for (size_t obs_i = 0; obs_i < observations_map.size(); obs_i++)
                {
                    int landmark_i = observations_map[obs_i].id;
                    double x = observations_map[obs_i].x;
                    double y = observations_map[obs_i].y;
                    double ux = map_landmarks.landmark_list[landmark_i].x_f;
                    double uy = map_landmarks.landmark_list[landmark_i].y_f;
                    w*=prob_const * exp(-(pow((x - ux)/std_landmark[0],2)/2.0 +
                                          pow((y - uy)/std_landmark[1],2)/2.0));
                }
                particles[i].weight = w;
            }
            else
            {
                particles[i].weight = 0;
            }
        }

        //now normalize
        double scale_factor = 0.0;
        for (int i = 0; i < num_particles; i++)
        {
            scale_factor+=particles[i].weight;
        }

        // update weights
        for(int i = 0; i < num_particles; i++)
        {
            weights[i] = particles[i].weight/scale_factor;
        }
}

void
ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    std::vector<Particle> particles_new;
    particles_new.resize(num_particles);

    default_random_engine rand_gen; // random generator
    std::discrete_distribution<int> distribution(weights.begin(),weights.end());

    for (int i =0; i<num_particles; i++){
        particles_new[i] = particles[distribution(rand_gen)];
    }

    // update the particles
    particles = particles_new;
}

Particle
ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    //Clear the previous associations
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
}

string
ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string
ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string
ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
