/*
 * helper_functions.h
 * Some helper functions for the 2D particle filter.
 *  Created on: Dec 13, 2016
 *      Author: Tiffany Huang
 * 		Modified: Ameya Wagh
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map.hpp>
#include <json.hpp>
// #include <Localizer.hpp>
// #include <particle_filter.hpp>
// #include <map.hpp>

// for portability of M_PI (Vis Studio, MinGW, etc.)
#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif

/*
 * Struct representing one position/control measurement.
 */
struct control_s 
{
	
	double velocity;	// Velocity [m/s]
	double yawrate;		// Yaw rate [rad/s]
};

/*
 * Struct representing one ground truth position.
 */
struct ground_truth 
{
	
	double x;		// Global vehicle x position [m]
	double y;		// Global vehicle y position
	double theta;	// Global vehicle yaw [rad]
};

/*
 * Struct representing one landmark observation measurement.
 */
struct LandmarkObs 
{
	
	int id;				// Id of matching landmark in the map.
	double x;			// Local (vehicle coordinates) x position of landmark observation [m]
	double y;			// Local (vehicle coordinates) y position of landmark observation [m]
};

/**
 * @brief Computes the Euclidean distance between two 2D points.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two 2D points
 */
inline double dist(double x1, double y1, double x2, double y2);

inline double * getError(	double gt_x, double gt_y, 
							double gt_theta, double pf_x, 
							double pf_y, double pf_theta);

/**
 * @brief Reads map data from a file.
 * @param filename Name of file containing map data.
 * @output True if opening and reading file was successful
 */

bool read_map_data(std::string filename, Map& map);

/**
 * @brief Reads control data from a file.
 * @param filename Name of file containing control measurements.
 * @output True if opening and reading file was successful
 */
inline bool read_control_data(	std::string filename, std::vector<control_s>& position_meas); 

/**
 * @brief Reads ground truth data from a file.
 * @param filename Name of file containing ground truth.
 * @output True if opening and reading file was successful
 */
inline bool read_gt_data(	std::string filename, std::vector<ground_truth>& gt); 

/**
 * @brief Reads landmark observation data from a file.
 * @param filename Name of file containing landmark observation measurements.
 * @output True if opening and reading file was successful
 */
inline bool read_landmark_data(std::string filename, std::vector<LandmarkObs>& observations); 


/**
 * @brief Checks if the SocketIO event has JSON data. 
 * If there is data the JSON object in string format will be returned, 
 * else the empty string "" will be returned.
 * 
 * @param s 
 * @return std::string 
 */
std::string hasData(std::string s); 

// // for convenience
// using json = nlohmann::json;

// template <class JSON_t>
// InputLocalizationMsg getDataFromJSON(JSON_t j)
// {
//     InputLocalizationMsg input;


//     input.sense_x          = std::stod(j[1]["sense_x"].get<std::string>());
//     input.sense_y          = std::stod(j[1]["sense_y"].get<std::string>());
//     input.sense_theta      = std::stod(j[1]["sense_theta"].get<std::string>());


//     // Predict the vehicle's next state from previous (noiseless control) data.
//     input.previous_velocity    = std::stod(j[1]["previous_velocity"].get<std::string>());
//     input.previous_yawrate     = std::stod(j[1]["previous_yawrate"].get<std::string>());

//     // receive noisy observation data from the simulator
//     // sense_observations in JSON format [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}]
//     std::vector<LandmarkObs> noisy_observations;
//     std::string sense_observations_x = j[1]["sense_observations_x"];
//     std::string sense_observations_y = j[1]["sense_observations_y"];

//     std::vector<float> x_sense;
//     std::istringstream iss_x(sense_observations_x);

//     std::copy(std::istream_iterator<float>(iss_x),
//     std::istream_iterator<float>(),
//     std::back_inserter(x_sense));

//     std::vector<float> y_sense;
//     std::istringstream iss_y(sense_observations_y);

//     std::copy(std::istream_iterator<float>(iss_y),
//     std::istream_iterator<float>(),
//     std::back_inserter(y_sense));

//     input.x_sense = x_sense;
//     input.y_sense = y_sense;

//     return input;
// }

// template <class JSON_t>
// JSON_t getJSONFromData(OutputLocalizationMsg out_msg)
// {
//     json json_msg;
//     json_msg["best_particle_x"]     = out_msg.x;
//     json_msg["best_particle_y"]     = out_msg.y;
//     json_msg["best_particle_theta"] = out_msg.theta;

//     //Optional message data used for debugging particle's sensing and associations
//     json_msg["best_particle_associations"]  = out_msg.association;
//     json_msg["best_particle_sense_x"]       = out_msg.sense_x;
//     json_msg["best_particle_sense_y"]       = out_msg.sense_y;
    
//     return json_msg;
// }

#endif 