#include <uWS/uWS.h>
#include <iostream>
#include <json.hpp>
#include <math.h>
#include <particle_filter.hpp>
#include <memory>
#include <vector>
#include <Localizer.hpp>
#include <socket_server.hpp>

using namespace std;

// for convenience
using json = nlohmann::json;

template <class JSON_t>
InputLocalizationMsg getDataFromJSON(JSON_t j)
{
    InputLocalizationMsg input;


    input.sense_x          = std::stod(j[1]["sense_x"].get<std::string>());
    input.sense_y          = std::stod(j[1]["sense_y"].get<std::string>());
    input.sense_theta      = std::stod(j[1]["sense_theta"].get<std::string>());


    // Predict the vehicle's next state from previous (noiseless control) data.
    input.previous_velocity    = std::stod(j[1]["previous_velocity"].get<std::string>());
    input.previous_yawrate     = std::stod(j[1]["previous_yawrate"].get<std::string>());

    // receive noisy observation data from the simulator
    // sense_observations in JSON format [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}]
    std::vector<LandmarkObs> noisy_observations;
    std::string sense_observations_x = j[1]["sense_observations_x"];
    std::string sense_observations_y = j[1]["sense_observations_y"];

    std::vector<float> x_sense;
    std::istringstream iss_x(sense_observations_x);

    std::copy(std::istream_iterator<float>(iss_x),
    std::istream_iterator<float>(),
    std::back_inserter(x_sense));

    std::vector<float> y_sense;
    std::istringstream iss_y(sense_observations_y);

    std::copy(std::istream_iterator<float>(iss_y),
    std::istream_iterator<float>(),
    std::back_inserter(y_sense));

    input.x_sense = x_sense;
    input.y_sense = y_sense;

    return input;
}

template <class JSON_t>
JSON_t getJSONFromData(OutputLocalizationMsg out_msg)
{
    json json_msg;
    json_msg["best_particle_x"]     = out_msg.x;
    json_msg["best_particle_y"]     = out_msg.y;
    json_msg["best_particle_theta"] = out_msg.theta;

    //Optional message data used for debugging particle's sensing and associations
    json_msg["best_particle_associations"]  = out_msg.association;
    json_msg["best_particle_sense_x"]       = out_msg.sense_x;
    json_msg["best_particle_sense_y"]       = out_msg.sense_y;
    
    return json_msg;
}


int main()
{
    // uWS::Hub h;

    // //Set up parameters here
    double delta_t        = 0.1; // Time elapsed between measurements [sec]
    double sensor_range   = 50; // Sensor range [m]

    double sigma_pos [3]      = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
    double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]


    Localizer localizer(delta_t,sensor_range,sigma_pos,sigma_landmark);

    SocketServer socket_server(4567);

    socket_server.registerOnMessageFunction(
        [&localizer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
        {
            // "42" at the start of the message means there's a websocket message event.
            // The 4 signifies a websocket message
            // The 2 signifies a websocket event

            if (length && length > 2 && data[0] == '4' && data[1] == '2')
            {
                auto s = hasData(std::string(data));
                if (s != "") 
                {	
                    auto j = json::parse(s);
                    std::string event = j[0].get<std::string>();
                    
                    if (event == "telemetry") 
                    {
                        InputLocalizationMsg input_msg  = getDataFromJSON<decltype(j)>(j);
                        OutputLocalizationMsg out_msg   = localizer.processLocation(input_msg);

                        auto out_json = getJSONFromData<decltype(j)>(out_msg);
                        auto msg = "42[\"best_particle\"," + out_json.dump() + "]";
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }
                }
                else 
                {
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
        }
    );


    auto exit_status = socket_server.runServer();
    return exit_status;


}























































































