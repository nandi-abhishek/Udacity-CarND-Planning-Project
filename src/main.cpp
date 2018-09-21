#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "utils.h"
#include "vehicle.h"
#include "planner.h"
#include "trajectory.h"

using namespace std;

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554; //m
    double speed_limit = 49.5; //mph 
    vector<double> lane_speeds = {speed_limit, speed_limit, speed_limit};

    Road road(speed_limit, lane_speeds, max_s);
    Planner my_planner(road);
    Trajectory tj;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    h.onMessage([&road, &my_planner, &tj, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                         uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);
                
                string event = j[0].get<string>();
                
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    
                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    vector<double> previous_path_x = j[1]["previous_path_x"];
                    vector<double> previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values 
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    if (previous_path_x.size() > road.prev_path_no_points) {
                        previous_path_x.resize(road.prev_path_no_points);
                        previous_path_y.resize(road.prev_path_no_points);
                        vector<double> sd = getFrenet(previous_path_x.back(), previous_path_y.back(), car_yaw, map_waypoints_x, map_waypoints_y);
                        end_path_s = sd[0];
                        end_path_d = sd[1];
                    }
                    //Reset prev_path_no_points if changed in previous iteration
                    road.prev_path_no_points = PREV_PATH_N;

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    Vehicle my_car(-1, car_d / 4, car_x, car_y, car_s, car_d, car_speed, car_yaw, 0, end_path_s);
                    road.update_car(my_car);

                    for (int i = 0; i < sensor_fusion.size(); i++)
                    { 
                        int id = sensor_fusion[i][0];
                        double x = sensor_fusion[i][1];
                        double y = sensor_fusion[i][2];
                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double check_car_yaw = atan2(vy, vx);
                        double check_car_speed = sqrt(vx*vx+vy*vy) * 2.24;
                        double check_car_s = sensor_fusion[i][5];
                        double check_car_d = sensor_fusion[i][6];
                        //Sometimes noticed simulator returning huge impossible velocities
                        //Filter those noisy velocities
                        if (check_car_speed > road.speed_limit) 
                            check_car_speed =  road.speed_limit;
                        //Assert garbage location of vehicles
                        //Otherwise this would cause collision even if there is no car present
                        //- Simulator bug
                        Vehicle car(id, check_car_d / 4, x, y, check_car_s, check_car_d, check_car_speed, check_car_yaw, 0, 0);
                        if (check_car_s == 0 && check_car_d == 0) {
                            cout << "\tSIMULATOR BUG - GARBAGE VEHICLE DATA" << endl;
                            cout << "\t" << car << endl;
                            //assert(0);
                        }
                        road.update_car(car);

                    }

                    //Call planner to get next target velocity and target 'd' location
                    vector<double> ref_kinematics = my_planner.plan(previous_path_x, previous_path_y,
                                                                    end_path_s, end_path_d);

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
            
                    //Generate trajectory using target_v and target_d using spline 
                    tj.create_trajectory(my_car, ref_kinematics, 
                        previous_path_x, previous_path_y,
                        end_path_s, end_path_d, map_waypoints_x,
                        map_waypoints_y, map_waypoints_s, 
                        next_x_vals, next_y_vals);

                    json msgJson;

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                                         size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                                                 char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
