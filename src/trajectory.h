#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <iostream>
#include <vector>
#include "vehicle.h"

using namespace std;

class Trajectory {
public:
    vector<double> ptsx;
    vector<double> ptsy;

    Trajectory() {};
    ~Trajectory() {};

    void create_trajectory(Vehicle &my_car, vector<double> ref_kinematics, 
                    vector<double> &previous_path_x, vector<double> &previous_path_y,
                    double end_path_s, double end_path_d, vector<double> &map_waypoints_x,
                    vector<double> &map_waypoints_y, vector<double> &map_waypoints_s,
                    vector<double> &next_x_vals, vector<double> &next_y_vals);    
};
 
#endif
