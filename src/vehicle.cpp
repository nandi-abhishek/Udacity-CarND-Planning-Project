#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include <cassert>
#include "vehicle.h"
#include "planner.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle() : id(-2), s(-1), speed(-1) {}

Vehicle::Vehicle(int id, int lane, double x, double y, double s, double d,
          double speed, double yaw, double acc, double p_s) :
        id(id), lane(lane), x(x), y(y), s(s), d(d),
        speed(speed), yaw(yaw), acc(acc), p_s(p_s) 
{
    max_acceleration = 10; //m/s^2
    max_jerk = 10; //m/s^2
}

Vehicle::~Vehicle() {}

//Vehicle id -1 is for the vehicle we are driving
//All other vehicle has id >=0
//Id -2 is for an invalid or un-initialized vehicle
bool Vehicle::is_valid() const 
{
    return (id != -2);
}

std::ostream & operator<<(std::ostream &os, const Vehicle& v)
{
    return os << "Vehicle:= id: " << v.id << " s: " << v.s  << " d: " << v.d << " v: " << v.speed << " project_s: " << v.p_s;
}

/**
 * Initializes Road
 */
Road::Road(double speed_limit, vector<double> lane_speeds, double max_s) {
    this->num_lanes = lane_speeds.size();
    this->lane_speeds = lane_speeds;
    this->speed_limit = speed_limit;
    this->track_s = max_s;
    this->prev_path_no_points = PREV_PATH_N;
}

Road::~Road() {}

bool Road::has_car(int id) {
    auto it = vehicles.find(id);
 
    // If the key is not found in the map, just return false
    if (it == vehicles.end())
    {
        return false;
    }
    return true;        
}

Vehicle& Road::get_car(int id) 
{
    auto it = vehicles.find(id);
    return it->second;
}

void Road::update_car(Vehicle &car) 
{
    if (car.id == -1) {
        my_car = car;
    } else {
        if (has_car(car.id)) {
            Vehicle &old_car = get_car(car.id);
            old_car = car;
            //cout << "Updated " << get_car(car.id) << endl;
        } else {
            vehicles.insert(std::pair<int, Vehicle> (car.id, car));
        }
    }
}

//Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
//rVehicle is updated if a vehicle is found.
//The track is a circular one.  So, the 's' value becomes 0 after max track distance
bool Road::get_vehicle_behind(int lane, double scan_distance, Vehicle & rVehicle) {
    int max_s = -track_s;
    bool found_vehicle = false;
    for (auto pair : vehicles) {
        Vehicle vehicle = pair.second;    
        if (vehicle.lane == lane) {    
            double s = vehicle.s;
            if (my_car.s < scan_distance) {
                double backward_d = track_s + my_car.s - scan_distance;
                if (s > backward_d) {
                    s -= track_s;
                }
            }

            if (s < my_car.s &&
                (my_car.s - s) < scan_distance &&
                s > max_s) {
                max_s = s;
                rVehicle = vehicle;
                found_vehicle = true;
            }
        }
    }
    return found_vehicle;
}

//Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
//rVehicle is updated if a vehicle is found.
bool Road::get_vehicle_ahead(int lane, double scan_distance, Vehicle & rVehicle) {
    int min_s = 2 * track_s;
    bool found_vehicle = false;
    for (auto pair : vehicles) {
        Vehicle vehicle = pair.second;    
        if (vehicle.lane == lane) {    
            double s = vehicle.s;
            if (my_car.s > (track_s - scan_distance)) {
                double forward_d = my_car.s + scan_distance - track_s;
                if (s < forward_d) {
                    s += track_s;
                }
            }
            if (s > my_car.s &&
                (s - my_car.s) < scan_distance &&
                s < min_s) {
                min_s = s;
                rVehicle = vehicle;
                found_vehicle = true;
            }
        }
    }
    return found_vehicle;
}

//Returns if there is a vehicle ahead in the left/right lane within 2m 'd' value difference
bool Road::get_vehicle_ahead(bool check_lane_left, double scan_distance, Vehicle & rVehicle) {
    bool found_vehicle = false;
    int lane = my_car.d / 4;
    lane = check_lane_left ? lane - 1 : lane + 1;
    assert(lane >= 0 && lane <= 2); 
    found_vehicle = get_vehicle_ahead(lane, scan_distance, rVehicle);
    if (found_vehicle) {
        if (fabs(my_car.d - rVehicle.d) > 2) {
            found_vehicle = false;
        }
    }    
    return found_vehicle;
}

//Returns how far the back vehicle is from the front vehicle
//If projected is true then returns the projected difference in position
double Road::position_diff(Vehicle &front, Vehicle &back, bool projected) 
{
    double f_s = projected ? front.p_s : front.s;
    double b_s = projected ? back.p_s : back.s;
    if (f_s < b_s) {
        f_s += track_s;
    }
    return f_s - b_s;
}

//Retunrs future vehicle predicted location
double Road::position_at(Vehicle &v, double t) {
    double s = v.s + v.speed/2.24*t + v.acc*t*t/2.0;
    if (s > track_s) {
        s -= track_s;
    }
    return s;
}

//Generate predicted location of the passed vehicle 
////assuming it is going at constant speed instantaneously
void Road::generate_predictions(Vehicle &vehicle, int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
    double next_s = position_at(vehicle, horizon * 0.02);
    //double next_v = vehicle.speed;
    //next_v += vehicle.acc * i * 0.02;
    vehicle.p_s = next_s;
    //vehicle.speed = next_v;
}


