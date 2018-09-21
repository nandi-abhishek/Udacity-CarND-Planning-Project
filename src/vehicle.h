#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  int id, lane;
  double x, y, s, d;
  double speed, yaw, acc;
  double max_acceleration, max_jerk;
  double p_s; 

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int id, int lane, double x, double y, double s, double d,
          double speed, double yaw, double acc, double p_s);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  bool is_valid() const;

  friend std::ostream & operator<<(std::ostream &os, const Vehicle& v);
};
 
class Road {
public:
    int num_lanes;
    vector<double> lane_speeds;
    double speed_limit;
    double track_s; 
    map<int, Vehicle> vehicles;
    Vehicle my_car;
    int prev_path_no_points;

    /**
    * Constructor
    */
    Road(double speed_limit, vector<double> lane_speeds, double track);
    /**
    * Destructor
    */
    virtual ~Road();

    bool has_car(int id);
    Vehicle& get_car(int id);
    void update_car(Vehicle &car);

    double position_diff(Vehicle &front, Vehicle &back, bool projected);
    double position_at(Vehicle &v, double t);
    void generate_predictions(Vehicle &vehicle, int horizon);
        
    bool get_vehicle_behind(int lane, double scan_distance, Vehicle & rVehicle);
    bool get_vehicle_ahead(int lane, double scan_distance, Vehicle & rVehicle);
    bool get_vehicle_ahead(double d, bool check_lane_left, double scan_distance, Vehicle & rVehicle);
};

#endif

