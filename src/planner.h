#ifndef PLANNER_H
#define PLANNER_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <chrono>
#include <iterator>
#include "vehicle.h"

#define CRITICAL_DISTANCE 10
#define BUFFER_DISTANCE 30
#define PREV_PATH_N 35

#define CRT_DST_FACTOR 1.5
#define SEMI_CRITICAL_DISTANCE CRT_DST_FACTOR * CRITICAL_DISTANCE
#define SCAN_DISTANCE 500
#define SCAN_DISTANCE_FACTOR 5
#define MAX_ACC 7.5
#define PREV_PATH_N_CRITICAL 5

using namespace std;

typedef enum {
        ST = 0, //Start State. Also used for dummy state
        KL,        //Keep lane state
        PLCL,    //Prepare lane change left 
        LCL,    //Lane change left
        PLCR,    //Prepare lane change right
        LCR        //Lane change right
} StateType;

class State 
{
public:
    static map<int, string> state_name;
    static State DUMMY_STATE;
    Vehicle v_ahead;
    Vehicle v_behind;
    Vehicle v_ahead_cur_lane;
    Vehicle v_behind_cur_lane;
    double target_v;
    double target_s;
    double target_d;
    double possible_v;
    StateType type;
    
    State() : target_v(0), target_s(0), target_d(0), type(ST) {}
    State(double v, double s, double d, StateType st) : target_v(v), target_s(s), target_d(d), type(st) {}
    ~State() {}
    friend std::ostream & operator<<(std::ostream &os, const State& st);
};

class Planner {
public:
    map<StateType, int> lane_direction = {{PLCL, -1}, {LCL, -1}, {LCR, 1}, {PLCR, 1}};

    State current_state;
    int previous_path_size;
    double v_old_final;
    double max_acc;
    Road &road;
	chrono::high_resolution_clock::time_point t_start;
 
    /**
    * Constructor
    */
    Planner(Road &r);
    /**
    * Destructor
    */
    virtual ~Planner();

	void print_time();
    vector<double> plan(vector<double> &previous_path_x, vector<double> &previous_path_y,
                        double end_path_s, double end_path_d);
    State choose_next_state();
    void choose_next_LC_state(vector<StateType>& states, State &best_state);
    vector<StateType> successor_states();
    State get_state_details(StateType st);

    State get_keep_lane_state();
    State get_lane_change_state(StateType st);
    State get_prep_lane_change_state(StateType st);

    double get_cost(State state);
    double get_buffer_distance_cost(State state, bool projected);
    double get_efficiency_cost(State state);
    double get_lane_change_extra_cost(State state);
    double get_target_d_cost(State state);

};

#endif
