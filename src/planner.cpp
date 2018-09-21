#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>
#include <cassert>
#include "utils.h"
#include "planner.h"

State State::DUMMY_STATE(-1, -1, -1, ST);
map<int, string> State::state_name = {{0, "ST"}, {1, "KL"}, {2, "PLCL"}, {3, "LCL"}, {4, "PLCR"}, {5, "LCR"}};

/**
 * Initializes Planner
 */
Planner::Planner(Road &r) : road(r) {
    this->current_state = State::DUMMY_STATE;
    this->previous_path_size = 0;
    this->v_old_final = 0;
    this->max_acc = MAX_ACC;
    this->t_start = chrono::high_resolution_clock::now();
}

Planner::~Planner() {}

std::ostream & operator<<(std::ostream &os, const State& s)
{
    os << "State: " << s.state_name[s.type] << " Lane: " << (int) (s.target_d / 4) << " d: "  << s.target_d << " Velocity: target " << s.target_v
             << " possible " << s.possible_v ;
    if (s.v_ahead.is_valid()) {
        os << endl;
        os << "\t\tAhead " << s.v_ahead;
    }    
    if (s.v_behind.is_valid()) {
        os << endl;
        os << "\t\tBehind " << s.v_behind;
    }    
    if (s.v_ahead_cur_lane.is_valid() &&
        s.v_ahead.id != s.v_ahead_cur_lane.id) {
        os << endl;
        os << "\t\tAhead current lane " << s.v_ahead_cur_lane;
    }    
    return os;
}

void Planner::print_time() {
    chrono::high_resolution_clock::time_point t = chrono::high_resolution_clock::now();
    chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(t - t_start);
    cout << "Time: " << time_span.count() << " seconds" << endl;
}

vector<double> Planner::plan(vector<double> &previous_path_x,
                             vector<double> &previous_path_y,
                             double end_path_s,
                             double end_path_d) 
{
    previous_path_size = previous_path_x.size();

    map<int, Vehicle>::iterator it = road.vehicles.begin();
    while(it != road.vehicles.end())
    {
        int v_id = it->first;
        if (v_id == -1) { 
            it++;
            continue;
        }
        road.generate_predictions(it->second, previous_path_size); 
        it++;
    }

    //Check for collision to assert - help debugging
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    bool forward_vehicle_found = road.get_vehicle_ahead(road.my_car.lane, SCAN_DISTANCE, vehicle_ahead);
    bool backward_vehicle_found = road.get_vehicle_behind(road.my_car.lane, SCAN_DISTANCE, vehicle_behind);
    
    if ((vehicle_ahead.is_valid() && road.position_diff(vehicle_ahead, road.my_car, false) < 5) ||
        (vehicle_behind.is_valid() &&  road.position_diff(road.my_car, vehicle_behind, false) < 5)) {
        cout << "================================" << endl;
        cout << "CRASH" << endl;
        cout << "================================" << endl;
        print_time();    
        cout << "My_car:= " << road.my_car << endl;
        cout << "Ahead vehicle:= " << vehicle_ahead << endl;
        cout << "Behind vehicle:= " << vehicle_behind << endl;
        //assert(0);
    }

    State next_state = choose_next_state();

    if (current_state.type != next_state.type) {
        cout << "NEXT STATE: " << next_state << endl; 
    }
    double v_final = next_state.target_v;

    double v_cur = road.my_car.speed;
    if (fabs (v_cur - v_old_final) > 0.1) {
        v_cur = v_old_final;
    }    
    double avg_acc = (v_final - v_cur) / (2.24 * 0.02); //m /s^2

    if (avg_acc > max_acc) {
        avg_acc = max_acc;
    } else if (avg_acc < -max_acc) {
        avg_acc = -max_acc;
    }    
    //Reset max_acc in case it is increased to avoid collision
    max_acc = MAX_ACC;

    v_final = v_cur / 2.24 + avg_acc * 0.02 ;
    v_old_final = v_final * 2.24;
    current_state = next_state;

#if 0
    cout << "target velocity: " <<  next_state.target_v;
    cout << " final velocity: " << v_final * 2.24;
    cout << " actual velocity: " << road.my_car.speed;
    cout << " acc: " << avg_acc << endl;
#endif
    cout << "\t\tSelected : Velocity: " << current_state.target_v << " D: " << next_state.target_d << endl;
    return {v_final, next_state.target_d};
}

double Planner::get_cost(State state) {
    double cost = 0.0;
    cost += get_buffer_distance_cost(state, false);
    cost += get_buffer_distance_cost(state, true);
    cost += get_efficiency_cost(state);
    cost += get_lane_change_extra_cost(state);
    cost += get_target_d_cost(state);
    return cost;
}

double Planner::get_buffer_distance_cost(State state, bool projected) {
    double cost = 0.0;
    string report_str = projected ? "projected after previous_path end " : "actual ";
    if (state.type == KL) {
        if (state.v_ahead.is_valid()) {
            double b_d = road.position_diff(state.v_ahead, road.my_car, projected); 
            cout << "\t\t\t Ahead: v_id: " << state.v_ahead.id << " s_diff: "  <<
                  report_str << b_d << endl;
            if (b_d < BUFFER_DISTANCE) {
                cost += exp( CRITICAL_DISTANCE - b_d);
            }
        }
    } else if (state.type == PLCL || state.type == PLCR) {
        if (state.v_ahead_cur_lane.is_valid()) {
            double b_d = road.position_diff(state.v_ahead_cur_lane, road.my_car, projected);
            cout << "\t\t\t Ahead_current_lane: v_id:  " << state.v_ahead_cur_lane.id << " s_diff: "  << 
                    report_str << b_d << endl;
            if (b_d < BUFFER_DISTANCE) {
                cost += exp( CRITICAL_DISTANCE - b_d);
            }
        }
    } else if (state.type == LCL || state.type == LCR) {
        Vehicle compare_vehicle = state.v_ahead;
        double b_d =  road.position_diff(state.v_ahead, road.my_car, projected);
        if (state.v_behind.is_valid()) {
            double tmp_d = road.position_diff(road.my_car, state.v_behind, projected);
            if (tmp_d < b_d && tmp_d < SEMI_CRITICAL_DISTANCE &&
                road.my_car.speed < state.v_behind.speed) {
                compare_vehicle = state.v_behind;
                b_d = tmp_d; 
            }
        }        
        if (compare_vehicle.is_valid()) {
            cout << "\t\t\tCompare: v_id: " << compare_vehicle.id << " s_diff: "  << 
                    report_str << b_d << endl;
            if (b_d < BUFFER_DISTANCE) {
                cost += exp( CRITICAL_DISTANCE - b_d);
            }
        }
    }
    return cost;
}

double Planner::get_efficiency_cost(State state) {
    double cost = 0.0;
    cost = 1 - (state.possible_v / road.speed_limit);
    return cost;
}

double Planner::get_lane_change_extra_cost(State state) {
    double cost = 0.0;
    Vehicle vehicle_ahead;
    if (current_state.type == KL &&
        state.type != KL) {
        int new_lane = road.my_car.lane + lane_direction[state.type];
        bool forward_vehicle_found = road.get_vehicle_ahead(new_lane, SCAN_DISTANCE / (2 * SCAN_DISTANCE_FACTOR), vehicle_ahead);
        if (vehicle_ahead.is_valid()) {
            double distance = road.position_diff(vehicle_ahead, road.my_car, false);
            //If <=30 m difference min velocity difference 10 mi/h
            //If 50 m difference min velocity difference 2 mi/h
            cost += 0.2 - 0.16 * (std::max((distance - BUFFER_DISTANCE), 0.0) /
                                      (SCAN_DISTANCE / (2 * SCAN_DISTANCE_FACTOR) - BUFFER_DISTANCE));
        } else {
            //If > 50 m difference min velocity difference 1 mi/h
            cost += 0.02; //fixed cost
        }
        if (road.my_car.lane != 1) {
            if ((!current_state.v_ahead.is_valid() || 
                 road.position_diff(current_state.v_ahead, road.my_car, false) >  BUFFER_DISTANCE) && 
                !state.v_ahead.is_valid()) {
                if (!state.v_behind.is_valid() ||
                    road.position_diff(road.my_car, state.v_behind, false) >  BUFFER_DISTANCE) {
                    cost -= 0.04;
                }
            }
        }
    }
    if (state.type != KL) {
        bool forward_vehicle_found = road.get_vehicle_ahead(road.my_car.lane, BUFFER_DISTANCE, vehicle_ahead);
        if (forward_vehicle_found) {
            int new_lane = road.my_car.lane + lane_direction[state.type];
            bool forward_vehicle_found = road.get_vehicle_ahead(new_lane, SCAN_DISTANCE, vehicle_ahead);
            if (!forward_vehicle_found || 
                road.position_diff(vehicle_ahead, road.my_car, false) > (SCAN_DISTANCE / (2 * SCAN_DISTANCE_FACTOR))) {
                cost -= 0.12; //more than 50 m diff. even if velocity is less by 5 miles opt this path
            }    
        }
    }
    if (current_state.type == PLCL || current_state.type == PLCR) {
        if (state.type == LCL || state.type == LCR) {
            if (!state.v_ahead.is_valid()) {
                cost -= 0.01;
            }
        }
    }
    return cost;
}

double Planner::get_target_d_cost(State state) {
    double cost = 0.0;
    if ((current_state.type == LCL && state.type == LCL) ||
        (current_state.type == LCR && state.type == LCR)) {
        //Reward this step till target_d is not met;
        cout << "\t\t\tDifference in d " << fabs(state.target_d - road.my_car.d) << endl; 
        if (fabs(state.target_d - road.my_car.d) > 0.5) {
               cost = -1; //Fixed cost - help to keep it in lane change state 
        }
    }
    return cost;
}

State Planner::choose_next_state() {
    vector<StateType> states = successor_states();
    double cost;
    vector<double> costs;
    vector<State> predicted_states;

    cout << "-----------------------------------" << endl;
    cout << "\t";
    print_time();
    cout << "\tMy_car: " << road.my_car << endl;
    cout << "\tCurrent " << current_state << endl;
    for (vector<StateType>::iterator it = states.begin(); it != states.end(); ++it) {
        State predicted_st = get_state_details(*it);
        cout << "\t" << predicted_st << endl;
        if (predicted_st.target_v != -1) {
            cost = get_cost(predicted_st);
            cout << "\t\t cost: " << cost << endl;
            costs.push_back(cost);
            predicted_states.push_back(predicted_st);
        }
    }

    vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    State best_state = predicted_states[best_idx];
    if (road.my_car.lane == 1) {
        choose_next_LC_state(states, best_state);
    }
    //Final check - Common for all states
    Vehicle vehicle_ahead;
    bool forward_vehicle_found = road.get_vehicle_ahead(road.my_car.lane, BUFFER_DISTANCE, vehicle_ahead);
    if (!forward_vehicle_found) {
        if (best_state.type == LCL || best_state.type == LCR) {
            if (fabs(road.my_car.d - best_state.target_d) < 2) {
                //Car has just crossed the lane            
                forward_vehicle_found = road.get_vehicle_ahead(road.my_car.d,
                            best_state.type == LCR ? true : false /*check_left_lane*/ ,
                            BUFFER_DISTANCE, vehicle_ahead);
            }
        }
    }
    if (forward_vehicle_found) {
        double forward_distance = road.position_diff(vehicle_ahead, road.my_car, false);
        if (vehicle_ahead.speed < road.my_car.speed &&
            forward_distance < BUFFER_DISTANCE - 5) {
            double delta = BUFFER_DISTANCE - SEMI_CRITICAL_DISTANCE;
            road.prev_path_no_points = PREV_PATH_N - (PREV_PATH_N - PREV_PATH_N_CRITICAL) * 
                                        (std::min((BUFFER_DISTANCE - forward_distance),  delta) / delta);
            cout << "\t\t\t Update prev_path no of points to " << road.prev_path_no_points << endl;
        }
        if (forward_distance <  SEMI_CRITICAL_DISTANCE) {
            if (vehicle_ahead.speed < best_state.target_v) {
                best_state.target_v = vehicle_ahead.speed;
                //Increase max acc to avoid collision
                max_acc = 10;
                cout << "\t\t\t Too close to forward vehicle in this state. Update max acc temporarily to " << max_acc << endl;
            }
            //Even now if selected speed is more than 30 reduce it
            if (best_state.target_v > 30) {
                //Reduce to 30 -- too close for higher speed
                best_state.target_v = 30;
            }    
        }
    }        
    
    return best_state;
}

void Planner::choose_next_LC_state(vector<StateType>& states, State &best_state)
{
    State other_state;
    if (best_state.type == PLCL) {
        for (vector<StateType>::iterator it = states.begin(); it != states.end(); ++it) {
            State st = get_state_details(*it);
            if (st.type == PLCR) {
                other_state = st;
                break;
            }
        }
    }
    if (best_state.type == PLCR) {
        for (vector<StateType>::iterator it = states.begin(); it != states.end(); ++it) {
            State st = get_state_details(*it);
            if (st.type == PLCL) {
                other_state = st;
                break;
            }
        }
    }
    if (other_state.type != ST) {
        //Compare distance to next car for these two states
        double best_state_distance = best_state.v_ahead.is_valid() ? 
                            road.position_diff(best_state.v_ahead, road.my_car, false) :
                            SCAN_DISTANCE;
        double other_state_distance = other_state.v_ahead.is_valid() ? 
                            road.position_diff(other_state.v_ahead, road.my_car, false) :
                            SCAN_DISTANCE;
        double distance_diff = other_state_distance - best_state_distance;
        if (other_state_distance > SCAN_DISTANCE / SCAN_DISTANCE_FACTOR &&
            distance_diff > 0) {
            best_state = other_state;
        }
    }
}

vector<StateType> Planner::successor_states() {
    vector<StateType> states;
    states.push_back(KL);
    if (current_state.type == KL) {
        if (road.my_car.lane != 0) {
          states.push_back(PLCL);
        }
        if (road.my_car.lane != 2) {
          states.push_back(PLCR);
        }
    } else if (current_state.type == PLCL) {
        states.push_back(PLCL);
        states.push_back(LCL);
    } else if (current_state.type == PLCR) {
        states.push_back(PLCR);
        states.push_back(LCR);
    } else if (current_state.type == LCL) {
        states.push_back(LCL);
    } else if (current_state.type == LCR) {
        states.push_back(LCR);
    } 
    return states;
}

State Planner::get_state_details(StateType st) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    State s;
    if (st == ST) {
        assert(0); //Can not reach here
    } else if (st == KL) {
        s = get_keep_lane_state();
    } else if (st == LCL || st == LCR) {
        s = get_lane_change_state(st);
    } else if (st == PLCL || st == PLCR) {
        s = get_prep_lane_change_state(st);
    }
    return s;
}

State Planner::get_keep_lane_state() {
    /*
    Generate a keep lane trajectory.
    */
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    bool forward_vehicle_found = road.get_vehicle_ahead(road.my_car.lane, SCAN_DISTANCE, vehicle_ahead);
    bool backward_vehicle_found = road.get_vehicle_behind(road.my_car.lane, SCAN_DISTANCE, vehicle_behind);

    double target_v = road.speed_limit;
    State s(target_v, -1, 2 + road.my_car.lane * 4, KL);
    
	double forward_distance = road.track_s;
    if (forward_vehicle_found) {    
        forward_distance = road.position_diff(vehicle_ahead, road.my_car, false);
        if (forward_distance < BUFFER_DISTANCE) {
            target_v = vehicle_ahead.speed;
           }
           if (forward_distance < SEMI_CRITICAL_DISTANCE) {
            if (vehicle_ahead.speed > current_state.target_v)
                target_v = current_state.target_v;
            else
                target_v = vehicle_ahead.speed * 0.9;
        }
    }
    s.target_v = target_v;
	s.possible_v = road.speed_limit;
	if (forward_vehicle_found && forward_distance < SCAN_DISTANCE / (2 * SCAN_DISTANCE_FACTOR)) 
	{
        s.possible_v = vehicle_ahead.speed;
	} else if (forward_distance < BUFFER_DISTANCE) {
       	s.possible_v = target_v;
	}
    s.v_ahead = vehicle_ahead;
    s.v_ahead_cur_lane = vehicle_ahead;
    s.v_behind = vehicle_behind;
    s.v_behind_cur_lane = vehicle_behind;
    return s;
}

State Planner::get_prep_lane_change_state(StateType st) {
    int new_lane = road.my_car.lane + lane_direction[st];
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    bool forward_vehicle_found = road.get_vehicle_ahead(new_lane, SCAN_DISTANCE, vehicle_ahead);
    bool backward_vehicle_found = road.get_vehicle_behind(new_lane, SCAN_DISTANCE, vehicle_behind);

    bool possible = true;
    State s(road.speed_limit, -1, 2 + road.my_car.lane * 4, st);
    Vehicle vehicle_ahead_cur_lane;
    if (road.get_vehicle_ahead(road.my_car.lane, SCAN_DISTANCE, vehicle_ahead_cur_lane)) {
        s.v_ahead_cur_lane = vehicle_ahead_cur_lane;
        if (road.position_diff(vehicle_ahead_cur_lane, road.my_car, false) < (SEMI_CRITICAL_DISTANCE)) {
            possible = false;
        }
    }
    Vehicle vehicle_behind_cur_lane;
    if (road.get_vehicle_behind(road.my_car.lane, SCAN_DISTANCE, vehicle_behind_cur_lane)) {
        s.v_behind_cur_lane = vehicle_behind_cur_lane;
    }

    double forward_distance = 0;
    if (possible && forward_vehicle_found) {
        forward_distance = road.position_diff(vehicle_ahead, road.my_car, false);     
        if (forward_distance < SEMI_CRITICAL_DISTANCE ||
            (s.v_ahead_cur_lane.is_valid() && forward_distance < road.position_diff(s.v_ahead_cur_lane, road.my_car, false))) {
            possible = false;
        }
    }
    double backward_distance = 0;
    if (possible && backward_vehicle_found) {
        backward_distance = road.position_diff(road.my_car, vehicle_behind, false);
        if  (backward_distance < (SEMI_CRITICAL_DISTANCE)) { 
            possible = false;
            if (road.my_car.speed > vehicle_behind.speed) {
                if (backward_distance > CRITICAL_DISTANCE) {
                    //Risky condition - tests require
                    possible = true;
                }
            }
        }
    }

    if (!possible) {
        return State::DUMMY_STATE;
    }

    s.possible_v = road.speed_limit;
    s.target_v = current_state.target_v;
    if (forward_vehicle_found && forward_distance < SCAN_DISTANCE / SCAN_DISTANCE_FACTOR) {
        s.possible_v = vehicle_ahead.speed;
    }
    if (backward_vehicle_found && backward_distance < BUFFER_DISTANCE) {
        double max_v = s.v_ahead_cur_lane.speed > vehicle_behind.speed ? s.v_ahead_cur_lane.speed : vehicle_behind.speed;
        if (max_v > s.target_v) {
            s.target_v = max_v;
        }
    }
    if (s.v_ahead_cur_lane.is_valid() && 
        road.position_diff(s.v_ahead_cur_lane, road.my_car, false) < (BUFFER_DISTANCE - 5) &&
        s.target_v > s.possible_v) {
        s.target_v = s.possible_v;
    }
    s.v_ahead = vehicle_ahead;
    s.v_behind = vehicle_behind;
    return s;
}

State Planner::get_lane_change_state(StateType st) {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = road.my_car.lane + lane_direction[st];
    if (current_state.type == st) {
        new_lane = current_state.target_d / 4;
    }

    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    bool forward_vehicle_found = road.get_vehicle_ahead(new_lane, SCAN_DISTANCE, vehicle_ahead);
    bool backward_vehicle_found = road.get_vehicle_behind(new_lane, SCAN_DISTANCE, vehicle_behind);

    State s(road.speed_limit, -1, 2 + new_lane * 4, st);
    s.possible_v = road.speed_limit;
    s.target_v = current_state.target_v;
    if (forward_vehicle_found) {
        if (road.position_diff(vehicle_ahead, road.my_car, false) < SCAN_DISTANCE / SCAN_DISTANCE_FACTOR) {
            s.possible_v = vehicle_ahead.speed;
        }
    }

    s.v_ahead = vehicle_ahead;
    s.v_ahead_cur_lane = vehicle_ahead;
    s.v_behind = vehicle_behind;
    s.v_behind_cur_lane = vehicle_behind;
    return s;
}


