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
    current_state = State::DUMMY_STATE;
    previous_path_size = 0;
    v_old_final = 0;
    max_acc = ACC;
    t_start = chrono::high_resolution_clock::now();
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

//The main planner API
//Input: Previous path and my_car s,d at previous path end
//Output: Target velocity and 'd' value sof target lane
vector<double> Planner::plan(vector<double> &previous_path_x,
                             vector<double> &previous_path_y,
                             double end_path_s,
                             double end_path_d) 
{
    previous_path_size = previous_path_x.size();

    //1. generate predicted location of all other vehicles when my_car reach end_path_s
    //Assume constant instatenuous velocity
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

    //2. Check for collision to assert - help debugging
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

    //3. Get next state with lowest cost
    State next_state = choose_next_state();

    if (current_state.type != next_state.type) {
        cout << "NEXT STATE: " << next_state << endl; 
    }
    double v_final = next_state.target_v;

    //Give preference to old_car_velocity if current velocity is not close
    double v_cur = road.my_car.speed;
    if (fabs (v_cur - v_old_final) > 0.1) {
        v_cur = v_old_final;
    }    
    //4. Calculate average acceleration required
    double avg_acc = (v_final - v_cur) / (2.24 * 0.02); //m /s^2

    //5. if avg_acc is more than max_acc - limit avg_acc to max_acc
    if (avg_acc > max_acc) {
        avg_acc = max_acc;
    } else if (avg_acc < -max_acc) {
        avg_acc = -max_acc;
    }    
    //6. Reset max_acc in case it is increased to avoid collision
    max_acc = ACC;

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

//Returns cost of taking the new 'state'
double Planner::get_cost(State state) {
    double cost = 0.0;
    //1. Actual distance cost
    cost += get_buffer_distance_cost(state, false);
    
    //2. Projected distance cost
    cost += get_buffer_distance_cost(state, true);
    
    //3. Efficieny cost - mostly how quick we can go
    cost += get_efficiency_cost(state);

    //4. Lane change extra cost
    cost += get_lane_change_extra_cost(state);

    //5. Target 'd' cost during lane change
    cost += get_target_d_cost(state);

    return cost;
}

//Returns cost of 'state' based on actual or 'projected' distance
//Distance has impact to cost only if it is less than BUFFER_DISTANCE
double Planner::get_buffer_distance_cost(State state, bool projected) {
    double cost = 0.0;
    string report_str = projected ? "projected after previous_path end " : "actual ";
    if (state.type == KL) {
        //1. If 'keep lane' state and we have a vehicle ahead within BUFFER_DISTANCE
        if (state.v_ahead.is_valid()) {
            double b_d = road.position_diff(state.v_ahead, road.my_car, projected); 
            cout << "\t\t\t Ahead: v_id: " << state.v_ahead.id << " s_diff: "  <<
                  report_str << b_d << endl;
            if (b_d < BUFFER_DISTANCE) {
                cost += exp( CRITICAL_DISTANCE - b_d);
            }
        }
    } else if (state.type == PLCL || state.type == PLCR) {
        //2. If 'prepare lane change' state and we have a vehicle ahead in current lane 
        //within BUFFER_DISTANCE
        if (state.v_ahead_cur_lane.is_valid()) {
            double b_d = road.position_diff(state.v_ahead_cur_lane, road.my_car, projected);
            cout << "\t\t\t Ahead_current_lane: v_id:  " << state.v_ahead_cur_lane.id << " s_diff: "  << 
                    report_str << b_d << endl;
            if (b_d < BUFFER_DISTANCE) {
                cost += exp( CRITICAL_DISTANCE - b_d);
            }
        }
    } else if (state.type == LCL || state.type == LCR) {
        //3. If 'lane change' state and we have a vehicle ahead/behind in target lane 
        //within BUFFER_DISTANCE. Choose behind vehicle distance if it is within
        //SEMI_CRITICAL_DISTANCE
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

//Returns cost of 'state' based on 'possible' velocity - state.possible_v
//(not current velocity - state.target_v)
//The faster the possible velocity the lower is the cost
double Planner::get_efficiency_cost(State state) {
    double cost = 0.0;
    cost = 1 - (state.possible_v / road.speed_limit);
    return cost;
}

//Lane change states have added extra cost associated with it
double Planner::get_lane_change_extra_cost(State state) {
    double cost = 0.0;
    Vehicle vehicle_ahead;
    if (current_state.type == KL &&
        state.type != KL) {
        //1. Current state is keep lane -> next 'state' is PLCL/PLCR
        //Add extra cost of 0.02. If there is a vehicle ahead in target lane
        //the extra cost increases 0.2 beased on its distance and velocity
        
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

        //2. This planner has given ann extra weightage to center lane.
        //When center lane is enpty till SCAN DISTANCE and we dont have a car
        //close enough in current lane - then reward center lane change
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
    if ((current_state.type == KL || current_state.type == PLCL || current_state.type == PLCR) && 
        state.type != KL) {
        bool forward_vehicle_found = road.get_vehicle_ahead(road.my_car.lane, BUFFER_DISTANCE, vehicle_ahead);
        if (forward_vehicle_found) {
            //3. possible state in this conditions : KL->PLCL, KL->PLCR
            //PLCL->PLCL/LCL, PLCR->PLCR/LCR
            //If more than 50 m diff with forward vehicle in target lane
            //then reward this path to allow even mph less speed
            int new_lane = road.my_car.lane + lane_direction[state.type];
            if (state.type == LCL || state.type ==  LCR) {
                new_lane = road.my_car.lane;
            }
            bool forward_vehicle_found = road.get_vehicle_ahead(new_lane, SCAN_DISTANCE, vehicle_ahead);
            if (!forward_vehicle_found || 
                road.position_diff(vehicle_ahead, road.my_car, false) > (SCAN_DISTANCE / (2 * SCAN_DISTANCE_FACTOR))) {
                cost -= 0.12; //more than 50 m diff. even if velocity is less by 5 miles opt this path
            }    
        }
    }
    if (current_state.type == PLCL || current_state.type == PLCR) {
        if (state.type == LCL || state.type == LCR) {
            //4. Given center lane weightage
            //Help to choose LCL/LCR states from prepare steps
            //when only target lane velocity is contributing to cost factor
            //this happens when car shifts from left/right lane to center lane
            //due to center lane weightage
            if (!state.v_ahead.is_valid()) {
                cost -= 0.01;
            }
        }
    }
    return cost;
}

//Reward a LCL/LCR step till target d is not close enough
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

//Returns next best state
State Planner::choose_next_state() {
    //1. Get all the successor steps from FSM
    vector<StateType> states = successor_states();
    double cost;
    vector<double> costs;
    vector<State> predicted_states;

    //2.Print state details for debugging
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

    //3.Get best state based on cost
    vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    State best_state = predicted_states[best_idx];
    if (road.my_car.lane == 1) {
        //4. If car is in middle lane then re-check distance cost for PLCL/PLCR
        //This is becuase distance impacts in get_cost only if it is within BUFFER_DISTANCE
        choose_next_LC_state(states, best_state);
    }

    //5. Final check - Common for all states
    Vehicle vehicle_ahead;
    //5.1 get forward vehicle if not found in current lane
    //then check if from next lane within 2m from my car's 'd'
    //This can happen if a vehicle ahead from next lane is turing 
    //to our target lane but not yet crossed its own lane
    bool forward_vehicle_found = road.get_vehicle_ahead(road.my_car.lane, BUFFER_DISTANCE, vehicle_ahead);
    if (!forward_vehicle_found) {
        if (best_state.type == LCL || best_state.type == LCR) {
            if (fabs(road.my_car.d - best_state.target_d) < 2) {
                //Car has just crossed the lane            
                forward_vehicle_found = road.get_vehicle_ahead(best_state.type == LCR ?
                                                                 true : false /*check_left_lane*/ ,
                                                               BUFFER_DISTANCE, vehicle_ahead);
            }
        }
    }
    if (forward_vehicle_found) {
        //5.2 If such a vehicle is found then and it is close to my_car,
        //also it is slower than my car then reduce no of point from previous path
        //based on distance. This is to react to this sudden change
        //and decelerate quickly
        double forward_distance = road.position_diff(vehicle_ahead, road.my_car, false);
        if (vehicle_ahead.speed < road.my_car.speed &&
            forward_distance < BUFFER_DISTANCE - 5) {
            double delta = BUFFER_DISTANCE - SEMI_CRITICAL_DISTANCE;
            road.prev_path_no_points = PREV_PATH_N - (PREV_PATH_N - PREV_PATH_N_CRITICAL) * 
                                        (std::min((BUFFER_DISTANCE - forward_distance),  delta) / delta);
            cout << "\t\t\t Update prev_path no of points to " << road.prev_path_no_points << endl;
        }
        //5.3 If that vehicle is very close - increase max_acc temporarily
        //Also cap target velocity to min(best_state.v,foreard_car.v,30) 
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

//If car is in center lane and best_state is PLCL/PCLR
//then re-check the other prep_lane_change cost
//It could happen during previous costing step the distance of these
//two steps didn't impact as they were > BUFFER_DISTANCE
//and we choose state only based on velocity. So, this step
//works as secondary costing step with their distance
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

//Returns lists of possible states from FSM
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

//Given a state type returns details of next_state from current_state
State Planner::get_state_details(StateType st) {
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

//Return next keep lane state details
State Planner::get_keep_lane_state() {
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    bool forward_vehicle_found = road.get_vehicle_ahead(road.my_car.lane, SCAN_DISTANCE, vehicle_ahead);
    bool backward_vehicle_found = road.get_vehicle_behind(road.my_car.lane, SCAN_DISTANCE, vehicle_behind);

    double target_v = road.speed_limit;
    State s(target_v, -1, 2 + road.my_car.lane * 4, KL);
    
    //1.Select immediate terget velocity
    double forward_distance = road.track_s;
    if (forward_vehicle_found) {    
        forward_distance = road.position_diff(vehicle_ahead, road.my_car, false);
        //1.1 If no car is within BUFFER_DISTANCE keep it to spped_limit
        //else set it to forward vehicle speed
        if (forward_distance < BUFFER_DISTANCE) {
            target_v = vehicle_ahead.speed;
           }
           //1.2 If forward car is too close select lowest velocity
           if (forward_distance < SEMI_CRITICAL_DISTANCE) {
            if (vehicle_ahead.speed > current_state.target_v)
                target_v = current_state.target_v;
            else
                target_v = vehicle_ahead.speed * 0.9;
        }
    }
    s.target_v = target_v;

    //2.Select possible future velocity in this state
    //2.1 default target velocity
    s.possible_v = target_v;
    if (forward_vehicle_found &&
         forward_distance < SCAN_DISTANCE / (2 * SCAN_DISTANCE_FACTOR)) 
    {
        //2.2 if there is a car within 50m - set it to min(car_speed, target_velocity)
        s.possible_v = std::min(vehicle_ahead.speed, target_v);
    }
    s.v_ahead = vehicle_ahead;
    s.v_ahead_cur_lane = vehicle_ahead;
    s.v_behind = vehicle_behind;
    s.v_behind_cur_lane = vehicle_behind;
    return s;
}

//Return next prepare lane change state details
State Planner::get_prep_lane_change_state(StateType st) {
    //1. Determine target lane
    int new_lane = road.my_car.lane + lane_direction[st];
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    bool forward_vehicle_found = road.get_vehicle_ahead(new_lane, SCAN_DISTANCE, vehicle_ahead);
    bool backward_vehicle_found = road.get_vehicle_behind(new_lane, SCAN_DISTANCE, vehicle_behind);

    State s(road.speed_limit, -1, 2 + road.my_car.lane * 4, st);
    Vehicle vehicle_ahead_cur_lane;
    if (road.get_vehicle_ahead(road.my_car.lane, SCAN_DISTANCE, vehicle_ahead_cur_lane)) {
        s.v_ahead_cur_lane = vehicle_ahead_cur_lane;
    }    
    Vehicle vehicle_behind_cur_lane;
    if (road.get_vehicle_behind(road.my_car.lane, SCAN_DISTANCE, vehicle_behind_cur_lane)) {
        s.v_behind_cur_lane = vehicle_behind_cur_lane;
    }

    //2. First check if we have enough gap for a lane change
    bool possible = true;
    if (s.v_ahead_cur_lane.is_valid()) {
        //2.1. Not possible if current lane ahead vehicle is too close
        if (road.position_diff(vehicle_ahead_cur_lane, road.my_car, false) < (SEMI_CRITICAL_DISTANCE)) {
            possible = false;
        }
    }
    double forward_distance = 0;
    if (possible && forward_vehicle_found) {
        forward_distance = road.position_diff(vehicle_ahead, road.my_car, false);     
        //2.1. Not possible if target lane ahead vehicle is too close or
        //target lane forward gap is less than current lane forward gap
        if (forward_distance < SEMI_CRITICAL_DISTANCE ||
            (s.v_ahead_cur_lane.is_valid() && forward_distance < road.position_diff(s.v_ahead_cur_lane, road.my_car, false))) {
            possible = false;
        }
    }
    double backward_distance = 0;
    if (possible && backward_vehicle_found) {
        backward_distance = road.position_diff(road.my_car, vehicle_behind, false);
        //2.2. Not possible if target lane behind vehicle is too close 
        if  (backward_distance < (SEMI_CRITICAL_DISTANCE)) { 
            possible = false;
            //2.3. But if the behind vehicle in target lane is going slower than my car
            //then it is possible till it is not within CRITICAL DISTANCE
            if (road.my_car.speed > vehicle_behind.speed) {
                if (backward_distance > CRITICAL_DISTANCE) {
                    //Risky condition - tests require
                    possible = true;
                }
            }
        }
    }

    //3. Return DUMMY state if not posible
    if (!possible) {
        return State::DUMMY_STATE;
    }

    //4.Select possible future velocity in this state
    //Default speed_limit
    //But, if there is a car within 100m in target lane- set it to min car_speed
    s.possible_v = road.speed_limit;
    if (forward_vehicle_found && forward_distance < SCAN_DISTANCE / SCAN_DISTANCE_FACTOR) {
        s.possible_v = vehicle_ahead.speed;
    }

    //5.Select immediate terget velocity
    //Default is whatever current_state target_v we have
    s.target_v = current_state.target_v;
    if (backward_vehicle_found && backward_distance < BUFFER_DISTANCE) {
        //5.1 If behind behicle in target lane is within BUFFER_DISTANCE try to match that velocity
        //if it is more than current target velocity
        double max_v = s.v_ahead_cur_lane.speed > vehicle_behind.speed ? s.v_ahead_cur_lane.speed : vehicle_behind.speed;
        if (max_v > s.target_v) {
            s.target_v = max_v;
        }
    }
    if (s.v_ahead_cur_lane.is_valid() && 
        road.position_diff(s.v_ahead_cur_lane, road.my_car, false) < (BUFFER_DISTANCE - 5) &&
        //5.2 If the ahead car in current lane is quite close then dont increase target_speed
        //beyond possible_velocity of target lane 
        s.target_v > s.possible_v) {
        s.target_v = s.possible_v;
    }
    s.v_ahead = vehicle_ahead;
    s.v_behind = vehicle_behind;
    return s;
}

//Return next lane change state details
State Planner::get_lane_change_state(StateType st) {
    //1. Determine target lane
    int new_lane = road.my_car.lane + lane_direction[st];
    if (current_state.type == st) {
        new_lane = current_state.target_d / 4;
    }

    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    bool forward_vehicle_found = road.get_vehicle_ahead(new_lane, SCAN_DISTANCE, vehicle_ahead);
    bool backward_vehicle_found = road.get_vehicle_behind(new_lane, SCAN_DISTANCE, vehicle_behind);

    State s(road.speed_limit, -1, 2 + new_lane * 4, st);
    
    //2.Select immediate terget velocity
    s.target_v = current_state.target_v;

    //3.Select possible future velocity in this state
    s.possible_v = road.speed_limit;
    if (forward_vehicle_found) {
        //3.2 if there is a car within 100m in target lane - set it to min car_speed
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


