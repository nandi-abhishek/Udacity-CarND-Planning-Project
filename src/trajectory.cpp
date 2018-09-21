#include "trajectory.h"
#include "utils.h"
#include "spline.h"


//Generate trajectory using target_v and target_d using spline
//Input: my_car - Vehicle that is being driven
//    ref_kinematics - contains target_v and target_d
//    previous path details
//    map waypoints
//Output: Spline trajectory x,y points in next_x_vals and next_y_vals
//
//Note: points of previous path are also part of new trajectry 
void Trajectory::create_trajectory(Vehicle &my_car, vector<double> ref_kinematics, 
                    vector<double> &previous_path_x, vector<double> &previous_path_y,
                    double end_path_s, double end_path_d, vector<double> &map_waypoints_x,
                    vector<double> &map_waypoints_y, vector<double> &map_waypoints_s,
                    vector<double> &next_x_vals, vector<double> &next_y_vals)
{    
    double car_s = my_car.s;
    int prev_size = previous_path_x.size();

    if (prev_size > 0) 
    {
        car_s = end_path_s;
    }

    double v_final = ref_kinematics[0];
    int lane = (int) (ref_kinematics[1] / 4);

    //Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
    //later we will interpolate these waypoints with a spline and fill it in with more points that control the spline
    vector<double> ptsx;
    vector<double> ptsy;

    //refernce x,y, yaw states
    //either we will reference the starting point as where the car is or at the previous paths end point
    double ref_x = my_car.x;
    double ref_y = my_car.y;
    double ref_yaw = deg2rad(my_car.yaw);

    //if previous size is almost enpty, use the car as starting reference
    if (prev_size < 2)
    {
        //Use two points that make the path tangent to the car
        double prev_car_x = my_car.x - cos(my_car.yaw);
        double prev_car_y = my_car.y - sin(my_car.yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(my_car.x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(my_car.y);
    }
    //use the previous path's end point as starting reference
    else 
    {
        //Redefine reference state as previous path end points
        ref_x = previous_path_x[prev_size -1 ];
        ref_y = previous_path_y[prev_size -1 ];

        double ref_x_prev = previous_path_x[prev_size -2 ];        
        double ref_y_prev = previous_path_y[prev_size -2 ];        
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }    

    //In Ferent add evenly 30m spaced points ahead of the starting ref
    vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); i++)
    {
        //shift car reference angle to 0 degrees
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // create a spline
    tk::spline s;
    //set (x, y) points to the spline
    s.set_points(ptsx, ptsy);
    
    //Start with all of the previous path points from last time
    for (int i = 0; i < previous_path_x.size(); i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
   
    //Calculate how to break up spline points so that we travel at our desired ref velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x+ target_y*target_y);

    double x_add_on = 0;
    //Fill up the rest of our ptah planner after filling it with previous points
    double t = 0;
    for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
        double N = (target_dist / (0.02 * v_final));
        double x_point = x_add_on + target_x / N;     
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        //rotate back to normal after rotating it earlier
        x_point = (x_ref * cos (ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin (ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        if (x_point > 6945.554) x_point -= 6945.554;
        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}

