#include <cstdio>
#include <algorithm>
#include <iostream>
#include <functional>

#include "spline.h"
#include "behavior_planner.h"

void BehaviorPlanner::setDestination(double end_path_s, double end_path_d, vector<double> available_speed, vector<double> closest_car_s, vector<bool> available_lanes) {
    
    // keep current line if all three lines are unavailable 
    int path_end_lane = identify_lane(end_path_d);
    final_lane = start_lane;

    // find left or right lane to change.
    for (int i=0; i<=1; i++) {
        int left_lane = start_lane-i;
        int right_lane = start_lane+i;
        // priority on left lane for overtaking
        if(left_lane>=0 && left_lane<=2) {
            if(available_lanes[left_lane]) {
                final_lane = left_lane;
                break;
            }
        }
        if(right_lane>=0 && right_lane<=2) {
            if(available_lanes[right_lane]) {
                final_lane = right_lane;
                break;
            }
        }
    }

    // to prevent getting stuck to traffic, center lane have priority when available.
    if (available_lanes[1]) {
        final_lane = 1; //overiding
    }

    // if car in front is too close
    double front_distance = closest_car_s[start_lane] - end_path_s;
    if (front_distance < SAFETY_GAP ) {
        printf("slow down!\n");
        final_speed = available_speed[start_lane];
    } else {
        // if ego car is slower than setting speed
        final_speed = MAX_SPEED;
    }
}

void BehaviorPlanner::setPath(vector<double>& next_x_vals, vector<double>& next_y_vals, vector<double>& previous_path_x, vector<double>& previous_path_y, double end_path_s,
                                std::function<vector<double>(double, double)> getXYfromMap, std::function<vector<double>(double, double, double)> getFrenetfromMap) {
    // There are two cases for path planning.
    // 1. If previous path does not exist(fresh start),
    // whole path will be generated based on current car position and prediction.
    // 2. Or if previous path exist, the previous path will be reused and 
    // new path will be generated/concatenated based on the *state at the last point* of previous path. 
    
    
    // First, define the initial position of new path.
    double prev_path_size = previous_path_x.size();
    double new_path_start_s;
    // for fresh start, generate path from current car position
    if (prev_path_size == 0) {  
        new_path_start_s = car_s;
    // if previous path exists, generate path at the end of the path.
    } else {                    
        new_path_start_s = end_path_s;
    }


    // now let's derive waypoints in map cartesian.
    vector<double> global_waypoints_x;
    vector<double> global_waypoints_y;


    // Second, derive boundary condition for spline end from previous path end.
    // (in global cartesian coordinate at path end)
    double global_path_end_x;
    double global_path_end_y;
    double global_path_end_yaw;
    // use car_yaw for boundary condition if path length is less than 2.
    if (prev_path_size < 2) {
        double prev_global_x = car_x-cos(car_yaw);
        double prev_global_y = car_y-sin(car_yaw);

        // This will be the boundary condition for spline
        global_waypoints_x.push_back(prev_global_x);
        global_waypoints_y.push_back(prev_global_y);
        global_waypoints_x.push_back(car_x);
        global_waypoints_y.push_back(car_y);

        // state at path end
        global_path_end_x = car_x;
        global_path_end_y = car_y;
        global_path_end_yaw = car_yaw;
    } else {
    // use path_end_yaw for boundary condition
        double prev_global_x = previous_path_x[prev_path_size-2];
        double prev_global_y = previous_path_y[prev_path_size-2];
        double global_x = previous_path_x[prev_path_size-1];
        double global_y = previous_path_y[prev_path_size-1];

        // This will be the boundary condition for spline
        global_waypoints_x.push_back(prev_global_x);
        global_waypoints_y.push_back(prev_global_y);
        global_waypoints_x.push_back(global_x);
        global_waypoints_y.push_back(global_y);

        // states at path end
        global_path_end_x = global_x;
        global_path_end_y = global_y;
        global_path_end_yaw = atan2(global_y-prev_global_y, global_x-prev_global_x);
    }

    // Third, desiging the frenet waypoints to visit
    vector<double> waypoints_s;
    vector<double> waypoints_d;
    waypoints_s.push_back(new_path_start_s+30); // destination
    waypoints_d.push_back(2+final_lane*4);
    waypoints_s.push_back(new_path_start_s+60); // for smoothing
    waypoints_d.push_back(2+final_lane*4);
    waypoints_s.push_back(new_path_start_s+90); // for smoothing
    waypoints_d.push_back(2+final_lane*4);
    // Complete frenet waypoints and convert to global cartesian.
    for(int i=0; i<waypoints_s.size(); i++) {
        auto waypoint_xy = getXYfromMap(waypoints_s[i], waypoints_d[i]);
        double waypoint_x = waypoint_xy[0];
        double waypoint_y = waypoint_xy[1];

        global_waypoints_x.push_back(waypoint_x);
        global_waypoints_y.push_back(waypoint_y);
    }

    // Fourth, convert global cartesian to local cartesian waypoints at *path end*
    vector<double> local_waypoints_x;
    vector<double> local_waypoints_y;
    
    for(int i=0; i<global_waypoints_x.size(); i++) {
        double waypoint_x = global_waypoints_x[i];
        double waypoint_y = global_waypoints_y[i];

        //linear transformation to local cartesian.
        double local_x = (waypoint_x-global_path_end_x)*cos(0-global_path_end_yaw)-(waypoint_y-global_path_end_y)*sin(0-global_path_end_yaw); 
        double local_y = (waypoint_x-global_path_end_x)*sin(0-global_path_end_yaw)+(waypoint_y-global_path_end_y)*cos(0-global_path_end_yaw);

        local_waypoints_x.push_back(local_x);
        local_waypoints_y.push_back(local_y);
    }



    // Now, let's generate path!
    // Fit the spline at local coordinate at path end
    tk::spline s;
    s.set_points(local_waypoints_x, local_waypoints_y);

    // copy previous path first
    for(int i=0; i<prev_path_size; i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Then convert to global coordinate
    // Remember that path end is the origin of local x, y
    double local_x = 0; 
    // generating and concatenating on previous path.
    for(int i=prev_path_size; i<50; i++) {
        if (current_speed > final_speed) {     // when deceleration is required
            current_speed -= MAX_ACC;
        } else {                            // when acceleration is required
            current_speed += MAX_ACC;          // speed up
            if(current_speed > MAX_SPEED) {    // within limit
                current_speed = MAX_SPEED;
            }
        }

        // move the car in local cartesian, then convert to global cartesian.
        double next_local_x = local_x + TIME_STEP*current_speed;
        double next_local_y = s(next_local_x);
        double next_global_x = global_path_end_x + next_local_x*cos(global_path_end_yaw) - next_local_y*sin(global_path_end_yaw);
        double next_global_y = global_path_end_y + next_local_x*sin(global_path_end_yaw) + next_local_y*cos(global_path_end_yaw);

        next_x_vals.push_back(next_global_x);
        next_y_vals.push_back(next_global_y);

        local_x = next_local_x;
    }
}

void BehaviorPlanner::log() {
    printf("Current: %d, Final %d\n", start_lane, final_lane);
}
