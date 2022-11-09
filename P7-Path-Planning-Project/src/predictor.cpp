#include <cmath>
#include <cstdio>
#include <limits>
#include "predictor.h"

using namespace std;

/**
 * @brief predict the future state of traffic.
 * 
 * @param sensor_fusion 
 * @param car_lane 
 * @param prev_path_size 
 * 
 * sensor_fusion returns the data of all "currently spawned vehicle" on the road
 */
void Predictor::predict(const vector<vector<int>>& sensor_fusion, const int prev_path_size)
{
    //printf("D: ");
    // iterate through sensor_fusion and predict it's location and lanes.
    for (int i = 0; i<sensor_fusion.size(); i++) {
        // sensor data format: [id, x, y, vx, vy, s, d]
        double opponent_vx = sensor_fusion[i][3];
        double opponent_vy = sensor_fusion[i][4];
        double opponent_s = sensor_fusion[i][5];
        double opponent_d = sensor_fusion[i][6];

        // identify lane
        int opponent_lane = identify_lane(opponent_d);

        //printf("[%5.1f, %d]", opponent_d, opponent_lane);

        if (opponent_lane == -1) {
            // skip if the opponent car does not match to any lane.
            continue;
        } else {
            // identify opponent speed in s-direction
            double opponent_vs = sqrt(pow(opponent_vx, 2) + pow(opponent_vy, 2));
            // opponent's position after previous trajectory

            opponents_lane.push_back(opponent_lane);
            opponents_s.push_back(opponent_s);
            opponents_vs.push_back(opponent_vs);
        }
    }

    // reserve vector memory
    available_lanes = {true, true, true};

    // iterate through opponents_lane and search available lanes.
    for (int i = 0; i<opponents_lane.size(); i++) {

        //path range is [car_s, car_s_predict]
        //check interference while executing the path.
        for (int j = 0; j <= prev_path_size; j++) {
            double car_s_predict = car_s + car_vs * TIME_STEP * j;
            double opponent_s_predict = opponents_s[i] + opponents_vs[i] * TIME_STEP * j;

            double distance_predict = opponent_s_predict - car_s_predict;

            // determine availability by prediction position
            if (car_lane == opponents_lane[i]) {                                            // if two cars are in the same lane
                if (distance_predict <= DISTANCE_FRONT_GAP && distance_predict >= 0) {                  // if within range and ahead
                    available_lanes[opponents_lane[i]] = false;                             // mark my lane as unavailable
                }
            } else {                                                                                // if the traffic is in another lane
                if (distance_predict <= DISTANCE_FRONT_GAP && distance_predict >= -DISTANCE_BEHIND_GAP) {    // if within range
                    available_lanes[opponents_lane[i]] = false;                                     // mark that lane as unavailable
                }
            }
        }
    }


    // find valid speed for not available lanes (for slow down)
    double inf = numeric_limits<double>::infinity();
    closest_car_s = {inf, inf, inf};
    available_speed = {inf, inf, inf};

    for (int i = 0; i < opponents_lane.size(); i++) {
        int lane = opponents_lane[i];
        if (opponents_s[i] > car_s-1 && opponents_s[i] < closest_car_s[lane]) {
            closest_car_s[lane] = opponents_s[i];
            available_speed[lane] = opponents_vs[i];
        }
    }

}

vector<double> Predictor::getAvailableSpeed() {
    return available_speed;
}

vector<double> Predictor::getClosestCarS() {
    return closest_car_s;
}

vector<bool> Predictor::getAvailableLanes() {
    return available_lanes;
}

void Predictor::log(int verbose=2) {
    if(verbose >= 2) {
        printf("Predict: ");
        for (int i = 0; i<opponents_lane.size(); i++) {
            printf("[L: %d, s: %4.0f] ", opponents_lane[i], opponents_s[i]);
        }
        printf("\n");
    }
    if (verbose >= 1) {
        printf("Lane Status: ");
        for (int i = 0; i<available_lanes.size(); i++) {
            printf("[L%d: %d] ", i, (int)available_lanes[i]);
        }
        printf("\n");
    }
    


}

