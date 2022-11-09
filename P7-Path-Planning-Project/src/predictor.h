#ifndef PREDICTOR_H
#define PREDICTOR_H

#include <vector>

using std::vector;

int identify_lane(const double);

class Predictor {
public:
    Predictor(double car_s, double car_d, double car_vs):
                car_s(car_s), car_d(car_d), car_vs(car_vs), car_lane(identify_lane(car_d)) { }

    void predict(const vector<vector<int>>&, const int);
    vector<double> getAvailableSpeed();
    vector<double> getClosestCarS();
    vector<bool> getAvailableLanes();

    int identify_lane(const double car_d) {
        int car_lane = -1;
        if (0 < car_d && car_d <= 4) {
            car_lane = 0;
        } else if (4 < car_d && car_d <= 8) {
            car_lane = 1;
        } else if (8 < car_d && car_d <=12) {
            car_lane = 2;
        }
        return car_lane;
    }

    void log(int);

private:
    const double car_s;
    const double car_d;
    const double car_vs;
    const double car_lane;

    const double TIME_STEP = 0.02; //0.02 millis
    const double DISTANCE_FRONT_GAP = 20;
    const double DISTANCE_BEHIND_GAP = 15;


    vector<int> opponents_lane;
    vector<double> opponents_s;
    vector<double> opponents_vs;
    
    vector<bool> available_lanes;
    vector<double> available_speed;
    vector<double> closest_car_s;

    
    
};

#endif