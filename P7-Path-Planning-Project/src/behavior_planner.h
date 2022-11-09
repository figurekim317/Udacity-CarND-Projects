#include <vector>
#include <cmath>

using std::vector;

class BehaviorPlanner {
public:
    BehaviorPlanner(double car_x, double car_y, double car_yaw, double car_s, double car_d, double car_vs):
                car_x(car_x), car_y(car_y), car_yaw(deg2rad(car_yaw)), 
                car_s(car_s), car_d(car_d), car_vs(car_vs), start_lane(identify_lane(car_d)){ }
    void setDestination(double, double, vector<double>, vector<double>, vector<bool>);
    void setPath(vector<double>&, vector<double>&, vector<double>&, vector<double>&, double,
                std::function<vector<double>(double, double)>, std::function<vector<double>(double, double, double)>);
    double interpolateSpeed(double, double);
    void log();

    double deg2rad(double degree) {
        return degree*M_PI/180.0;
    }

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

    static double current_speed;

private:
    double interpolateDistance(double, double);
    const double car_x;
    const double car_y;
    const double car_s;
    const double car_d;
    const double car_yaw;
    const double car_vs;
    const int start_lane;

    int final_lane;
    double final_speed;

    const double SAFETY_GAP = 10; //15m
    const double MAX_SPEED = 22; //22m/s == 48mph
    const double MAX_ACC = 0.05;  //5m/s^2
    const double TIME_STEP = 0.02; //0.02 millis
};

