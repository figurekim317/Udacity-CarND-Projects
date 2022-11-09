#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}



void straight(double car_x, double car_y, double car_yaw, vector<double>& next_x_vals, vector<double>& next_y_vals) {
    double dist_inc = 0.5;
    for (int i = 0; i < 50; ++i) {
        next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
        next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
    }
}

void straight_along(double car_s, double car_d, vector<double>& next_x_vals, vector<double>& next_y_vals, std::function<vector<double>(double, double)> getXYfromMap) {
    double dist_inc = 0.5;
    for (int i = 0; i < 50; ++i) {
        double next_s = car_s + dist_inc*i;
        vector<double> next_xy = getXYfromMap(next_s, car_d);
        next_x_vals.push_back(next_xy[0]);
        next_y_vals.push_back(next_xy[1]);
    }
}

void circle(double car_x, double car_y, double car_yaw, vector<double>& next_x_vals, vector<double>& next_y_vals, vector<double>& previous_path_x, vector<double>& previous_path_y) {
    double pos_x;
    double pos_y;
    double angle;
    int path_size = previous_path_x.size(); //length of remaining path

    for (int i = 0; i < path_size; ++i) //attach previous path in front.
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    if (path_size == 0) //if path length is zero, then start from current position
    {
        pos_x = car_x;
        pos_y = car_y;
        angle = deg2rad(car_yaw);
    }
    else //else interpolate the last two path point.
    {
        pos_x = previous_path_x[path_size - 1];
        pos_y = previous_path_y[path_size - 1];

        double pos_x2 = previous_path_x[path_size - 2];
        double pos_y2 = previous_path_y[path_size - 2];
        angle = atan2(pos_y - pos_y2, pos_x - pos_x2); //x2 -> x1
    }

    double dist_inc = 0.5;
    for (int i = 0; i < 50 - path_size; ++i) //path length is 50
    {
        next_x_vals.push_back(pos_x + (dist_inc)*cos(angle + (i + 1) * (pi() / 100))); //make circle
        next_y_vals.push_back(pos_y + (dist_inc)*sin(angle + (i + 1) * (pi() / 100))); //but I don't know what this really means.
        pos_x += (dist_inc)*cos(angle + (i + 1) * (pi() / 100));
        pos_y += (dist_inc)*sin(angle + (i + 1) * (pi() / 100));
    }
}






#endif  // HELPERS_H