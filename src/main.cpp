/**
 ** A simple path planner in C++ for a simulated self-driving car.
 **
 ** Scott Penberthy
 ** aai@google.com
 ** August, 2017
 **
 **/

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#define MAX_SPEED 49.5
#define MAX_ACCEL 0.4
#define NUM_POINTS 30
#define LANE_CHANGE 150

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
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

bool overlaps(vector<double> i1, vector<double> i2, double tolerance)
{
  // left start, proximity to right
  double closest = fabs(i1[0]-i2[0]);
  closest = min(closest, fabs(i1[0]-i2[1]));

  // left end, proximity to right
  closest = min(closest, fabs(i1[1]-i2[0]));
  closest = min(closest, fabs(i1[1]-i2[1]));

  return (closest <= tolerance);
}


double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
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

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // our lane
  double lane=1;

  // our speed
  double ref_vel = 0;

  // crowds
  bool was_crowded = false;
  bool going_slow = false;
  
  h.onMessage([&was_crowded, &going_slow, &lane,
               &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
                int prev_size = previous_path_x.size();

          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

                // create points in the local coordinate system of the car
                // for path planning and spline fitting
                vector<double> ptsx;
                vector<double> ptsy;

                // These are the "reference" values for the most recent
                // location and orientation of the car.  We either use the
                // start of the system at launch, or the end of the most
                // recent path we sent to the simulator.
                double ref_x = car_x;
                double ref_y = car_y;
                double ref_yaw = deg2rad(car_yaw);

                // if previous path is empty, use the car as a starting reference
                if (prev_size < 2) {
                  double prev_car_x = car_x - cos(car_yaw);
                  double prev_car_y = car_y - sin(car_yaw);

                  ptsx.push_back(prev_car_x);
                  ptsy.push_back(prev_car_y);
                  ptsx.push_back(car_x);
                  ptsy.push_back(car_y);
                }
                else {
                  // redefine reference state as end of last path
                  ref_x = previous_path_x[prev_size-1];
                  ref_y = previous_path_y[prev_size-1];
                  
                  double ref_x_prev = previous_path_x[prev_size-2];
                  double ref_y_prev = previous_path_y[prev_size-2];
                  ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

                  ptsx.push_back(ref_x_prev);
                  ptsy.push_back(ref_y_prev);
                  ptsx.push_back(ref_x);
                  ptsy.push_back(ref_y);
                }

                // We drive using a fixed heuristic in lieu of creating multiple
                // trajectories and costs.  Our car attempts to drive the speed
                // limit by default.
                //
                // 1) If we see a car in our lane, we reduce
                // our speed and follow behind to match its speed, which we indicate
                // by setting the "going_slow" flag. 
                // choose lane & velocity
                //
                // 2) If we see cars in our lane, look for an 30m opening in front
                // and behind us to pass on the left, or to pass on the right.  Pass
                // if possible.
                //
                // 3) If we're traveling in the left or right lane, and the coast
                // is clear, switch back to the middle lane for better maneuverability.
                //
                bool too_close = false;

                // We scan the sensor fusion data and create three vectors,
                // one each for the lane on the left and right of us.  These
                // vectors contain the identifiers of cars +/- 30m from us.
                // An empty vector denotes an "open" lane.
                vector<double> c_left;
                vector<double> c_right;
                vector<double> c_lane;

                double v_left = ref_vel;
                double v_right = ref_vel;
                double v_lane = ref_vel;
                double target_lane = (double) lane;

                // data format for each car is [id, x, y, vx, vy, s, d]
                bool crowded = false;
                bool changes = false;
                
                // deduce our reference coordinates in frenet space, for path planning
                vector<double> ref_sd = getFrenet(ref_x, ref_y, car_yaw, map_waypoints_x, map_waypoints_y);
                double my_v = ref_vel/2.24;

                // iterate through our sensor fusion data to create our
                // vectors of close vehicles, throwing the "crowded" switch
                // if any lane has something close.
                for (int i=0; i < sensor_fusion.size(); i++) {
                  vector<double> car = sensor_fusion[i];
                  double id_other = car[0];
                  double s_other = car[5];
                  double d_other = car[6];
                  double v_other = sqrt(car[3]*car[3]+car[4]*car[4]);
                  vector<double> my_s = {car_s, car_s+NUM_POINTS*0.02*car_speed/2.24};
                  vector<double> other_s = {s_other, s_other+NUM_POINTS*0.02*v_other};
                  bool in_lane = ((d_other > (2+4*lane-2) && (d_other < (2+4*lane+2))));
                  bool on_left = ((d_other > (2+4*(lane-1)-2)) && (d_other < (2+4*(lane-1)+2)));
                  bool on_right = ((d_other > (2+4*(lane+1)-2)) && (d_other < (2+4*(lane+1)+2)));
                  bool is_close = overlaps(my_s, other_s, 30.0);
                  bool ahead = (s_other > car_s);
                  
                  if (in_lane && ahead && is_close) {
                    crowded = true;
                    c_lane.push_back(i);
                    v_lane = min(v_lane,v_other);
                  }
                  else if (on_left && is_close) {
                    crowded = true;
                    c_left.push_back(i);
                    v_left = min(v_left,v_other);
                  }
                  else if (on_right && is_close) {
                    crowded = true;
                    c_right.push_back(i);
                    v_right = min(v_right,v_other);
                  }
                }

                // choose lane
                if (crowded) {
                  if (c_lane.size() > 0) {  // something is in front of us

                    // pass on left if free
                    if (c_left.size() < 1 && lane > 0) {
                      cout << "Passing on left" << endl;
                      target_lane = lane-1;
                      going_slow = false;
                      changes = true;
                    }
                    // pass on right if free
                    else if (c_right.size() < 1 && lane < 2) {
                      cout << "Passing on right" << endl;
                      target_lane = lane+1;
                      going_slow = false;
                      changes = true;
                    }
                  }
                }
                // move to center lane for better maneuverability
                else if (lane == 0 && c_right.size() < 1) {
                  cout << "Moving to center lane" << endl;
                  changes = true;
                  going_slow = false;
                  target_lane = 1;
                }
                else if (lane == 2 && c_left.size() < 1) {
                  cout << "Moving to center lane" << endl;
                  changes = true;
                  going_slow = false;
                  target_lane = 1;
                }


                // Now that we've chosen a lane, choose how to adjust
                // our speed.  We slow down and match the car in front
                // if its close.  Otherwise, we gradually accelerate to
                // approach the speed limit.

                // choose speed
                if (crowded) {
                  if (c_lane.size() > 0 && (my_v > v_lane)) {
                    // drop our speed until we can switch lanes
                    going_slow = true;
                    //cout << "Braking " << v_lane << " vs " << ref_vel << endl; // be smarter here
                    //cout << "...dv " << dv << endl;
                    double dv = max((v_lane - my_v)*2.24, -MAX_ACCEL);
                    ref_vel += dv;
                    changes = true;
                  }
                  else {
                    going_slow = false;
                    ref_vel += MAX_ACCEL;
                    changes = true;
                  }
                }
                else if (ref_vel < MAX_SPEED && !going_slow) {
                  ref_vel += MAX_ACCEL;
                  changes = true;
                  //cout << "Accelerating " << ref_vel << endl;
                }
                ref_vel = fmin(MAX_SPEED, ref_vel);

                if (crowded) {
                  if (!was_crowded) {
                    cout << "Cars detected " << endl;
                    was_crowded = true;
                  }
                  /**
                  // print out a diagnostic
                  cout << "[";
                  for (const auto i: c_left) cout << i << " ";
                  cout << "] ";
                  
                  cout << "[";
                  for (const auto i: c_lane) cout << i << " ";
                  cout << "] ";
                  
                  cout << "[";
                  for (const auto i: c_right) cout << i << " ";
                  cout << "] ";
                  cout << endl;
                  **/
                }
                else if (was_crowded) {
                  cout << "...free!" << endl;
                  was_crowded = false;
                }

                // create control points for the s spline, easing into the next lane
                // over 240 meters.
                for (int i=30; i <= LANE_CHANGE; i += 15) {  // change lanes in 3 seconds 3x50x0.02
                  double d_lane = lane+(target_lane-lane)*(i-30.0)/LANE_CHANGE;
                  vector<double> wp = getXY(ref_sd[0]+i,(2+4*d_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
                  ptsx.push_back(wp[0]);
                  ptsy.push_back(wp[1]);
                }

                // switch lanes if need be, now that we baked a spline into the path
                lane = target_lane;

                // shift to local coordinates
                for (int i=0; i < ptsx.size(); i++) {
                  
                  // shift our ref angle to 0 degrees
                  double shift_x = ptsx[i]-ref_x;
                  double shift_y = ptsy[i]-ref_y;

                  ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
                  ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
                }

                // create a spline for our lane transition / path ahead
                tk::spline s;
                s.set_points(ptsx, ptsy);
               
                // first path
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

                for (int i=0; i < prev_size; i++) {
                  next_x_vals.push_back(previous_path_x[i]);
                  next_y_vals.push_back(previous_path_y[i]);
                }
        
                // split up the spline
                double target_x = 90.0;
                double target_y = s(target_x);
                double target_dist = sqrt(target_x*target_x + target_y*target_y);
                double N = target_dist/(0.02*ref_vel/2.24); // convert mph to m/s by 1/2.24
                double x_add_on = 0;

                // add new spline points at the end
                //cout << "Adding " << (NUM_POINTS-prev_size) << " points " << " dx=" << round((target_x/N)*100) << endl;
                for (int i=0; i < NUM_POINTS-prev_size; i++) {
                  double x_point = x_add_on + target_x/N;
                  double y_point = s(x_point);

                  x_add_on = x_point;
                  double x_ref = x_point;
                  double y_ref = y_point;

                  // rotate back to normal after earlier
                  x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
                  y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

                  x_point += ref_x;
                  y_point += ref_y;

                  next_x_vals.push_back(x_point);
                  next_y_vals.push_back(y_point);
                }

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































