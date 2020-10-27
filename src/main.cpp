#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// constant variables
const double MAX_ACC = 0.224;
const double MAX_SPEED = 49.5;
const int LEFT_LANE = 0;
const int MIDDLE_LANE = 1;
const int RIGHT_LANE = 2;
const int NO_LANE = -1;


int main() {
  uWS::Hub h;

  /* Load up map values for waypoint's x,y,s and d normalized normal vectors
   way points are the points on the map. we can find the closest waypoint
   closest waypoint could be behind the car but next waypoint could be where to go next */
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  /* Waypoint map to read from. Has 180 way points along the center of the track */
  string map_file_ = "../data/highway_map.csv";
  /* The max s value before wrapping around the track back to 0 -
  max length of the track */
  double max_s = 6945.554;

  /* Read the map file */
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  /* Get the x, y, s, d_x, d_y values of all the waypoints and
  add it to the vector */
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    //normal component of way point
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }


  /* Start in lane 1 which is the middle lane; left lane is 0 */
  int ego_lane = MIDDLE_LANE;

  /* Set some reference velocity in MPH - taken as 0, maximum being 49.5 */
  double ref_vel = 0.0f;

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ego_lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
            // j[1] is the data JSON object

            /* Main car's localization Data - these values come from the simulator */
            double car_x = j[1]["x"];
            double car_y = j[1]["y"];
            double car_s = j[1]["s"];
            double car_d = j[1]["d"];
            double car_yaw = j[1]["yaw"];
            double car_speed = j[1]["speed"];

            /* Previous path data given to the Planner */
            auto previous_path_x = j[1]["previous_path_x"];
            auto previous_path_y = j[1]["previous_path_y"];

            /* Previous path's end s and d values */
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];

            /* Sensor Fusion Data, a list of all other cars on the same side
            of the road */
            auto sensor_fusion = j[1]["sensor_fusion"];

            // Start of To Do

            /* Get the size of the previous path vector */
            int prev_size = previous_path_x.size();

            /* car_s will be previous end point if previous points exist */
            if (prev_size > 0) {
                car_s = end_path_s;
            }

            /* Prediction - Use the sensor fusion information to analyse other
            cars positions and velocities

            /* Variable to define car lane */
            int targer_car_lane;
            /* Variable to define change in speed */
            double ego_speed_change;
            /* Variable to define direction of lanes */
            bool target_car_ahead = false;
            bool target_car_left = false;
            bool target_car_right = false;

            for (int i = 0; i < sensor_fusion.size(); i++) {
                /* d value gives what lane other cars are in */
                float d_target = sensor_fusion[i][6];

                /* Check in which lane the cars are present */
                if (d_target > 0 && d_target < 4) {
                    targer_car_lane = LEFT_LANE;
                }
                else if (d_target > 4 && d_target < 8) {
                    targer_car_lane = MIDDLE_LANE;
                }
                else if (d_target > 8 && d_target < 12) {
                    targer_car_lane = RIGHT_LANE;
                }
                else {
                    targer_car_lane = NO_LANE;
                }

                /* Get velocities of the target cars */
                double vx_target = sensor_fusion[i][3];
                double vy_target = sensor_fusion[i][4];
                /* Speed is important to predict */
                double check_target_speed = std::sqrt(vx_target * vx_target + vy_target * vy_target);
                /* checks the s value of the other cars to check if they
                are nearby */
                double check_target_s = sensor_fusion[i][5];

                /* Behavior Planning - Gives direction to the ego vehicle based on the inputs of prediction */

                /* Estimate car s position after executing previous trajectory */
                check_target_s += ((double)prev_size * .02 * check_target_speed);

                /* If the car is in the same lane and the the gap between the other car
                and our car is less than 30 meters, set the flag */
                if ((targer_car_lane == ego_lane) && (check_target_s > car_s) && ((check_target_s - car_s) < 30)) {
                    target_car_ahead = true;
                }
                /* If the car is in the left side and the the gap between the other car
                and our car is less than 30 meters, set the flag */
                else if ((targer_car_lane == (ego_lane - 1)) && (car_s - 30 < check_target_s && check_target_s < car_s + 30)) {
                    target_car_left = true;
                /* If the car is in the right side and the the gap between the other car
                and our car is less than 30 meters, set the flag */
                }
                else if ((targer_car_lane == (ego_lane + 1)) && (car_s - 30 < check_target_s && check_target_s < car_s + 30)) {
                    target_car_right = true;
                }
            }

            /* Change the ego vehicle's lane and velocity depending on the state machine */
            if (target_car_ahead) {
                ref_vel -= MAX_ACC;

                /* And if there is no car in the left side of the lane */
                if (ego_lane > LEFT_LANE && !target_car_left) {
                    ego_lane--;
                }
                /* And if there is no car in the right side of the lane */
                else if (ego_lane < RIGHT_LANE && !target_car_right) {
                    ego_lane++;
                }
            }
            else {
                if (ref_vel < MAX_SPEED) {
                    ref_vel += MAX_ACC;
                }

                if (ego_lane != MIDDLE_LANE) {
                    if ((ego_lane == LEFT_LANE && !target_car_right) || (ego_lane == RIGHT_LANE && !target_car_left)) {
                        ego_lane = MIDDLE_LANE;
                    }
                }
            }

          /* Trajectory Generation - Based on the directions given by behavior planning, trajectory is
          generated for the car to move along */

          /* Create a widely spaced vector points spaced at 30m each, later we will
          interpolate these points with spline and fill in more points */
          std::vector<double> points_x{};
          std::vector<double> points_y{};

          /* Create reference variable for x , y and yaw. they could be either the
          starting point of the car or the end point of the previous path */
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          /* Check whether the previous car state is nearly empty or has some points */
          if (prev_size < 2) {
              double previous_car_x = car_x - cos(car_yaw);
              double previous_car_y = car_y - sin(car_yaw);

              /* Use two points that make path tangent to the car */
              points_x.push_back(previous_car_x);
              points_x.push_back(car_x);

              points_y.push_back(previous_car_y);
              points_y.push_back(car_y);
              /* Use the previous path's end point as the reference */
          }
          else {
              /* Redefine reference state as the end point of the previous path */
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              /* Calculate the angle based on the previous points */
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              points_x.push_back(ref_x_prev);
              points_x.push_back(ref_x);

              points_y.push_back(ref_y_prev);
              points_y.push_back(ref_y);
          }

          std::vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * ego_lane), map_waypoints_s,
              map_waypoints_x, map_waypoints_y);
          std::vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * ego_lane), map_waypoints_s,
              map_waypoints_x, map_waypoints_y);
          std::vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * ego_lane), map_waypoints_s,
              map_waypoints_x, map_waypoints_y);

          points_x.push_back(next_wp0[0]);
          points_x.push_back(next_wp1[0]);
          points_x.push_back(next_wp2[0]);

          points_y.push_back(next_wp0[1]);
          points_y.push_back(next_wp1[1]);
          points_y.push_back(next_wp2[1]);

          for (int i = 0; i < points_x.size(); i++) {
              /*Shift car reference angle to 0 degrees */
              double shift_x = points_x[i] - ref_x;
              double shift_y = points_y[i] - ref_y;

              /* Transform to local cordinates */
              points_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
              points_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          /* Create a spline */
          tk::spline s;

          /* Set some points in the spline */
          s.set_points(points_x, points_y);

          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

          /* Start adding the previous points to the path planner */
          /* Instead of recreating points from the scratch, add points
          left from the previous path */
          for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          /* calculate how to break up the 30m spaced spline points */

          /* Horizontal x axis */
          double target_x = 30;
          /* The vertical y value of corresponding x value */
          double target_y = s(target_x);
          /* Value of the hypoteneuse */
          double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
          double x_add_on = 0;

          for (int i = 1; i <= (50 - previous_path_x.size()); i++) {
              double target_dist_points = (0.02 * ref_vel / 2.24);
              double N = (target_dist / target_dist_points);
              double dist_target_x_points = (target_x) / N;
              double x_point = x_add_on + dist_target_x_points;
              double y_point = s(x_point);

              /* Keep adding x points to calculate the x point in the spline */
              x_add_on = x_point;

              /* Transfer to a variable to convert it back to global co-ordinates */
              double x_temp = x_point;
              double y_temp = y_point;

              /* Transform back to global co-ordinates */
              x_point = (x_temp * cos(ref_yaw) - y_temp * sin(ref_yaw));
              y_point = (x_temp * sin(ref_yaw) + y_temp * cos(ref_yaw));

              /* Add the calculated x-points to the initial x, y points of the car which we got from localization */
              x_point += ref_x;
              y_point += ref_y;

              /* Push these final calculated x,y points to the path planner */
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
