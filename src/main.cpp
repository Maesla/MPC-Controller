#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

Eigen::VectorXd derivePoly(Eigen::VectorXd coeffs)
{
  int derivativeGrade = coeffs.size() - 1;
  Eigen::VectorXd derivativePoly(derivativeGrade);

  for(int grade = 0; grade < derivativeGrade; grade++)
  {
    int originalGrade = grade + 1;
    derivativePoly[grade] = coeffs[originalGrade]*originalGrade;
  }
  return derivativePoly;
}


void transformWorldCoordinates2VehicleCoordinates
(float x_world_vehicle, float y_world_vehicle, float psi_world_vehicle, vector<double> xvals,vector<double> yvals, vector<double> &xlocals, vector<double> &ylocals)
{
  int arraySize = xvals.size();
  for(int i = 0; i < arraySize; i++)
  {
    double x = xvals[i] - x_world_vehicle;
    double y = yvals[i] - y_world_vehicle;
    double x_local = x * cos(-psi_world_vehicle) - y * sin(-psi_world_vehicle);
    double y_local = x * sin(-psi_world_vehicle) + y * cos(-psi_world_vehicle);

    xlocals.push_back(x_local);
    ylocals.push_back(y_local);
  }
}

void transformWorldCoordinates2VehicleCoordinates
(float x_world_vehicle, float y_world_vehicle, float psi_world_vehicle, double x_world, double y_world, double &x_local, double &y_local)
{
  double x = x_world - x_world_vehicle;
  double y = y_world - y_world_vehicle;
  x_local = x * cos(-psi_world_vehicle) - y * sin(-psi_world_vehicle);
  y_local = x * sin(-psi_world_vehicle) + y * cos(-psi_world_vehicle);
}

double transformSteering2SteeringInput(double steering)
{
  // Input = 1 => steer = 25 degrees OR 0.436332 rads
  // X steer (rad) · 1(input)/ 0.436332 (rad) => X · (1/0.436332)(input) => X · 2.291832 (input)
  double conversionFactor = 2.291832;
  return -steering/0.436332;
  //return -steering/2.67;
  //return steering * conversionFactor;
}

double CalculateCte(Eigen::VectorXd coeffs, double x, double y)
{
  return polyeval(coeffs, x) - y;
}

double CalculateEpsi(Eigen::VectorXd derivative_coeffs, double x, double psi)
{
  return psi - atan(polyeval(derivative_coeffs, x));
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // TODO all mixed in world and local
          vector<double> waypoints_x_local;
          vector<double> waypoints_y_local;

          transformWorldCoordinates2VehicleCoordinates(px, py, psi, ptsx, ptsy, waypoints_x_local, waypoints_y_local);
          Eigen::VectorXd waypoints_x_local_eigen = Eigen::VectorXd::Map(waypoints_x_local.data(), waypoints_x_local.size());
          Eigen::VectorXd waypoints_y_local_eigen = Eigen::VectorXd::Map(waypoints_y_local.data(), waypoints_y_local.size());

          auto coeffs = polyfit(waypoints_x_local_eigen, waypoints_y_local_eigen, 3);
          auto derivative_coeffs = derivePoly(coeffs);

          Eigen::VectorXd state(6);
          double x = 0;
          double y = 0;
          psi = 0;
          double cte = CalculateCte(coeffs, x, y);
          double epsi = CalculateEpsi(derivative_coeffs, x, psi);

          state << x, y, psi, v, cte, epsi;
          auto result = mpc.Solve(state, coeffs);

          //TODO transfrom from angle to -1 1
          //double steer_value = vars[0];
          double steer_value = result.steer[0];

          steer_value = transformSteering2SteeringInput(steer_value);
          //double throttle_value = vars[1];
          double throttle_value = result.acceleration[0];

          std::cout << "steer = " << steer_value << " throttle = " << throttle_value << std::endl;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;


          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          for(int i = 0; i < result.x.size(); i++)
          {
            double world_x = result.x[i];
            double world_y = result.y[i];

            mpc_x_vals.push_back(world_x);
            mpc_y_vals.push_back(world_y);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for(int i = 0; i < waypoints_x_local.size(); i++)
          {
            double poly_x = waypoints_x_local[i];
            double poly_y = polyeval(coeffs, poly_x);

            next_x_vals.push_back(poly_x);
            next_y_vals.push_back(poly_y);
          }
          //next_x_vals = waypoints_x_local;
          //next_y_vals = waypoints_y_local;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
