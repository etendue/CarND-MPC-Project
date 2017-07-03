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


void test_lake_track();

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  const double degree25_radian = 0.436332;

  //test_lake_track();
  //return 0;
  h.onMessage([&mpc,&degree25_radian](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          double steering_angle = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          //transform way points to car coordination
          vector<double> ptsx_car;
          vector<double> ptsy_car;

          for(size_t i=0; i< ptsx.size();i++){
        	  double x_car = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py)*sin(psi);
        	  double y_car = (ptsy[i] - py)* cos(psi) - (ptsx[i] - px) * sin(psi);
        	  ptsx_car.push_back(x_car);
        	  ptsy_car.push_back(y_car);
          }

          //get the waypoints
          Eigen::Map<Eigen::VectorXd> xvals (ptsx_car.data(),ptsx_car.size());
          Eigen::Map<Eigen::VectorXd> yvals (ptsy_car.data(),ptsy_car.size());
          auto coeffs = polyfit(xvals,yvals,3);
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          //predict cars position in 100 miliseconds
          v = v * 0.44704; //convert mph to m/s
          double dt = 0.1;
          double px_in_100ms = v * cos(-steering_angle) * dt;
          double py_in_100ms = v * sin(-steering_angle) * dt;
          double psi_in_100ms = - v /MPC::Lf * steering_angle* dt;
          double v_in_100ms = v + throttle_value * dt;

          //calculate the cte and epsi
          double cte_in_100ms = -py_in_100ms + polyeval(coeffs,px_in_100ms);
          // TODO: calculate the orientation error
          //double gradient = coeffs[1] + 2* coeffs[2] * px_in_100ms + 3* coeffs[3]*px_in_100ms*px_in_100ms;
          double gradient = coeffs[1];
          double epsi_in_100ms = psi_in_100ms - atan(gradient) ;

          Eigen::VectorXd state(6);
          state<< px_in_100ms,py_in_100ms,psi_in_100ms,v_in_100ms,cte_in_100ms,epsi_in_100ms;


          double steer_value;


          auto results= mpc.Solve(state,coeffs);
          steer_value = -results[0]/degree25_radian;
          throttle_value = results[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          size_t n = (results.size()-2)/2;
          mpc_x_vals.assign(results.begin()+2,results.begin()+2+n);
          mpc_y_vals.assign(results.begin()+2+n,results.end());
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          next_x_vals = ptsx_car;
          next_y_vals = ptsy_car;

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

void test_lake_track()
{
	MPC mpc;

	// NOTE: free feel to play around with these
	double px = 180;
	double py = 98;
	double psi = 0;
	double v = 30;

	vector<double> ptsx = { 179.3083, 172.3083, 165.5735, 151.7483,
			133.4783, 114.8083, 103.8935, 94.21827, 85.37355, 70.40827,
			61.24355, 45.30827, 36.03354, 15.90826, 5.64827, -9.99173,
			-24.01645, -32.16173, -43.49173 };
	vector<double> ptsy = { 98.67102, 117.181, 127.2894, 140.371,
			150.771, 156.621, 158.4294, 158.891, 158.731, 157.101, 155.4194,
			151.201, 148.141, 140.121, 135.321, 127.081, 119.021, 113.361,
			105.941 };

	double steering_angle = 0;

	//transform way points to car coordination
	vector<double> ptsx_car;
	vector<double> ptsy_car;

	for (size_t i = 0; i < 5; i++) {
		double x_car = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi);
		double y_car = (ptsy[i] - py) * cos(psi) - (ptsx[i] - px) * sin(psi);
		ptsx_car.push_back(x_car);
		ptsy_car.push_back(y_car);
	}

	//get the waypoints
	Eigen::Map<Eigen::VectorXd> xvals(ptsx_car.data(), ptsx_car.size());
	Eigen::Map<Eigen::VectorXd> yvals(ptsy_car.data(), ptsy_car.size());
	auto coeffs = polyfit(xvals, yvals, 3);
	/*
	 * TODO: Calculate steering angle and throttle using MPC.
	 *
	 * Both are in between [-1, 1].
	 *
	 */
	//predict cars position in 100 miliseconds
	v = v * 0.44704; //convert mph to m/s
	double dt = 0.1;
	double px_in_100ms = 0;
	double py_in_100ms = 0;
	double psi_in_100ms =0;

	//calculate the cte and epsi
	double cte_in_100ms = polyeval(coeffs, px_in_100ms);
	// TODO: calculate the orientation error
	double gradient = coeffs[1] + 2 * coeffs[2] * px_in_100ms
			+ 3 * coeffs[3] * px_in_100ms * px_in_100ms;
	double epsi_in_100ms = 0;
	psi_in_100ms = atan(gradient);

	Eigen::VectorXd state(6);
	state << px_in_100ms, py_in_100ms, psi_in_100ms, v, cte_in_100ms, epsi_in_100ms;

	auto results = mpc.Solve(state, coeffs);
}
