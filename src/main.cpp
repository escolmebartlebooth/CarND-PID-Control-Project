#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

double best_error = 100000;
double t_state = 0;
int t_idx = 0;
int t_iter = 0;
int n_iter = 1500;
int tune_count = 0;
std::vector<double> p = {0.105,0.002,1.0};
std::vector<double> dp = {0.05,0.0005,0.25};

void twiddle(PID &pid_steer) {
  if (t_state == 0) {
    best_error = pid_steer.MeanError();
    p[t_idx] += dp[t_idx];
    t_state = 1;
  } else if (t_state == 1) {
    if (pid_steer.MeanError() < best_error) {
      best_error = pid_steer.MeanError();
      dp[t_idx] *= 1.1;
      t_idx = (t_idx + 1) % 3;
      p[t_idx] += dp[t_idx];
      t_state = 1;
    } else {
      p[t_idx] -= 2 * dp[t_idx];
      if (p[t_idx] < 0) {
        p[t_idx] = 0;
        t_idx = (t_idx + 1) % 3;
      }
      t_state = 2;
    }
  } else {
    if (pid_steer.MeanError() < best_error) {
      best_error = pid_steer.MeanError();
      dp[t_idx] *= 1.1;
      t_idx = (t_idx + 1) % 3;
      p[t_idx] += dp[t_idx];
      t_state = 1;
    } else {
      p[t_idx] += dp[t_idx];
      dp[t_idx] *= 0.9;
      t_idx = (t_idx + 1) % 3;
      p[t_idx] += dp[t_idx];
      t_state = 1;
    }
  }
  pid_steer.Init(p[0], p[1], p[2]);
}

int main(int argc, char* argv[])
{
  uWS::Hub h;

  // get control parameters - consider getopt next time...
  // looking for -t(une) -s(peed)
  int c = 1;
  int tune_controller = -1;
  int use_throttle_controller = -1;

  std::vector<double> coeffs = {0.105,0.002,1.0};
  std::vector<double> throttle_coeffs = {0.5,0.0,1.0};
  int count_params = 0;
  while (c < argc) {
    if (strcmp(argv[c],"-t") == 0) {
      tune_controller = 0;
      std::cout << "TUNING!" << std::endl;
    } else if (strcmp(argv[c],"-s") == 0) {
        use_throttle_controller = 0;
        std::cout << "throttle control on!" << std::endl;
      } else {
        if (count_params < 3) {
          coeffs[count_params] = atof(argv[c]);
          count_params += 1;
        } else if (count_params < 6) {
          throttle_coeffs[count_params-3] = atof(argv[c]);
          count_params += 1;
          }
      }
    c += 1;
  }

  PID pid;
  // TODO: Initialize the pid variable.
  pid.Init(coeffs[0], coeffs[1], coeffs[2]);

  PID t_pid;
  // TODO: Initialize the pid variable.
  t_pid.Init(throttle_coeffs[0], throttle_coeffs[1], throttle_coeffs[2]);

  h.onMessage([&pid, &t_pid, &tune_controller, &use_throttle_controller](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          if (steer_value > 1.0) {
            steer_value = 1.0;
          }
          if (steer_value < -1.0) {
            steer_value = -1.0;
          }

          if (use_throttle_controller == 0) {
            t_pid.UpdateError(cte);
            throttle = 1.0 - fabs(t_pid.TotalError());
          } else {
            throttle = 0.3;
          }
          if (tune_controller == 0) {
            pid.t_iter += 1;
            if (pid.t_iter > pid.n_iter) {
              pid.tune_count += 1;
              double mte = pid.MeanError();
              pid.TunePID();
              pid.t_iter = 0;
              std::string msg = "42[\"reset\", {}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              std::cout << mte << "|" << tune_count << std::endl;
            } else {
              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              // std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
            // DEBUG
            //std::cout << "CTE|" << cte << "|Steering Value|" << steer_value;
            //std::cout << "|angle|" << angle;
            //std::cout << "|speed|" << speed << std::endl;
          } else {
            // DEBUG
            std::cout << cte << "|" << steer_value;
            std::cout << "|" << angle << "|" << throttle;
            std::cout << "|" << speed << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
