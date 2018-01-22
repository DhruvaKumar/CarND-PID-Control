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

int main()
{
  uWS::Hub h;

  PID pid("steering"), pid_throttle("throttle");

  // Initialize the pid variable.
  pid.Init(0.065, 0.0, 0.6);
  // pid.Init(0.1, 0.0, 0.7);
  // Kp=0.258914 Kd=1.45 Ki=0.000700685
  // Kp=0.182477 Kd=1.35468 Ki=0.00129725 
  // pid.Init(0.18, 0.0013, 1.355);
  
  std::cout << "pid-steering: Kp=" << pid.Kp << " Ki=" << pid.Ki << " Kd=" << pid.Kd << std::endl;

  // pid_throttle.Init(0.15, 0, 0.5);
  pid_throttle.Init(0.05, 0, 0.1); // +angle
  
  std::cout << "pid-throttle: Kp=" << pid_throttle.Kp << " Ki=" << pid_throttle.Ki << " Kd=" << pid_throttle.Kd << std::endl;

  // run twiddle?
  pid.autotune = false;
  std::cout << "Running pid-steering twiddle=" << pid.autotune << std::endl;
  pid_throttle.autotune = false;
  std::cout << "Running pid-throttle twiddle=" << pid_throttle.autotune << std::endl;

  h.onMessage([&pid, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          // double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, throttle;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = -pid.TotalError();
          if (steer_value < -1.0) steer_value = -1.0;
          if (steer_value > 1.0) steer_value = 1.0;
          
          pid_throttle.UpdateError(std::abs(cte)+std::abs(angle/pi()));
          // pid_throttle.UpdateError(std::abs(cte));
          throttle = 0.5 - pid_throttle.TotalError(); // +angle
          if (throttle < 0.0) throttle = 0.0;
          // if (throttle < -1.0) throttle = -1.0;
          if (throttle > 1.0) throttle = 1.0;

          // // // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << "speed: " << speed << "angle: " << angle << "angle/pi: " << angle/pi() << std::endl;
          // std::cout << "cte+angle: " << std::abs(cte)+std::abs(angle) << "\n";
          // std::cout << "throttle: " << throttle << "\n";
          // std::cout << "pid-steering: Kp=" << pid.Kp << " Kd=" << pid.Kd << " Ki=" << pid.Ki << std::endl;
          // std::cout << "pid-steering: perr=" << pid.p_error << " derr=" << pid.d_error << " ierr=" << pid.i_error << std::endl;
          // if (pid.autotune)
          //   std::cout << "step: " << pid.step << "twiddle_step: " << pid.twiddle_step << "\n";

          // std::cout << "pid-throttle: Kp=" << pid_throttle.Kp << " Kd=" << pid_throttle.Kd << " Ki=" << pid_throttle.Ki << std::endl;
          // std::cout << "pid-throttle: perr=" << pid_throttle.p_error << " derr=" << pid_throttle.d_error << " ierr=" << pid_throttle.i_error << std::endl;


          // reset sim for a new twiddle run
          if (pid.autotune && pid.reset_sim)
          {
            std::cout << "reseting sim...\n"; 
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            pid.reset_sim = false;
            return;
          }
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          // msgJson["throttle"] = 0.3;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT); 
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
