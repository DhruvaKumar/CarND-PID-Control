#ifndef PID_H
#define PID_H
#include <vector>
#include <fstream>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  int step;
  std::string name;

  // twiddle params
  bool autotune = false;
  int N_total_steps = 500; // total steps to execute for a twiddle run
  int threshold_step_to_record = 100; // steps after which twiddle should start recording error
  bool reset_sim = false; // flag to notify main to reset sim after a twiddle run
  double twiddle_error = 0.0;
  std::vector<double> dp;
  int index; // 0:Kp, 1:Kd, 2:Ki
  int twiddle_step;
  double best_twiddle_error;
  bool tried_increasing, tried_decreasing;
  int tolerance;
  std::ofstream twiddlef;
  /*
  * Constructor
  */
  PID();
  PID(std::string name);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

private:
  
  void IncrementK(int index, double dp);
};

#endif /* PID_H */
