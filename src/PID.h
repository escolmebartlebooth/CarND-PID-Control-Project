#ifndef PID_H
#define PID_H

#include <vector>

class PID {
private:
  /*
  * track cte change
  */
  double d_cte;
  double i_cte;
  double mean_error;
  int step_count;
  double best_error;
  double t_state;
  int t_idx;
  std::vector<double> p;
  std::vector<double> dp;

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

  /*
  * Tuning Variables
  */
  int t_iter;
  int n_iter;
  int tune_count;

  /*
  * Constructor
  */
  PID();

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
  * Self Tuning Algorithm.
  */
  void TunePID();

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  /*
  * Calculate the mean PID error.
  */
  double MeanError();
};

#endif /* PID_H */
