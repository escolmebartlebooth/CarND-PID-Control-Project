#ifndef PID_H
#define PID_H

class PID {
private:
  /*
  * track errors
  */
  double d_cte;
  double i_cte;

  /*
  * track mean error over time
  */
  double mean_error;
  int step_count;

public:
  /*
  * Errors at each time step
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
  * Calculate and return the total PID error.
  */
  double TotalError();
  /*
  * Return the mean absolute PID error.
  */
  double MeanError();
};

#endif /* PID_H */
