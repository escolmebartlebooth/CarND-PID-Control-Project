#include "PID.h"
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

/*
  * Initialiser for the class
  * Takes control gains and initialises class variables
*/
void PID::Init(double Kp, double Ki, double Kd) {
  // set the initial gains
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  // set the initial errors for the time step
  this->p_error = 0;
  this->i_error = 0;
  this->d_error = 0;

  // set the errors for tracking over time
  this->d_cte = 0;
  this->i_cte = 0;
  this->step_count = 0;
  this->mean_error = 0;

  // report initialisation complete
  cout << "PID Controller Initialised" << endl;
  cout << this->Kp << " " << this->Ki << " " << this->Kd << endl;
}


/*
  * Update the error at each time step
  * Take the current cross track error
  * Calculate and update the component errors
*/
void PID::UpdateError(double cte) {
  // proportional error
  p_error = -Kp*cte;

  // take difference between time steps for differential error
  d_error = -Kd*(cte - d_cte);
  d_cte = cte;

  // take sum over all error for integral step
  i_cte += cte;
  i_error = -Ki*i_cte;

  // mean error
  step_count += 1;
  double d1 = fabs(cte) - mean_error;
  mean_error = mean_error + (d1/step_count);
}


/*
  * return the sum of the PID error terms
*/
double PID::TotalError() {
  return p_error + i_error + d_error;
}


/*
  * return the mean error over time
*/
double PID::MeanError() {
  return mean_error;
}
