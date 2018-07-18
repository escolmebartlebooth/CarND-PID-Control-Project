#include "PID.h"
#include <iostream>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // set the initial gains and errors
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  this->p_error = 0;
  this->i_error = 0;
  this->d_error = 0;
  this->d_cte = 0;
  this->i_cte = 0;
  this->step_count = 0;
  this->mean_error = 0;

  this->best_error = 100000;
  this->t_state = 0;
  this->t_idx = 0;
  this->t_iter = 0;
  this->n_iter = 1500;
  this->tune_count = 0;
  this->p = {Kp,Ki,Kd};
  this->dp = {0.05,0.0005,0.25};

  cout << "PID Controller Initialised" << endl;
  cout << this->Kp << " " << this->Ki << " " << this->Kd << endl;
}

void PID::UpdateError(double cte) {
  // proportional bit
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

double PID::TotalError() {
  return p_error + i_error + d_error;
}

double PID::MeanError() {
  return mean_error;
}


void PID::TunePID() {
  if (this->t_state == 0) {
    this->best_error = this->MeanError();
    this->p[this->t_idx] += this->dp[this->t_idx];
    this->t_state = 1;
  } else if (this->t_state == 1) {
    if (this->MeanError() < this->best_error) {
      this->best_error = this->MeanError();
      this->dp[this->t_idx] *= 1.1;
      this->t_idx = (this->t_idx + 1) % 3;
      this->p[this->t_idx] += this->dp[this->t_idx];
      this->t_state = 1;
    } else {
      this->p[this->t_idx] -= 2 * this->dp[this->t_idx];
      if (this->p[this->t_idx] < 0) {
        this->p[this->t_idx] = 0;
        this->t_idx = (this->t_idx + 1) % 3;
      }
      this->t_state = 2;
    }
  } else {
    if (this->MeanError() < this->best_error) {
      this->best_error = this->MeanError();
      this->dp[this->t_idx] *= 1.1;
      this->t_idx = (this->t_idx + 1) % 3;
      this->p[this->t_idx] += this->dp[this->t_idx];
      this->t_state = 1;
    } else {
      this->p[this->t_idx] += this->dp[this->t_idx];
      this->dp[this->t_idx] *= 0.9;
      this->t_idx = (this->t_idx + 1) % 3;
      this->p[this->t_idx] += this->dp[this->t_idx];
      this->t_state = 1;
    }
  }
  this->Init(p[0], p[1], p[2]);
}
