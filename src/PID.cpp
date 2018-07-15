#include "PID.h"
#include <iostream>

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
  this->squared_error = 0;
  this->d_cte = 0;
  this->i_cte = 0;

  cout << "PID Controller Initialised" << endl;
  cout << this->Kp << " " << this->Ki << " " << this->Kd << endl;
  cout << this->p_error << " " << this->i_error << " " << this->d_error << endl;
  cout << this->d_cte << " " << this->i_cte << endl;
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

  // track sum of squared errors
  squared_error += pow(cte, 2);
}

double PID::TotalError() {
  return p_error + i_error + d_error;
}

