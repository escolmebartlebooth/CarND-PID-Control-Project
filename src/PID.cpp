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

  cout << "PID Controller Initialised" << endl;
  cout << this->Kp << " " << this->Ki << " " << this->Kd << endl;
  cout << this->p_error << " " << this->i_error << " " << this->d_error << endl;

}

void PID::UpdateError(double cte) {
  // proportional bit
  p_error = -Kp*cte;
}

double PID::TotalError() {
  double total_error = p_error + i_error + d_error;
  if (total_error > 1) {
    return 1;
  } else if (total_error < -1) {
    return -1;
  } else {
    return total_error;
  }
}

