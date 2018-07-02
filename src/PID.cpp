#include "PID.h"

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

  std::cout << "PID Controller Initialised" << std::endl;
  std::cout << this->Kp << " " << this->Ki << " " << this->Kd << std::endl;
  std::cout << this->p_error << " " << this->i_error << " " << this->d_error << std::endl;

}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
}

