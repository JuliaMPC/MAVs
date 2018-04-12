#include "PID.h"
#include <iostream>

void PID::set_PID(const double &Kp, const double &Ki, const double &Kd){
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void PID::set_Kp(const double &Kp){
  Kp_ = Kp;
}

void PID::set_Ki(const double &Ki){
  Ki_ = Ki;
}

void PID::set_Kd(const double &Kd){
  Kd_ = Kd;
}

void PID::set_step_size(const double &step_size){
  step_size_ = step_size;
}

void PID::set_output_limit(const double &output_lower_limit,
                           const double &output_upper_limit){
  output_lower_limit_ = output_lower_limit;
  output_upper_limit_ = output_upper_limit;
}

void PID::initialize(){
  output_ = 0.0;
  previous_output_ = 0.0;
  error_ = 0.0;
  previous_error_ = 0.0;
  integral_ = 0.0;
}

double PID::control(const double &error){
  error_ = error;
  integral_ += error_;
  double diff = error_ - previous_error_;
  if(first_hit){
    first_hit = false;
    diff = 0;
  }
  output_ = Kp_ * error_ + Ki_ * integral_ + Kd_ * diff / step_size_;
  std::cout << Kp_ * error_ << " " << Ki_ * integral_ << " " <<  Kd_ * diff / step_size_ << std::endl;
  if (output_ < output_lower_limit_) output_ = output_lower_limit_;
  else if (output_ > output_upper_limit_) output_ = output_upper_limit_;

  previous_error_ = error_;
  previous_output_ = output_;
  return output_;
}
