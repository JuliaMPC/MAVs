#ifndef PID_H_
#define PID_H_
#include <string>

class PID{
public:
  PID() {
    Kp_ = 0.0, Ki_ = 0.0, Kd_ = 0.0, Kw_ = 0.0;
    output_ = 0.0;
    previous_output_ = 0.0;
    error_ = 0.0;
    previous_error_ = 0.0;
    integral_ = 0.0;
    windup_integral_ = 0.0;
    step_size_ = 1.0;
    output_upper_limit_ = 0.0;
    output_lower_limit_ = 0.0;
    first_hit = true;
    std::string windup = "clamping";
  }

  void set_PID(const double &Kp, const double &Ki, const double &Kd, const double &Kw);
  void set_Kp(const double &Kp);
  void set_Ki(const double &Ki);
  void set_Kd(const double &Kd);
  void set_Kw(const double &Kw);
  void set_step_size(const double &step_size);
  void set_output_limit(const double &output_lower_limit, const double &output_upper_limit);
  void set_windup_metohd(const std::string &s);
  void initialize();
  double control(const double &error);


private:
  double Kp_, Ki_, Kd_, Kw_;
  double output_;
  double previous_output_;
  double error_;
  double previous_error_;
  double integral_;
  double windup_integral_;
  double step_size_;
  double output_upper_limit_;
  double output_lower_limit_;
  double first_hit;
  std::string windup;
};

#endif
