#ifndef PID_H_
#define PID_H_

class PID{
public:
  PID(){};
  void set_PID(const double &Kp, const double &Ki, const double &Kd);
  void set_Kp(const double &Kp);
  void set_Ki(const double &Ki);
  void set_Kd(const double &Kd);
  void set_step_size(const double &step_size);
  void set_output_limit(const double &output_lower_limit, const double &output_upper_limit);
  void initialize();
  double control(const double &error);


private:
  double Kp_ = 0.0, Ki_ = 0.0, Kd_ = 0.0;
  double output_ = 0.0;
  double previous_output_ = 0.0;
  double error_ = 0.0;
  double previous_error_ = 0.0;
  double integral_ = 0.0;
  double step_size_ = 1.0;
  double output_upper_limit_ = 0.0;
  double output_lower_limit_ = 0.0;
  double first_hit = true;
};

#endif
