#include <iostream>
#include "PID.h"

using namespace std;

int main(){
  PID controller;
  controller.set_PID(-10.0, 0.1, 0.0001);
  controller.set_step_size(0.01);
  controller.set_output_limit(0, 10);
  controller.initialize();

  double output;
  output = controller.control(1.0);
  cout << "Output: " << output << endl;
  output = controller.control(2.0);
  cout << "Output: " << output << endl;

  return 0;
}
