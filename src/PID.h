#ifndef PID_H
#define PID_H

#include <vector>
#include <cmath>
#include <iostream>
#include <limits>

using namespace std;

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();


  void Twiddle(double cte);

  double count;
  bool reach_cruise_speed;

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;


  double total_error;
  double best_error;
  bool error_initilialize;
  bool new_best_error;

  bool previous_add;
  bool previous_sub;

  std::vector<double> dp;
  int index_dp;

  void modify_pid_parameters(int index, double value);
};



#endif  // PID_H