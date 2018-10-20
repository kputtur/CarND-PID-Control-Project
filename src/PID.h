#ifndef PID_H
#define PID_H

#include<vector>

class PID {
public:
  /*
  * Errors initalize it to zero
  */
  double p_error = 0;
  double i_error = 0;
  double d_error = 0;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
   * Twiddle variables
   */
  std::vector<double> dp;
  int step, param_index;

  //number of steps to allow changes to settle, then evaluate error
  int n_settle_steps, n_eval_steps;
  double total_error, best_error;

  bool try_adding, try_subtracting,  twiddle_or_not;



  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Convenience fn for adding amount of dp to a PID controller parameter based onindex
   */
  void ParameterAdditionAtIndex(int index, double amount);

};

#endif /* PID_H */
