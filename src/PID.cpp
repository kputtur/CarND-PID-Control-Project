#include "PID.h"
#include<cmath>
#include<iostream>
#include<limits>

using namespace std;

/*
* DONE: Complete the PID class.
* Initialize the constructor and PID control, update the CTE(cross track error) and update PID
* twiddle, coefficients and calculate the total error of steering angle.
*/

//constructor
PID::PID() {}

//virtual destructor
PID::~PID() {}


/* Initialize the PID controller with the coefficients as the input values
 */
void PID::Init(double Kp_local, double Ki_local, double Kd_local) {
	this->Kp = Kp_local;
	this->Ki = Ki_local;
	this->Kd = Kd_local;

	//Twiddling parameters 
	twiddle_or_not = false;
	dp = {0.1 * Kp_local, 0.1 * Kd_local, 0.1 *Ki_local};
	step = 1;
	param_index = 2;  //this will be 0 after first twiddle loop
	n_settle_steps = 100;
	n_eval_steps = 2000;
	total_error = 0;
	best_error = std::numeric_limits<double>::max();
	try_adding = false;
	try_subtracting = false;
}

/* UpdateError fn updates the error values for calculating total error below.
 */

void PID::UpdateError(double cte) {


	// d_error is different from old cte(p_error) to the new cte
	d_error = (cte - p_error);
	//p_error set to new cte
	p_error = cte;
	//i_error is the num of ctes to this point
	i_error += cte;

	/*
	//update total error only if we are past number of settle steps
	if ( step % (n_settle_steps + n_eval_steps) > n_settle_steps) {
		total_error += pow(cte,2);
	 }
	
	if ( twiddle_or_not && step % (n_settle_steps + n_eval_steps) == 0) {
		std::cout << "step :  " << step << std::endl;
     		std::cout << "total error : " << total_error << std::endl;  			       
		std::cout << "best error :  " << best_error << std::endl;

		if ( total_error < best_error) {
			std::cout << "improvement " << std::endl;
			best_error = total_error;

			if( step != n_settle_steps + n_eval_steps) {
				dp[param_index] *= 1.1;
			}

			param_index = (param_index + 1 ) % 3;
			try_adding = try_subtracting = false;
		}

		if (!try_adding && !try_subtracting) {
		  ParameterAdditionAtIndex(param_index, dp[param_index]);
	  	  try_adding = true;
		}

		else if (try_adding && !try_subtracting) {
		  ParameterAdditionAtIndex(param_index, -2 * dp[param_index]);
  		  try_subtracting = true;
	        }
		else {
			ParameterAdditionAtIndex(param_index, dp[param_index]);
			dp[param_index] *= 0.9;
			param_index = (param_index + 1) % 3;
			try_adding = try_subtracting = false;
		}
			total_error = 0;
			std::cout<< "new parameters" << std::endl;
			std::cout<<" P: " << Kp << ", I: " << Ki << ", D: " << Kd << std::endl;
		}
		step++;
	*/
	
}

/* Return the total error of each coefficient multiplied by the respective error
 */

double PID::TotalError() {
	return  -Kp * p_error - Kd * d_error - Ki * i_error;
//	return 0.0;	
}

void PID::ParameterAdditionAtIndex(int index, double amount) {
	if (index == 0) {
		Kp += amount;
	}
	else if (index == 1) {
		Kd += amount;
	}
	else if (index == 2) {
		Ki += amount;
	}
	else {
		std::cout << "ParameterAdditionAtIndex: Index out of bounds " << std::endl;
	}
}
