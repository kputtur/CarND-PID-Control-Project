#include "PID.h"

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

}

/* Return the total error of each coefficient multiplied by the respective error
 */

double PID::TotalError() {
	return  -Kp * p_error - Kd * d_error - Ki * i_error;
}
