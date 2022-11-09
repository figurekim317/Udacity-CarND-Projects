#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_)
{
    /**
     * TODO: Initialize PID coefficients (and errors, if needed)
     */
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    
    p_error = 0;
    i_error = 0;
    d_error = 0;

    time_step = 1;
}

void PID::UpdateError(double cte)
{
    /**
     * TODO: Update PID errors based on cte.
     */

    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError()
{
    /**
     * TODO: Calculate and return the total error
     */
    double total_error = Kp*p_error + Kd*d_error + Ki*i_error;

    return total_error;
}

double PID::AverageCTE(double cte) {
    average_cte = (double)time_step/((double)time_step+1) * average_cte + (double)1/((double)time_step+1) * cte;
    time_step += 1;

    return average_cte;
}