/**
 * Discrete PID class to calculate an output for a process given an input.
 * 
 * Reference: https://www.scilab.org/discrete-time-pid-controller-implementation
 */


#include "PID.h"

/**
 * Constructor for the class.
 * 
 * @param Kp - proportional term gain
 * @param Ki - integral term gain
 * @param Ki - dirivative term gain
 * @param N - filtering amount
 */
PID::PID(float Kp,float Ki, float Kd, float N, unsigned long sample_time)
{
    setConstants(Kp, Ki, Kd, N, sample_time);
}

/**
 * Set the constants for the PID calculations.
 * 
 * @param Kp - proportional term gain
 * @param Ki - integral term gain
 * @param Ki - dirivative term gain
 * @param N - filtering amount
 * @param sample_time - time between updates in ms
 */
void PID::setConstants(float Kp, float Ki, float Kd, float N, unsigned long sample_time)
{
    this -> Kd = Kd;
    this -> Kp = Kp;
    this -> Ki = Ki;
    this -> N  = N;
    this -> Ts = sample_time/1000.0;
    
    calculateCoeffs();
}

/**
 * @brief Calculate the constant coeifficients for the PID control.
 */
void PID::calculateCoeffs()
{
    // Numerators
    b0 = Kp * (1+N*Ts) + Ki * Ts * (1 + N * Ts) + Kd * N;
    b1 = -( Kp * ( 2 + N * Ts ) + Ki * Ts + 2 * Kd * N );
    b2 = Kp + Kd*N;
    
    // Denominators
    a0 = (1+ N * Ts);
    a1 = - (2 + N * Ts);
    a2 = 1;

    // Coefficients for output terms
    ku1 = a1/a0;
    ku2 = a2/a0;

    // Coefficients for error terms
    ke0 = b0/a0;
    ke1 = b1/a0;
    ke2 = b2/a0;
}

/**
 * @brief Calculate the new output given the current input.
 * 
 * @param input - the current measured value
 * @return the calculated output value.
 */
float PID::calculateOutput(float input)
{
    // Roll the output and error values
    // TODO: might be worth using a rolling array here
    e2 = e1;
    e1 = e0;
    u2 = u1;
    u1 = output;

    // Calculate the new error and output
    e0 = target - input;
    output = -ku1 * u1 - ku2 * u2 + ke0 * e0 + ke1 * e1 + ke2 * e2;
    //output = target*Kf;
    
    // Constrain and return the output
    if (output > max) {
        return max;
    } else if (output < min) {
        return min;
    }
    return output;
}

/**
 * @brief Set the limits for returned output values.
 * 
 * @param min - the minimum returnable value.
 * @param max - the maximum returnable value.
 */
void PID::setLimits(float min, float max)
{
    this->min = min;
    this->max = max;
}

/**
 * @brief Set the target value for the process.
 * 
 * @param target - the new target value.
 */
void PID::setTarget(float target) {
  this->target = target;
}

/**
 * @brief Get the current target value.
 */
float PID::getTarget() {
  return target;
}

/**
 * @brief Reset the PID error and output history.
 */
void PID::reset()
{
    // Reset errors
    e0 = 0;
    e1 = 0;
    e2 = 0;

    // Reset outputs
    output = 0;
    u1 = 0;
    u2 = 0;

}
