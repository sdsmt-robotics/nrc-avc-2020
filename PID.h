/**
 * Header for the PID class.
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>

class PID {
    public:
        PID(float Kp, float Ki, float Kd, float N, unsigned long sample_time);

        float calculateOutput(float input);

        void setConstants(float Kp,float Ki, float Kd, float N, unsigned long sample_time);
        void setLimits(float min, float max);
        void setTarget(float target);
        float getTarget();
        void reset();

    private:
        float Ts = 2500/1000000.0; // Sample time in seconds
        float e2,e1,e0,u2,u1;         // Error and output terms
        float ku1,ku2,ke0,ke1,ke2;    // Transfer function coefficients for error and output terms
        float a0, a1, a2, b0, b1, b2; // Numerators and denominators of the coefficients

        // Output value limits
        float min = 0;
        float max = 100;

        // PID Constants
        float Kp = 10;
        float Ki = 1;
        float Kd = 1;
        float Kf = 0;
        float N = 20;

        float output;
        float target;

        void calculateCoeffs();
};


#endif
