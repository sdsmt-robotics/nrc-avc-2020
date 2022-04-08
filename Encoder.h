/* 
 * Header for the Encoder class.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"
#include <SimpleKalmanFilter.h>  // https://github.com/denyssene/SimpleKalmanFilter

class Encoder {
public:
    Encoder(int trigPin, float ticksPerRotation, float minRpm);
  
    void init();
    int estimateSpeed();
    int getSpeed();
    int getFilteredSpeed();
    float getRevCount();
    void resetRevCount();
    
private:
    //Static stuff for interrupt handling
    static void isr();
    static Encoder* instance;
    void tick();

    // Pins and pin registers
    int trigPin;
    
    volatile int ticks = 0;  // Number of ticks since last speed estimate
    volatile unsigned int totTicks = 0;  // Number of ticks since last speed estimate

    volatile int speed; //current speed of the motor in rpm
    int filteredSpeed; //current filtered speed of the motor in rpm

    volatile unsigned long lastTickTime; //time in microseconds of the last encoder tick
    volatile unsigned long lastEstTime; //time in microseconds of the tick preceding the last estimate

    // Conversion from (ticks / us) -> (rot / min)
    float ticksPerRotation = 360;
    unsigned long tickConversion = 1000000ul * 60 / ticksPerRotation;
    unsigned long timeOut = 1000*1000ul;

    /*
     SimpleKalmanFilter(e_mea, e_est, q);
     e_mea: Measurement Uncertainty 
     e_est: Estimation Uncertainty 
     q: Process Noise
     */
    SimpleKalmanFilter speedFilter;

};

#endif
