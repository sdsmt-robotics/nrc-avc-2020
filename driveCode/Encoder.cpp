/* 
 * Class for the Encoder.
 * This class provides functions for tracking encoder ticks and determining speed.
 * 
 * Note: this class is only set up to support one encoder. Attempting to use it for more will break things.
 */

#include "Encoder.h"

// for use by ISR routine
Encoder * Encoder::instance;

/**
 * @brief Constructor for the class.
 * 
 * @param trigPin - First encoder pin. Attached to interrupt.
 * @param ticksPerRotation - Number of encoder ticks per rotation.
 */
Encoder::Encoder(int trigPin, float ticksPerRotation, float minRpm) 
    : trigPin(trigPin), ticksPerRotation(ticksPerRotation), speedFilter(40, 40, 0.25)  { //speedFilter(300, 300, 0.025)
    timeOut = 60 * 1000 * 1000 / (ticksPerRotation * minRpm);
}

/**
 * @brief Initialize the encoder.
 */
void Encoder::init() {
    // Because doing ISRs with classes is painful.
    instance = this;
    
    //Setup control pins
    pinMode(trigPin, INPUT);
    
    //Set up the encoder interrupt pin
    int intNum = digitalPinToInterrupt(trigPin);
    attachInterrupt(intNum, Encoder::isr, RISING);
    
    //initialize the timer
    lastTickTime = micros();
    lastEstTime = lastTickTime;
}

/**
 * Update the current speed estimate and return the filtered value.
 */
int Encoder::estimateSpeed() {
  // Calculate the speed if we got a tick. Otherwise assume 0.
  if (ticks != 0) {
    speed = ticks * (tickConversion / (lastTickTime - lastEstTime));

    // Reset things
    lastEstTime = lastTickTime;
    ticks = 0;
  } else if (speed != 0 && micros() - lastTickTime > timeOut) {
    speed = 0;
  }
  
  // Filter
  filteredSpeed = speedFilter.updateEstimate(speed);

  // Return the speed
  return filteredSpeed;
}

/**
 * @brief Get the current speed the motor is running at in RPMs.
 * 
 * @return the current speed of the motor.
 */
int Encoder::getSpeed() {
  return speed;
}

/**
 * @brief Get the filtered current speed the motor is running at in RPMs.
 * 
 * @return the filtered current speed of the motor.
 */
int Encoder::getFilteredSpeed() {
  return filteredSpeed;
}

/**
 * Handle an encoder tick.
 */
void Encoder::tick() {
  lastTickTime = micros();

  ++ticks;
}

/**
 * @brief Handle interrupt
 */
void Encoder::isr() {
  instance->tick();
}
