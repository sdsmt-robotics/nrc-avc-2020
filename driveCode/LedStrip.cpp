/**********************************************************************
* LedStrip.cpp
* Date: 4/5/2019
* Author: Sam R.
* 
* Description: Class to control an RGB LED light strip. Uses one analog 
* pin for each color. Color and brightness can be toggled using keys. 
* Includes predefined colors for ease of use.
* 
* Usage: 
*   LedStrip(redPin, greenPin, bluePin) - initialize pins.
*   setColorRGB(red, green, blue) - set color values to 0-255.
*   setColorHEX(0x------) - set color values using hex.
*   setColorHEX(LedStrip::color) - set color values using predefined.
*   setBrightness(brightness) - set brightness (0-255).
*   printColor() - print out the RGB vals.
*   
**********************************************************************/

#include "LedStrip.h"

//=====Constructor used to initialize pins==============================
LedStrip::LedStrip(int rPin, int gPin, int bPin) {
  _redPin = rPin;
  _greenPin = gPin;
  _bluePin = bPin;

}

//=====Initialize pin outputs==============================
void LedStrip::init() {
    pinMode(_redPin, OUTPUT);
    pinMode(_greenPin, OUTPUT);
    pinMode(_bluePin, OUTPUT);
    
    //update values sent to strip
    setOutput();
}

//=====Set color using RGB vals==============================
void LedStrip::setColorRGB(int red,int green, int blue) {
  //update RGB values
  _redVal = constrain(red, 0, 255);
  _greenVal = constrain(green, 0, 255);
  _blueVal = constrain(blue, 0, 255);

  //update values sent to strip
  setOutput();
}


//=====Set color using hex val==============================
void LedStrip::setColorHEX(long int hexVal) {
  //get RGB vals from hex
  _blueVal = hexVal & 0xFF;
  hexVal >>= 8;
  _greenVal = hexVal & 0xFF;
  hexVal >>= 8;
  _redVal = hexVal & 0xFF;

  
  //update values sent to strip
  setOutput();
}


//=====Set strip brightness==============================
void LedStrip::setBrightness(int brightness) {
  _brightness = constrain(brightness, 0, 255);

  //update values sent to strip
  setOutput();
}


//=====Set values sent to strip==============================
void LedStrip::setOutput() {
  //set pin values using RGB vals scaled by brightness
  //val = color * (brightness / 256)
  analogWrite(_redPin, (_redVal * _brightness + 1) >> 8);
  analogWrite(_greenPin, (_greenVal * _brightness + 1) >> 8);
  analogWrite(_bluePin, (_blueVal * _brightness + 1) >> 8);
}


//=====Print the RGB values==============================
void LedStrip::printColor() {
  Serial.print("Red: ");
  Serial.print(_redVal);
  Serial.print(", Green: ");
  Serial.print(_greenVal);
  Serial.print(", Blue: ");
  Serial.print(_blueVal);
  Serial.print("\n");
}