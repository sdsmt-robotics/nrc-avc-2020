/**********************************************************************
* LedStrip.h
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

#ifndef LedStrip_h
#define LedStrip_h

#include "Arduino.h"

/**********************************************************************
* Class to control an RGB LED light strip. Uses one analog
* pin for each color.
**********************************************************************/
class LedStrip {
  public:
    LedStrip(int rPin, int gPin, int bPin);
    void init();
    void setColorRGB(int red,int green, int blue);
    void setColorHEX(long int hexVal);
    void setBrightness(int brightness);
    void printColor();

    //define some colors
    static const long Red = 0xFF0000;
    static const long Pink = 0xFFC0CB;
    static const long HotPink = 0xFF69B4;
    static const long OrangeRed = 0xFF4500;
    static const long Orange = 0xFFA500;
    static const long Gold = 0xFFD700;
    static const long Yellow = 0xFFFF00;
    static const long Violet = 0xEE82EE;
    static const long Purple = 0x800080;
    static const long GreenYellow = 0xADFF2F;
    static const long Green = 0x00FF00;
    static const long Teal = 0x008080;
    static const long Cyan = 0x00FFFF;
    static const long Blue = 0x0000FF;
    static const long SaddleBrown = 0x8B4513;
    static const long White = 0xFFFFFF;
    
  private:
    int _redPin;
    int _greenPin;
    int _bluePin;

    int _brightness = 0;
    int _redVal = 0;
    int _greenVal = 0;
    int _blueVal = 0;

    void setOutput();
};


#endif