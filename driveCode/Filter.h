/* 
 * Header for the filter class.
 */

#ifndef FILTER_H
#define FILTER_H

#include "Arduino.h"

class Filter {
public:
    Filter(unsigned size);
    ~Filter();

    int filter(int newVal);
    int getFilteredVal();
    void reset(int fillVal = 0);

private:
    unsigned bufferSize;  //size of the input buffer
    int* buffer;          //buffer of input values
    unsigned curIndex = 0;     //index in the input values buffer
    int filteredVal = 0;  //filtered result
    long total = 0;       //running total of the values
};

#endif
