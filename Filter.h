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

    double filter(double newVal);
    double getFilteredVal();
    void reset(double fillVal = 0);

private:
    unsigned bufferSize;  //size of the input buffer
    double* buffer;          //buffer of input values
    unsigned curIndex = 0;     //index in the input values buffer
    double filteredVal = 0;  //filtered result
    double total = 0;       //running total of the values
};

#endif
