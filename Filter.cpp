/**
 * @file Filter.cpp
 * 
 * Class for a filter. Used to filter some input based on the history of that value.
 * 
 * 
 * int filter(int newVal) - add a raw value and return the filtered value.
 * int getFilteredVal() - get the filtered value
 * void reset(int fillVal = 0) - reset the filter buffer
 * 
 */

#include "Filter.h"

/**
 * @brief Constructor for the class.
 * 
 * @param size - size for the filter buffer.
 */
Filter::Filter(unsigned size)
{

  //sets up the buffer array
  this->bufferSize = size; //"this" unnecessary

  //allocate our array
  buffer = new double[size];

  //reset the values in the buffer
  reset();
}

/**
 * @brief Destructor for the class.
 * 
 */
Filter::~Filter()
{
  delete[] buffer;
}

/**
 * @brief Reset the filter buffer.
 * 
 * Use this when using for the first time in a long time (or ever).
 * 
 * @param fillVal - initial value for the filter. (Defaults to 0.)
 */
void Filter::reset(double fillVal)
{
  //fill the buffer with the given value
  for (int i = 0; i < bufferSize; i++)
  {
    buffer[i] = fillVal;
  }

  //update the running totals
  total = fillVal * bufferSize;

  //update the filtered value
  filteredVal = fillVal;
}

/**
 * @brief Add a new value to the filter and get the filtered output.
 * 
 * @param newVal - value to add
 * @return the filtered value
 */
double Filter::filter(double newVal)
{
  double oldVal = buffer[curIndex];

  //add the new value
  buffer[curIndex] = newVal;

  //update position in the rolling array
  curIndex++;
  if (curIndex >= bufferSize)
  {
    curIndex = 0;
  }

  //get the new filtered value
  total = total - oldVal + newVal;
  filteredVal = total / bufferSize;

  //return the filtered value
  return filteredVal;
}

/**
 * @brief Get the filtered value from the filter.
 * 
 * @return the filtered value.
 */
double Filter::getFilteredVal()
{
  return filteredVal;
}
