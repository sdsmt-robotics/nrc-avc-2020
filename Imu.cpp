/************************************************************************/ /**
*@file
*@brief Contains the POV class functions.
***************************************************************************/
#include "Imu.h"

/** ***************************************************************************
* @par Description:
* Constructor for the Imu class. This constructor basically sets the 
* the initial arrays to zero and assigns conversions to the relevant
* sensor structors.
*
* @returns none.
*****************************************************************************/
Imu::Imu()
{
    int i;

    for (i = 0; i < num_input; ++i)
    {
        GYRO_vals[i] = 0;
        AVG_gyro_vals[i] = 0;
    }

    gyro_z.conversion = &gyro_to_rad;
}

/** ***************************************************************************
* @par Description:
* Destructor for the Imu class. Deletes pointers...
* 
* @returns none.
*****************************************************************************/
Imu::~Imu() 
{
    return;
}

/** ***************************************************************************
* @par Description:
* Initialises the class by calibrating the sensors and testing if the IMU 
* could be initialized.
*
* @returns true if successful and false otherwise.
*****************************************************************************/
bool Imu::init()
{
    int i = 0;
    int j;
    Serial.println("Starting...");

    //test if the imu has started successfully
    //if (!IMU.begin())
    //    return false;
    IMU.begin();
    Serial.println("Reading 100...");
    //average 100 values from the from the IMU to use for calibration
    while (i < 100)
    {
        if (IMU.gyroscopeAvailable() /*&& IMU.accelerationAvailable()*/)
        {
            //when the gyroscope is avalible, read its values in
            IMU.readGyroscope(GYRO_vals[gyroscope_x], 
                GYRO_vals[gyroscope_y], GYRO_vals[gyroscope_z]);
            
            for (j = 0; j < num_input; ++j)
            {
                //sum all measurements
                AVG_gyro_vals[j] = AVG_gyro_vals[j] + GYRO_vals[j];
            }
            ++i;
            delay(5);
        }
    }
    Serial.println("Averaging...");

    //divide by the total number of measurements
    for (i = 0; i < num_input; ++i)
    {
        AVG_gyro_vals[i] = AVG_gyro_vals[i] / 100;
    }

    return true;
}

/** ***************************************************************************
* @par Description:
* Get the raw values for the IMU adjusted by their average error.
*
* @returns true if an update was available and false otherwise.
*****************************************************************************/
bool Imu::Get_raw()
{
    int i;

    //if the gyroscope is ready, take the new values and subtract the
    //average static error.
    if (IMU.gyroscopeAvailable() /*&& IMU.accelerationAvailable()*/)
    {

        //when the gyroscope is avalible, read its values in
        IMU.readGyroscope(GYRO_vals[gyroscope_x], 
            GYRO_vals[gyroscope_y], GYRO_vals[gyroscope_z]);

        //IMU.readAcceleration(GYRO_vals[accelerometer_x], 
        //    GYRO_vals[accelerometer_y], GYRO_vals[accelerometer_z]);

        for (i = 0; i < num_input; ++i)
        {
            GYRO_vals[i] = GYRO_vals[i] - AVG_gyro_vals[i];
        }

        //fileter gyroscope z value
        GYRO_vals[gyroscope_z] = gyro_z.rf.filter(GYRO_vals[gyroscope_z]*100000) /100000.00;

        //imu was ready
        return true;
    }

    //imu was not ready
    return false;
}

/** ***************************************************************************
* @par Description:
* Performs simpsons 1/3 integration on the last three data points without
* multiplying by delta t. The function doesn't return useful values until
* the first three values are passed in and you must pass in at least three
* new values to completely clear the function.
*
* @param [in] new_val : the newest value to be used in the estimation.
* @param [in/out] data : a structure that contains all relevant data for 
* the estimation.
*
* @returns float - the estamie of the integral value not multiplied by 
* delta t from the last three values passed in.
*****************************************************************************/
float Imu::integrate(integrate_struct &data, float new_val)
{
    //based off of simpsons 1/3 to find the approximate integral

    //scan through the array to add the new value
    if (data.mid > 2)
    {
        data.mid = 0;
        data.vals[2] = new_val;
    }
    else if (data.mid == 0)
    {
        data.vals[0] = new_val;
    }
    else
    {
        data.vals[1] = new_val;
    }

    //update the new mid of the array
    ++data.mid;

    //compute simpsons 1/3 for the past three values without multiplying by delta t
    return ((data.vals[0] + data.vals[1] + data.vals[2] + 3 * data.vals[data.mid - 1]) / 3);
}

/** ***************************************************************************
* @par Description:
* Update the estimation for the BB orientation.
*
* @returns true of a major change was made based on new values and 
* false otherwise.
*****************************************************************************/
bool Imu::update()
{

    //get new raw values
    bool new_vals = Get_raw();

    //if there are new raw values then recalculate the speed
    if (new_vals)
    {

        //intigrate the gyroscope_z value
        gyro_z.speed = integrate(gyro_z, GYRO_vals[gyroscope_z]);
    }

    //update time since last update
    delta_t = float(micros() - loop_start_time) / 1000000;

    //recalculate the integral
    gyro_z.integral = gyro_z.integral + gyro_z.speed * delta_t;

    //update the last time an update occurred
    loop_start_time = micros();

    return new_vals;
}

/** ***************************************************************************
* @par Description:
* Convert from the generic integral value to 2pi radians.
*
* @param [in/out] data : a structure that contains all relevant data for 
* the estimation.
*
* @returns float of the estimated bb angle.
*****************************************************************************/
float Imu::make2pi(integrate_struct &data)
{
    //**********integral to rad**********
    if(data.integral > *(data.conversion))
    {
        data.integral = data.integral - *(data.conversion);
    }
    else if (data.integral < 0)
    {
        data.integral = data.integral + *(data.conversion);
    }

    //convert the integral to radians

    return remainderf(((PI * 2) / *(data.conversion) * data.integral + offset) + PI, 2*PI) + PI;
}

/** ***************************************************************************
* @par Description:
* Return a float of the estimated bb angle. 
*
* @returns float of the estimated bb angle.
*****************************************************************************/
float Imu::Get_val()
{
    return make2pi(gyro_z);
}

/** ***************************************************************************
* @par Description:
* Takes a offset betwen 0 and 2PI and adds it to the return value.
*
* @returns WIP
*****************************************************************************/
void Imu::Set_offset(float new_offset)
{
    offset = remainderf(abs(new_offset), 2*PI);
}


void Imu::reset() {
    gyro_z.integral = 0;
    loop_start_time = micros();
}

/** ***************************************************************************
* @par Description:
* Tests the gyroscope to see if the bot is upright
*
* @returns true if the bot is upright or false if the bot is upside down
*****************************************************************************/
bool Imu::Get_upright()
{
    if(GYRO_vals[accelerometer_z] < -0.9)
    {
        return false;
    }

    return true;
}
