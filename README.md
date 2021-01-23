


Running the GPS:
The GPS is currently the Adafruit Ultimate GPS Breakout board. This board has an internal antenna, but it is better to connect an external one to the antenna connector on top ofthe board. This will make the measurements more accurate. 

The GPS talks to the car over USB. It is connected to /dev/ttyUSB0 with a usb to Vcc-Gnd-Rx-Tx cable. 
Vcc - Vin
Gnd - Gnd
Rx - Tx
Tx - Rx

When the chip is powered, the built-on LED will flash every 1 sec with no fix, or every 15 sec with a fix.

To confirm that the chip is talking to the car, open a terminal and run 'cat /dev/ttyUSB0'. If you get no output, try running the command again, but after 10 tries you might try to reboot the jetson. Once you get output, you can try: 
'cat /dev/ttyUSB0 | grep 'GPGSV''. The GPGSV line tells you how many satellites the antenna is traching. Google it for more infor. 
'cat /dev/ttyUSB0 | grep 'GPGGA''. The GPGGA line tells you the time, and the current GPS fix. It will be mostly empty until the antenna is tracking at least 3 satellites. Tape the antenna to a window or go outside, and wait at least 2 minutes if you aren't getting a fix. 

To more easily view the data from the GPS, run 'roslaunch avc_nrc_2020 data_collection.launch'. Then in another teminal, run 'rostopic echo /fix_m'. This will output the GPS distance you are from the time you started the data_collection launch file. There is some error, so even sitting still the value may drift += 0.25m each measurement. The gps-m conversion is done by multiplying  the difference between the current point and the start a constant value. This is just math using the circumference of the earth and assuming it is a perfect sphere. There is some incredibly complex math for more accurate math, using the earth as an ellipsiod, but for our purposes even half a meter of inaccuracy over 100m is fine and this will still be far better than that. 
