# MT_MTM
This repository contains all code for the MTM

The code extracts the sensor data for measuring the motion conducted by an operator. 

Signal Processing:
This includes a:

Spike detection algorithm for filtering random spikes in the orientation data provided by the IMU. The threshold T and the termination criteria are changeable.
Setting the termination criterion to zero disables the filter

Rollover detection algorithm which is necessary to prepare the received encoder data for the kinematic computations. 
The algorithm ensures that negative angular values are depictable and that the sensor data is continous over a sensor over- or underflow event.

A series of debugging options in the SerialPrintData function for evaluation of the data
