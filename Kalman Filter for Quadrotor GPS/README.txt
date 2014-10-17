There are two parts to the code:

- the first Matlab implementation can be found in the KalmanFilter.m file. This code takes as an input ‘dataFilePath’ which can take either of the following csv-file paths: [‘Data/loop_walk’, ‘Data/aroundTechGreen’].
The code computes the Matlab offline Kalman filter and plots both the results of this offline computations, as well as the saved results from the online Kalman computation

- the Arduino implementations can be found in the Arduino folder. The ‘ArduinoKalman_Offline.ino’ file shows the first offline implementation of the Kalman Filter that was run on collected data. The ‘ArduinoKalman_Online.ino’ file has the modified version to take as input the real-time GPS inputs from the GPS onboard of the quad rotor. Finally, the ‘ArduinoKalman_Online_With_Interpolation.ino’ code computes the interpolation data points when the GPD frequency is too low.

The Arduino code runs with the two libraries NazaDecoderLib and MatrixMath. The former library is used by Arduino to interpret the GPS data sent from the GPS device to the Arduino board via Serial communications. The latter is called inside the Kalman Filter implementation to do all the matrix computations.

The data sets given with the code are two examples of data collection with online Kalman filtering.

I worked on this code with Josephine Simon and Ning Wang. 