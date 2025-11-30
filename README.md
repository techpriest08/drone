# drone
monocopter_drone

Graduation project started at Chungnam National University, Department of Electrical Engineering at 2025

I declare that received no assistance from any team member in purchasing parts like motors, designing, assembling, writing code, or conducting experiments.
Although it wasn't a complete success due to various issues, it should provide some ideas for those building monocopters.
After measuring the height using a barometric pressure sensor, the PWM duty was controlled using the pi control to adjust the rotation speed of the BLDC motor.

The reason for the failure was that the purchased dual ESC motor driver was not controlled individually but was controlled by receiving only one PWM signal.
If you're going to do a similar experiment, I highly recommend purchasing a separate ESC motor driver. It may be my fault, but sometimes the low-speed ones come bundled with the MCU.

I will also add what parts purchased in the future.
last update at 2025/12/01

When measuring height using air pressure, use this site and provide citations to the site.
https://ncar.github.io/aircraft_ProcessingAlgorithms/
UCAR/NCAR - Earth Observing Laboratory (2022) RAF Technical Note: Processing Algorithms. UCAR/NCAR - Earth Observing Laboratory. https://doi.org/10.26023/zc3gpm25 V2022.0
