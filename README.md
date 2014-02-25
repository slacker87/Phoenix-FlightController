Flight Controller based on K20DX256 (Teensy 3.1), forked from cTn
------------------------------------------------

PIN setup [Teensy 3.1](https://www.pjrc.com/teensy/teensy31.html) pin numbering:
------------------------------------
  - I2C SCL 19
  - I2C SDA 18
  
  - SUMD RX1 ( Receiver, 115kBaud ) 0
  - Alternative PPM/PWM 3
  
  - BT RX2 9
  - BT TX2 10
  
  - GPS RX3 7
  - GPS TX3 8
 
  - Sonar 11
  - Sonar 12
  
  - UMon 14
  - UMon 15
  
  - LED Stripe White 2
  - LED Stripe Red 4
  - LED Stripe MultiColor 6 ? 
  - LED Internal 13

  - Rotor 1 22
  - Rotor 2 23
  - Rotor 3 20
  - Rotor 4 21
  
  - Battery Monitor (current sensor) 17
  
Filters, kinematics, data handling
----------------------------------
  - Initial raw data from sensors (read every 1ms = 1000Hz) is being averaged by a simple averaging filter
  - Averaged data from sensors are being processed every 10ms (100Hz)
  - CMP kinematitcs (my own) selected by default
  - ARG kinematics (from aeroquad) is also supported (can be enabled by simple include change)
  - DCM kinematics (from FreeIMU)
  - Flight controller supports
    - Rate (ACRO) | gyro only
    - Attitude | gyro with accel corrections
    - Altitute hold | barometer or sonar
  - Pilot commands are being read via single PIN (with HW timer or USART), PPM, PWM and SUMD in is supported
  - Stabilization and pilot commands are mixed together by 2 separate PID controllers
    - First (only used in attitude mode) mixes pilot commands with kinematics output
    - Second (used in both attitude and rate mode) mixes output from first PID or raw stick input with gyroRate output
  - For ESC signal output i am using an build in 8 channel FLEX timer (yes you can controll octocopter with this)
    - ESC PWM signal supports both 250Hz and 400Hz update rate (running at 400Hz by default)

Accelerometer offset trimming via TX
------------------------------------
  - While in dis-armed state
  - To enter the trimming mode = Throttle stick UP & Rudder LEFT
  - To adjust the values = PITCH UP & DOWN || ROLL LEFT & RIGHT
  - To save the values in EEPROM = Throttle UP & Rudder RIGHT 
    - (you dont have to save the values right away, you can test them, then return back to trimming mode and save them)
  - To leave trimming mode = Throttle stick DOWN