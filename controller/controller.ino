/* 
    Phoenix flight controller was initially build to support only one array of sensors but
    with minor sensor changes it seemed like a waste to just "drop" previous sensor support
    with this idea in mind i will try to make the flight controller feature set flexible as
    possible to accommodate as much hw as possible.

    Defines below only enable/disable "pre-defined" hw setups, you can always define your own
    setup in the == Hardware setup == section.
*/

// Arduino standard library imports
#include <Arduino.h>
#include <Wire.h>
//#include <i2c_t3.h>	// improved wire replacement ...
#include <EEPROM.h>

// Custom imports
#include "controller.h"
#include "sensors.h"
#include "math.h"
#include "PID.h"
#include "dataStorage.h"

// == Hardware setup/s == 
    // Led defines
    #define LED_WHITE 2
    //#define LED_RED 4
    #define LED_ARDUINO 13
    //#define LED_STATUS 6

    //#define DISPLAY_ITTERATIONS
    #define DISPLAY_GYRO
    #define DISPLAY_ACCEL
    #define DISPLAY_MAG
    //#define ReceiverEnabled
    // Features requested
    //#define Bluetooth
    #define Magnetometer
    #define AltitudeHoldBaro
    //#define BatteryMonitorCurrent
    //#define GPS
    #define YawByMag
    //#define ACC_AUTO_CALIBRATION

    #define ESC_400HZ

	// Critical sensors on board (gyro/accel)
    //#include <mpu6050_6DOF_stick_px01.h>
    #include <mpu6050_10DOF_stick_px01.h>

    // Magnetometer
    #include <Magnetometer_HMC5883L.h>

    // Barometer
    #include <Barometer_ms5611.h>

    // GPS (ublox neo 6m)
    //#include <GPS_ublox.h>

    // Current sensor
    //#include <BatteryMonitor_current.h>

    // Kinematics used
    #include <kinematics_CMP.h>

    #ifdef ReceiverEnabled
      // Receiver
      #include <Receiver_teensy3_HW_SUMD.h>
    #endif
    

    // Frame type definition
    #include <FrameType_QuadX.h>

    // Motor / ESC setup
    #include <ESC_teensy3_HW2.h>

#ifdef LED_STATUS
	#include <Adafruit_NeoPixel.h>
#endif
	#include "LED.h"


// Global PID object definitions
PID yaw_command_pid;
PID pitch_command_pid;
PID roll_command_pid;

PID yaw_motor_pid;
PID pitch_motor_pid;
PID roll_motor_pid;

#ifdef AltitudeHoldBaro
PID altitude_hold_baro_pid;
#endif

#ifdef AltitudeHoldSonar    
PID altitude_hold_sonar_pid;
#endif

#ifdef GPS
PID yaw_position_hold_pid;
PID pitch_position_hold_pid;
PID roll_position_hold_pid;
#endif  

// Function to reset I terms inside PID objects
void reset_PID_integrals() {
    yaw_command_pid.IntegralReset();
    pitch_command_pid.IntegralReset();
    roll_command_pid.IntegralReset();
    
    yaw_motor_pid.IntegralReset();
    pitch_motor_pid.IntegralReset();
    roll_motor_pid.IntegralReset();
    
    #ifdef AltitudeHoldBaro
    altitude_hold_baro_pid.IntegralReset();
    #endif
    
    #ifdef AltitudeHoldSonar
    altitude_hold_sonar_pid.IntegralReset();
    #endif      
}
  
// Include this last as it contains objects from previous declarations
#include "GPS.h"

#ifdef ReceiverEnabled
  #include "PilotCommandProcessor.h"
#endif

#include "SerialCommunication.h"  
  
void setup() {

	LED_Init();

    // Initialize serial communication
    Serial.begin(38400); // Virtual USB Serial on teensy 3.0 is always 12 Mbit/sec (can be initialized with baud rate 0)

#ifdef GPS
    Serial3.begin(38400);
#endif
 
    // Join I2C bus as master
/**/
    Wire.begin();
    // I2C bus hardware specific settings
#if defined(__MK20DX128__)
    I2C0_F = 0x00; // 2.4 MHz (prescaler 20)
    I2C0_FLT = 4;
#endif
#if defined(__MK20DX256__)
    //I2C0_F = 0x00; // 2.4 MHz (prescaler 20)
    I2C0_F = 0x1A;	// 400kHz
    I2C0_FLT = 4;
#endif
#if defined(__AVR__)
    TWBR = 12; // 400 KHz (maximum supported frequency)
#endif
/**/
//    Wire.begin( I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_800 ); // ..., I2C_RATE_600, I2C_RATE_800, I2C_RATE_1000, I2C_RATE_1200, I2C_RATE_1500, I2C_RATE_2000, I2C_RATE_2400
    
    // Read data from EEPROM to CONFIG union
    readEEPROM();
    
#ifdef ReceiverEnabled
    vInitPitchRollExpoRate( 65, 90 );
#endif

    // Initialize PID objects with data from EEPROM
    yaw_command_pid = PID(&headingError, &YawCommandPIDSpeed, &headingSetpoint, (float*) &CONFIG.data.PID_YAW_c);
    pitch_command_pid = PID(&kinematicsAngle[YAXIS], &PitchCommandPIDSpeed, &commandPitch, (float*) &CONFIG.data.PID_PITCH_c);
    roll_command_pid = PID(&kinematicsAngle[XAXIS], &RollCommandPIDSpeed, &commandRoll, (float*) &CONFIG.data.PID_ROLL_c);
    
    yaw_motor_pid = PID(&gyro[ZAXIS], &YawMotorSpeed, &YawCommandPIDSpeed, (float*) &CONFIG.data.PID_YAW_m);
    pitch_motor_pid = PID(&gyro[YAXIS], &PitchMotorSpeed, &PitchCommandPIDSpeed, (float*) &CONFIG.data.PID_PITCH_m);
    roll_motor_pid = PID(&gyro[XAXIS], &RollMotorSpeed, &RollCommandPIDSpeed, (float*) &CONFIG.data.PID_ROLL_m);  
    
#ifdef AltitudeHoldBaro
    altitude_hold_baro_pid = PID(&baroAltitudeToHoldTarget, &AltitudeHoldMotorSpeed, &baroAltitudeRunning, (float*) &CONFIG.data.PID_BARO);
#endif
    
#ifdef AltitudeHoldSonar
    altitude_hold_sonar_pid = PID(&sonarAltitudeToHoldTarget, &AltitudeHoldMotorSpeed, &sonarAltitude, (float*) &CONFIG.data.PID_SONAR);
#endif    
    
#ifdef GPS
    // yaw_position_hold_pid = PID();
    // pitch_position_hold = PID();
    // roll_position_hold = PID();
#endif
    
    // Initialize motors/receivers/sensors
    initializeESC();    
    
#ifdef ReceiverEnabled
    initializeReceiver();
#endif
    
    sensors.initializeGyro();
    sensors.initializeAccel();
    
#ifdef Magnetometer
    sensors.initializeMag();
#endif
    
#ifdef AltitudeHoldBaro
    sensors.initializeBaro();    
#endif
    
#ifdef AltitudeHoldSonar
    initializeSonar();
#endif    
    
#ifdef GPS
    gps.initializeBaseStation();
#endif
    
    sensorPreviousTime = 0;
    previousTime = 0;

    // All is ready, start the loop
    all_ready = true;
}

void loop() {   
    // Dont start the loop until everything is ready
    if (!all_ready) return; 
 
    // Used to measure loop performance
    itterations++;
    
    // Timer
    currentTime = micros();
    
    // Read data (not faster then every 1 ms)
    if (currentTime - sensorPreviousTime >= 1000) {		// 2400kHz: 175us, 400kHz: 492us(560us), 100kHz: 1750us
        sensors.readGyroSum();
        sensors.readAccelSum();        
        
#ifdef AltitudeHoldSonar
        // Bring sonar pin down (complete TLL trigger pulse)
        readSonarFinish();
#endif    
loopTimeSensor += micros() - currentTime;
cntSensor++;

        sensorPreviousTime = currentTime;
    }    
    
    // 100 Hz task loop (10 ms)
    if (currentTime - previousTime > 10000) {		// 2400kHz: 730us, 400kHz: 900us(950us), 100kHz: 4000us
        frameCounter++;
        
        process100HzTask();
        
        // 50 Hz tak (20 ms)
        if (frameCounter % TASK_50HZ == 0) {
            process50HzTask();
        }
        
        // 10 Hz task (100 ms)
        if (frameCounter % TASK_10HZ == 0) {
            process10HzTask();
        }  
        
        // 1 Hz task (1000 ms)
        if (frameCounter % TASK_1HZ == 0) {
            process1HzTask();
        }
        
        // Reset frameCounter back to 0 after reaching 100 (1s)
        if (frameCounter >= 100) {
            frameCounter = 0;
        }

        loopTimeTask += micros() - currentTime;
        cntTask++;

        previousTime = currentTime;
    }
}

void process100HzTask() {    
    sensors.evaluateGyro();
    sensors.evaluateAccel();
    
#ifdef AltitudeHoldBaro
    // Baro is being sampled every 10ms (because measuring pressure is slow) 
    sensors.readBaroSum();
#endif    
    
#ifdef GPS
    sensors.readGPS();
#endif

    // Listens/read Serial commands on Serial1 interface (used to pass data from configurator)
    readSerial();
    
    // Update kinematics with latest data
    kinematics_update(gyro[XAXIS], gyro[YAXIS], gyro[ZAXIS], accel[XAXIS], accel[YAXIS], accel[ZAXIS]);
    
    // Update PIDs according the selected mode
    if (flightMode == ATTITUDE_MODE) {
        // Compute command PIDs (with kinematics correction)
        // yaw calculation does not depend on ACC, so it should be handled as 
        // in RATE_MODE. Exception: magnetometer based heading.
#ifdef YawByMag
        // Update heading
        //headingError = kinematicsAngle[ZAXIS] - commandYawAttitude;
        headingError = magHeadingAbsolute - commandYawAttitude;
        // magHeadingAbsolute needs strong filtering
        NORMALIZE(headingError); // +- PI
    	yaw_command_pid.Compute();
#else
        headingError = kinematicsAngle[ZAXIS] - commandYawAttitude;
        NORMALIZE(headingError); // +- PI
    	yaw_command_pid.Compute();
        //YawCommandPIDSpeed = commandYaw * 4.0;  // see the comment above
#endif
        pitch_command_pid.Compute();
        roll_command_pid.Compute();
    } else if (flightMode == RATE_MODE) {
        // Stick input, * 4.0 is the rotation speed factor
        YawCommandPIDSpeed = commandYaw * 4.0;
        PitchCommandPIDSpeed = commandPitch * 4.0;
        RollCommandPIDSpeed = commandRoll * 4.0;
    }   
    
    // Compute motor PIDs (rate-based)    
    yaw_motor_pid.Compute();
    pitch_motor_pid.Compute();
    roll_motor_pid.Compute();     
    
    #ifdef ReceiverEnabled
    // This is the place where the actual "force" gets applied
    if (armed) {
        if (TX_throttle >= CONFIG.data.minimumArmedThrottle) { // disable stabilization if throttle is very low
            updateMotorsMix(); // Frame specific motor mix
        } else {
            // Set all motors to minimum armed throttle
            for (uint8_t i = 0; i < MOTORS; i++) {
                MotorOut[i] = CONFIG.data.minimumArmedThrottle;
            }
        }
        updateMotors(); // Update ESCs
    } 
    #endif
}

void process50HzTask() {

#ifdef SUMD_IS_ACTIVE  
    ReceiverReadPacket(); // dab 2014-02-01: non interrupt controlled receiver reading  
#endif

#ifdef ReceiverEnabled
    processPilotCommands();
#endif
    
#ifdef AltitudeHoldBaro
    sensors.evaluateBaroAltitude();
#endif   

    LED_50Hz();
}

void process10HzTask() {

#ifdef ReceiverEnabled
    // Trigger RX failsafe function every 100ms
    RX_failSafe();
    if(failsafeEnabled) armed = false; //ToDo: change this !!!
#endif    
    
#ifdef AltitudeHoldSonar
    // Request sonar reading
    readSonar();
#endif    

#ifdef Magnetometer
    sensors.readMag();
    sensors.evaluateMag();
    meanMag = filterSmooth(magHeadingAbsolute, meanMag, 0.2f);
#endif
    
#ifdef BatteryMonitorCurrent
    readBatteryMonitorCurrent();
#endif   
    
    // Print itterations per 100ms
#ifdef DISPLAY_ITTERATIONS
    Serial.println("");
    Serial.println("Itterations this cycle: ");
    Serial.print(itterations);
    Serial.println("");
#endif

    LED_10Hz();
    
    // Reset Itterations
    itterations = 0;    
}

void process1HzTask() {   
	float tmpd;
	static int16_t iseconds=0;
	float accelScaleFactor = 9.81 / 8192.0;
	int16_t signal = 1;


    // Mag problem eg. 1.0 with stopped motors and 1.6 with full speed: +34 Degree ...
    // Linear, reproduceable, throttle correction?
    
#ifdef DISPLAY_MAG
    Serial.println("");
    Serial.println("Magnometer reading: ");
    Serial.print("Absolute heading: ");
    Serial.print( magHeadingAbsolute );
    Serial.print(", Mean readings: " );
    Serial.print( meanMag );
    Serial.println("");
#endif

    //Serial.print( loopTimeSensor / cntSensor );
    //Serial.print( " \t" );
    //Serial.println( loopTimeTask / cntTask );

#ifdef DISPLAY_GYRO
    Serial.println("");
    Serial.println("Gyro reading: ");
    Serial.print("X Axis: ");
    Serial.print( gyro[XAXIS] );
    Serial.print( ", Y Axis: " );
    Serial.print( gyro[YAXIS] );
    Serial.print( ", Z Axis: " );
    Serial.print( gyro[ZAXIS] );
    Serial.println("");
#endif
#ifdef DISPLAY_ACCEL
    Serial.println("");
    Serial.println("Accelerometer reading: ");
    Serial.print("X Axis: ");
    Serial.print( accel[XAXIS] );
    Serial.print( ", Y Axis: " );
    Serial.print( accel[YAXIS] );
    Serial.print( ", Z Axis: " );
    Serial.print( accel[ZAXIS] );
    Serial.println("");
#endif

    Serial.println("");
    Serial.println("---------------");
	//tmpd = (commandAux + 1.0 ) * 0.5;
	//tmpd = -1.0 * tmpd * 10.0;	// set D via AUX_2 from 0 .. -10
	//CONFIG.data.PID_PITCH_m[D] = tmpd;
	//CONFIG.data.PID_ROLL_m[D] = tmpd;

	//tmpd = (commandAux + 1.0 ) * 0.5;
	//tmpd = tmpd * 200.0;	// set D via AUX_2 from 0 .. 200
	//CONFIG.data.PID_PITCH_m[P] = tmpd;
	//CONFIG.data.PID_ROLL_m[P] = tmpd;

	//Serial.print( " \t" );
    //Serial.println( tmpd );
/*
    Serial.print( gyro[XAXIS] );
    Serial.print( " \t" );
    Serial.print( RollCommandPIDSpeed );
    Serial.print( " \t" );
    Serial.println( RollMotorSpeed );
*/

#if defined(__MK20DX256__)
//	Serial.println( "MK20DX256" );
#endif

	// Experiments for automatic calibration
	// ToDo: enable by tx? Move to mpu6050 code. Uncomment and test ZAXIS.

#ifdef ACC_AUTO_CALIBRATION
	if( TX_stick_moved )
	{
		iseconds = 0;
	}
	else if( iseconds >= 3 )
	{
	    tmpd = ( accel[XAXIS] / accelScaleFactor ) - (float)CONFIG.data.ACCEL_BIAS[XAXIS];	// remove the old bias
	    CONFIG.data.ACCEL_BIAS[XAXIS] = (int16_t)(-tmpd);				                 	// use the raw acc as new bias
	    tmpd = ( accel[YAXIS] / accelScaleFactor ) - (float)CONFIG.data.ACCEL_BIAS[YAXIS];
	    CONFIG.data.ACCEL_BIAS[YAXIS] = (int16_t)(-tmpd);
	    //tmpd = accel[ZAXIS]/accelScaleFactor - CONFIG.data.ACCEL_BIAS[ZAXIS];
	    //CONFIG.data.ACCEL_BIAS[ZAXIS] = (int16_t)(-tmpd) + 8192; // +1g

	    iseconds = 0;
	    signal = 0;
	}
	else
	{
		iseconds++;
	}
	TX_stick_moved=0;
#endif

	if( signal )LED_1Hz();

    loopTimeSensor = loopTimeTask = 0;
    cntTask = cntSensor = 0;

}
