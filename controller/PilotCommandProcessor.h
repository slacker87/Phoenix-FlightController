/*  Pilot / Receiver command handling routine

    Featuring dynamic channel assignment and dynamic auxiliary funtion assignment
    with "side" support for automatic receiver failsafe routines.
    
    Dynamic channel assignment was initially requested by Politt @aeroquad hangout.
    
    Dynamic auxiliary funtion assignment was inspired by very similar feature originally found
    in multiwii flight control software.
*/

#define CMD_TO_RAD	0.001570796f	// +-500 to +-PI/4
#define CMD_CENTER	1500
#define CMD_MAX		2000
#define CMD_MIN		1000
#define CMD_HIGH	1750
#define CMD_LOW		1250
#define THROTTLELIMIT	1950


int16_t TX_roll, TX_pitch, TX_throttle, TX_yaw, TX_AUX1, TX_AUX2, TX_AUX3, TX_AUX4;
int16_t TX_AUX5, TX_AUX6, TX_AUX7, TX_AUX8, TX_AUX9, TX_AUX10, TX_AUX11, TX_AUX12;
int16_t TX_last_roll, TX_last_pitch, TX_stick_moved;
uint64_t AUX_chan_mask;
static int16_t lookupPitchRollRC[6];// lookup table for expo & RC rate PITCH+ROLL

bool throttlePanic = false;
typedef enum { PC_NONE, PC_ARM, PC_DISARM, PC_INIT_GYRO, PC_TRIM_ACC, PC_INIT_ACC } PilotCommand_t;

void vGetRxData( void )
{
    // read data into variables
    cli(); // disable interrupts
    
    // Channel assignment variables are loaded from eeprom
    // allowing user to "dynamically" (via configurator) change the rx channel assignment
    // potentionally allowing to "wire" channels from RX to FC completely wrong 
    // (and then fixing them manually in Channel Assigner)
    TX_roll     = RX[CONFIG.data.CHANNEL_ASSIGNMENT[0]];
    TX_pitch    = RX[CONFIG.data.CHANNEL_ASSIGNMENT[1]];
    TX_throttle = RX[CONFIG.data.CHANNEL_ASSIGNMENT[2]];
    TX_yaw      = RX[CONFIG.data.CHANNEL_ASSIGNMENT[3]];
    TX_AUX1     = RX[CONFIG.data.CHANNEL_ASSIGNMENT[4]];
    TX_AUX2     = RX[CONFIG.data.CHANNEL_ASSIGNMENT[5]];
    TX_AUX3     = RX[CONFIG.data.CHANNEL_ASSIGNMENT[6]];
    TX_AUX4     = RX[CONFIG.data.CHANNEL_ASSIGNMENT[7]];
    TX_AUX5     = RX[CONFIG.data.CHANNEL_ASSIGNMENT[8]];
    TX_AUX6     = RX[CONFIG.data.CHANNEL_ASSIGNMENT[9]];
    TX_AUX7     = RX[CONFIG.data.CHANNEL_ASSIGNMENT[10]];
    TX_AUX8     = RX[CONFIG.data.CHANNEL_ASSIGNMENT[11]];
    TX_AUX9     = RX[CONFIG.data.CHANNEL_ASSIGNMENT[12]];
    TX_AUX10    = RX[CONFIG.data.CHANNEL_ASSIGNMENT[13]];
    TX_AUX11    = RX[CONFIG.data.CHANNEL_ASSIGNMENT[14]];
    TX_AUX12    = RX[CONFIG.data.CHANNEL_ASSIGNMENT[15]];
    
    sei(); // enable interrupts
}

void vSetAuxMask( void )
{
    // Set the mask
    AUX_chan_mask = 0x00; // reset the mask
    
    if (TX_AUX1 < CMD_LOW) { // LOW
        AUX_chan_mask |= 1 << 0;
    } else if (TX_AUX1 < CMD_HIGH) { // MID
        AUX_chan_mask |= 1 << 1;
    } else { //HIGH
        AUX_chan_mask |= 1 << 2;
    }

    if (TX_AUX2 < CMD_LOW) {
        AUX_chan_mask |= 1 << 3;
    } else if (TX_AUX2 < CMD_HIGH) {
        AUX_chan_mask |= 1 << 4;
    } else {
        AUX_chan_mask |= 1 << 5;
    }

    if (TX_AUX3 < CMD_LOW) {
        AUX_chan_mask |= 1 << 6;
    } else if (TX_AUX3 < CMD_HIGH) {
        AUX_chan_mask |= 1 << 7;
    } else {
        AUX_chan_mask |= 1 << 8;
    }
    
    if (TX_AUX4 < CMD_LOW) {
        AUX_chan_mask |= 1 << 9;
    } else if (TX_AUX4 < CMD_HIGH) {
        AUX_chan_mask |= 1 << 10;
    } else {
        AUX_chan_mask |= 1 << 11;
    }
    
    if (TX_AUX5 < CMD_LOW) {
        AUX_chan_mask |= 1 << 12;
    } else if (TX_AUX5 < CMD_HIGH) {
        AUX_chan_mask |= 1 << 13;
    } else {
        AUX_chan_mask |= 1 << 14;
    }
    
    if (TX_AUX6 < CMD_LOW) {
        AUX_chan_mask |= 1 << 15;
    } else if (TX_AUX6 < CMD_HIGH) {
        AUX_chan_mask |= 1 << 16;
    } else {
        AUX_chan_mask |= 1 << 17;
    }
 
    if (TX_AUX7 < CMD_LOW) {
        AUX_chan_mask |= 1 << 18;
    } else if (TX_AUX7 < CMD_HIGH) {
        AUX_chan_mask |= 1 << 19;
    } else {
        AUX_chan_mask |= 1 << 20;
    }
    
    if (TX_AUX8 < CMD_LOW) {
        AUX_chan_mask |= 1 << 21;
    } else if (TX_AUX8 < CMD_HIGH) {
        AUX_chan_mask |= 1 << 22;
    } else {
        AUX_chan_mask |= 1 << 23;
    }
    
    if (TX_AUX9 < CMD_LOW) {
        AUX_chan_mask |= 1 << 24;
    } else if (TX_AUX9 < CMD_HIGH) {
        AUX_chan_mask |= 1 << 25;
    } else {
        AUX_chan_mask |= 1 << 26;
    }
    
    if (TX_AUX10 < CMD_LOW) {
        AUX_chan_mask |= 1 << 27;
    } else if (TX_AUX10 < CMD_HIGH) {
        AUX_chan_mask |= 1 << 28;
    } else {
        AUX_chan_mask |= 1 << 29;
    }
    
    if (TX_AUX11 < CMD_LOW) {
        AUX_chan_mask |= 1ULL << 30; // Starting to use ULL because we will go over the 32bit barier soon
    } else if (TX_AUX11 < CMD_HIGH) {
        AUX_chan_mask |= 1ULL << 31;
    } else {
        AUX_chan_mask |= 1ULL << 32;
    }
    
    if (TX_AUX12 < CMD_LOW) {
        AUX_chan_mask |= 1ULL << 33;
    } else if (TX_AUX12 < CMD_HIGH) {
        AUX_chan_mask |= 1ULL << 34;
    } else {
        AUX_chan_mask |= 1ULL << 35;
    }
}

PilotCommand_t getPilotCommand( void )
{
	PilotCommand_t pc = PC_NONE;
	if( armed == true )
	{
		if ((TX_throttle < 1100 && TX_yaw < CMD_LOW) || (TX_throttle <= CONFIG.data.minimumArmedThrottle && failsafeEnabled))
		{
			pc = PC_DISARM;
		}
	}
	else
	{
	    if (TX_throttle < 1100 && TX_yaw > 1850)
	    {
	    	pc = PC_ARM;
	    }
	    else if (TX_throttle > CMD_HIGH && TX_yaw > CMD_HIGH)
	    {
	    	pc = PC_TRIM_ACC;
	    }
	    else if (TX_throttle < 1100 && TX_yaw < CMD_LOW && TX_pitch < CMD_LOW )
	    {
	    	pc = PC_INIT_GYRO;
	    }
	    else if (TX_throttle > CMD_HIGH && TX_yaw < CMD_LOW && TX_pitch < CMD_LOW )
	    {
	    	pc = PC_INIT_ACC;
	    }

	}
	return pc;
}

void vArm( void )
{
    // Arming-Disarming sequence
    reset_PID_integrals(); // Reset all integrals inside PID controllers to 0
    commandYawAttitude = kinematicsAngle[ZAXIS];
    commandYaw = 0.0;
    armed = true;
}

void vDisarm( void )
{
    // Arming-Disarming sequence
    // Reset all motors to 0 throttle / power
    for (uint8_t i = 0; i < MOTORS; i++)
    {
    	MotorOut[i] = CMD_MIN;
    }
    updateMotors(); // Update ESCs
    armed = false;
}

void vInitGyro( void )
{
	LED_ON();
	sensors.initializeGyro();
	LED_OFF();
}

void vCalibAcc( void )
{
	LED_ON();
	sensors.calibrateAccel();
	LED_OFF();
}

void vTrimAcc( void )
{
    // TX accelerometer trimming
	// We jest enter trimming mode
	static bool changed = false;

	LED_ON();
	// We will "lock" the user in here until trimming is done
	while (TX_throttle > CMD_HIGH) {

#ifdef SUMD_IS_ACTIVE
		ReceiverReadPacket(); // dab 2014-02-01: non interrupt controlled receiver reading
#endif
		// Fetch latest values
		TX_roll     = RX[CONFIG.data.CHANNEL_ASSIGNMENT[0]];
		TX_pitch    = RX[CONFIG.data.CHANNEL_ASSIGNMENT[1]];
		TX_throttle = RX[CONFIG.data.CHANNEL_ASSIGNMENT[2]];
		TX_yaw      = RX[CONFIG.data.CHANNEL_ASSIGNMENT[3]];

		// Flip the flag if anything will be changed
		if (TX_roll > CMD_HIGH || TX_roll < CMD_LOW || TX_pitch > CMD_HIGH || TX_pitch < CMD_LOW) {
			changed = true;
		}

		// Check the sticks
		if (TX_roll > CMD_HIGH) {
			CONFIG.data.ACCEL_BIAS[YAXIS] += 5;
			LED_OFF();
		} else if (TX_roll < CMD_LOW) {
			CONFIG.data.ACCEL_BIAS[YAXIS] -= 5;
			LED_OFF();
		}

		if (TX_pitch > CMD_HIGH) {
			CONFIG.data.ACCEL_BIAS[XAXIS] -= 5;
			LED_OFF();
		} else if (TX_pitch < CMD_LOW) {
			CONFIG.data.ACCEL_BIAS[XAXIS] += 5;
			LED_OFF();
		}

		// Save trimmed calibration values to EEPROM (if there were any changes in the values)
		//if (TX_yaw > CMD_HIGH && changed) { // Rudder = RIGHT
		if (TX_yaw < CMD_LOW && changed) { // Rudder = LEFT
			writeEEPROM();

			// reset flag
			changed = false;
		}

		// Blink LED to indicate activity
		delay(200); // 0.5s loop delay to allow precise trimming
		LED_ON();
		delay(400);
    }
}

void vRateAttitudeMode( void )
{
    // Rate-Attitude mode
    if (CONFIG.data.CHANNEL_FUNCTIONS[0] & AUX_chan_mask || failsafeEnabled) {
        if (flightMode == RATE_MODE) {
            // We just switched from rate to attitude mode
            // That means YAW correction should be applied to avoid YAW angle "jump"
            commandYawAttitude = kinematicsAngle[ZAXIS];
            commandYaw = 0.0;
        }
        
        flightMode = ATTITUDE_MODE;
    } else {
        if (flightMode == ATTITUDE_MODE) {
            // We just switched from attitude to rate mode
            // That means commandYaw reset
            commandYawAttitude = kinematicsAngle[ZAXIS];
            commandYaw = 0.0;
        }
        
        flightMode = RATE_MODE;
    }
}

void vAltHold( void )
{
#if defined(AltitudeHoldBaro) || defined(AltitudeHoldSonar)
    // Altitude hold ON/OFF
    if ((CONFIG.data.CHANNEL_FUNCTIONS[1] & AUX_chan_mask) == false) {
        // throttle controlled by stick
        altitudeHoldBaro = false;
        altitudeHoldSonar = false;
        
        // reset throttle panic flag
        throttlePanic = false;
    }
#endif

#ifdef AltitudeHoldBaro
    else if ((CONFIG.data.CHANNEL_FUNCTIONS[1] & AUX_chan_mask) && throttlePanic == false) {
        // throttle controlled by baro
        if (altitudeHoldBaro == false) { // We just switched on the altitudeHoldBaro
            // save the current altitude and throttle
            baroAltitudeToHoldTarget = baroAltitudeRunning;
            baroAltitudeHoldThrottle = TX_throttle;
        }
        
        altitudeHoldSonar = false;
        altitudeHoldBaro = true;
        
        // Trigger throttle panic if throttle is higher or lower then 100 compared
        // to initial altitude hold throttle.
        if (abs(TX_throttle - baroAltitudeHoldThrottle) > 100) {
            // Pilot will be forced to re-flip the altitude hold switch to reset the throttlePanic flag.
            throttlePanic = true;
            altitudeHoldBaro = false;
        }
    }
#endif
    
#ifdef AltitudeHoldSonar
    else if ((CONFIG.data.CHANNEL_FUNCTIONS[2] & AUX_chan_mask) && throttlePanic == false) {
        // throttle controlled by sonar
        if (altitudeHoldSonar == false) { // We just switched on the altitudeHoldSonar
            sonarAltitudeToHoldTarget = sonarAltitude;
            sonarAltitudeHoldThrottle = TX_throttle;
        }
        
        altitudeHoldBaro = false;
        altitudeHoldSonar = true;
        
        // Trigger throttle panic if throttle is higher or lower then 100 compared
        // to initial altitude hold throttle.
        if (abs(TX_throttle - sonarAltitudeHoldThrottle) > 100) {
            // Pilot will be forced to re-flip the altitude hold switch to reset the throttlePanic flag.
            throttlePanic = true;
            altitudeHoldSonar = false;
        }        
    }
#endif
}

void vGpsHold( void )
{
#ifdef GPS
    if (CONFIG.data.CHANNEL_FUNCTIONS[3] & AUX_chan_mask) {
        // Position hold enabled
        if (positionHoldGPS == false) { // We just switched on the position hold
            // current heading (from magnetometer should be saved here)
            // current GPS pos should be saved here
        }
        
        positionHoldGPS = true;
    } else {
        // Position hold disabled
        if (positionHoldGPS == true) { // We just switched off the position hold
        }
        
        positionHoldGPS = false;
    }
#endif
}



float fThottleCorrection( float t, float a, float b )
{

	float cosa = 1.0 -( a * a * 0.4 );	// replace 0.5 with 0.4 to avoid over compensation
	float cosb = 1.0 -( b * b * 0.4 );
	cosa *= cosb;
	if( cosa < 0.6 ) cosa = 0.6;
	t -= 1000;
    t /= cosa;
    t += 1000;
    if( t > THROTTLELIMIT ) t = THROTTLELIMIT;	//
    return t;
}

void vInitPitchRollExpoRate( uint8_t expo, uint8_t rate )
{
	// defaults for rate = 90 and expo = 65
	  for( int i=0;i<6;i++) {
	    lookupPitchRollRC[i] = (2500+expo*(i*i-25))*i*(int32_t)rate/2500;
	  }

}

int16_t ui16PitchRollExpo( int16_t value )	// like multiwii
{
	int16_t tmp, tmp2, result;
	tmp = value;
	if( tmp < 0 ) tmp = -tmp;
	tmp2 = tmp / 100;
	if( tmp2 > 5 ) tmp2 = 5;
	result = lookupPitchRollRC[tmp2] + (tmp-tmp2*100) * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2]) / 100;
	if( value < 0 ) result = -result;
	return result;
}


void processPilotCommands( void )
{
    vGetRxData();
    vSetAuxMask();

    switch( getPilotCommand() ){
    case PC_ARM:
    	vArm();
    	break;
    case PC_DISARM:
    	vDisarm();
    	break;
    case PC_INIT_GYRO:
    	vInitGyro(); // blocking
    	//Serial.println( "vInitGyro()" );
    	break;
    case PC_TRIM_ACC:
    	vTrimAcc();	// blocking
    	//Serial.println( "vTrimAcc()" );
    	break;
    case PC_INIT_ACC:
    	vCalibAcc(); // blocking
    	//Serial.println( "vCalibAcc()" );
    	break;

    case PC_NONE:
    default:
    	break;
    }

    vRateAttitudeMode();
    vAltHold();
    vGpsHold();

    // calculate the command values:

    // Ignore TX_yaw while throttle is below minimum armed throttle
    if (TX_throttle < CONFIG.data.minimumArmedThrottle) {
        TX_yaw = CMD_CENTER;
        
        // Keep command yaw and attitude yaw error around ~0
        commandYawAttitude = kinematicsAngle[ZAXIS];
        commandYaw = 0.0;
    }    
    
    // Channel center = 1500
    TX_roll = TX_roll - CMD_CENTER;
    TX_pitch = -(TX_pitch - CMD_CENTER);
    TX_yaw = TX_yaw - CMD_CENTER;
    
    // Reverse YAW
    TX_yaw = -TX_yaw;
    
    // YAW deadband (tiny 5us deadband to cancel any noise/offset from the TX)
    // PWM2RAD = 0.002
    // ATTITUDE_SCALING = 0.75 * PWM2RAD = 0.0015 // 0.75 is just an arbitrary scale factor to limit the range to about ~43 degrees (0.75 radians)
    if( abs(TX_yaw) < 6 ) // If yaw signal is bigger then 5 (5us) allow commandYaw to change
    {
    	TX_yaw = 0;
    }

    // YAW angle build up over time
    commandYawAttitude += ( float(TX_yaw) * CMD_TO_RAD) / 8; // division by 8 is used to slow down YAW build up
    NORMALIZE(commandYawAttitude); // +- PI

    // expo for roll and pitch
    TX_roll = ui16PitchRollExpo( TX_roll );
    TX_pitch = ui16PitchRollExpo( TX_pitch );

    // raw stick input
    commandYaw   = float(TX_yaw)   * CMD_TO_RAD;
    commandRoll  = float(TX_roll)  * CMD_TO_RAD;
    commandPitch = float(TX_pitch) * CMD_TO_RAD;
    commandAux   = float(TX_AUX2 - CMD_CENTER) / 500.0;
    
    // detect movement in stabilization control (roll/pitch)
    int16_t TX_diff = TX_last_roll - TX_roll;
    if( (TX_diff < -2) || (TX_diff > 2) ) TX_stick_moved=1;
    TX_diff = TX_last_pitch - TX_pitch;
    if( (TX_diff < -2) || (TX_diff > 2) ) TX_stick_moved=1;
    if( TX_throttle < 1400 ) TX_stick_moved=1;
    if( armed == false ) TX_stick_moved=1;
    // if failsafe TX_stick_moved=1;
    TX_last_roll = TX_roll;
    TX_last_pitch = TX_pitch;

    // Compute throttle according to altitude switch (pilot input/baro/sonar)
    if (altitudeHoldBaro == false && altitudeHoldSonar == false) {
        throttle = TX_throttle;
    }
    
#ifdef AltitudeHoldBaro
    else if (altitudeHoldBaro == true) {
        altitude_hold_baro_pid.Compute();
        throttle = baroAltitudeHoldThrottle - constrain(AltitudeHoldMotorSpeed, -200.0, 200.0);
    }
#endif
    
#ifdef AltitudeHoldSonar
    else if (altitudeHoldSonar == true) {
        altitude_hold_sonar_pid.Compute();
        throttle = sonarAltitudeHoldThrottle - constrain(AltitudeHoldMotorSpeed, -200.0, 200.0);
    }
#endif
    
#ifdef GPS
    if (positionHoldGPS == true) {
        // compute gps pids
    }
#endif

    // try to correct the height loss for angles different from 0, but the idea works only
    // for low speed activity. With higher speed the height may increase rapidly.
    //throttle = fThottleCorrection( throttle, kinematicsAngle[XAXIS], kinematicsAngle[YAXIS] );
}    
