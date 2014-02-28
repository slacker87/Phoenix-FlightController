/*
 * LED.h
 *
 *  Created on: 28.02.2014
 *      Author: dab
 */

#ifndef LED_H_
#define LED_H_


#define LED_STATUS_COUNT	20
#define LED_STATUS_MAX 		30


#ifdef LED_STATUS

#define RED		strip.Color(LED_STATUS_MAX, 0,              0)
#define GREEN	strip.Color(0,              LED_STATUS_MAX, 0)
#define BLUE	strip.Color(0,              0,              LED_STATUS_MAX)
#define WHITE	strip.Color(LED_STATUS_MAX, LED_STATUS_MAX, LED_STATUS_MAX)
#define YELLOW	strip.Color(LED_STATUS_MAX, LED_STATUS_MAX, 0)
#define BLACK	strip.Color(0,              0,              0)

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_STATUS_COUNT, LED_STATUS, NEO_GRB + NEO_KHZ800);

uint32_t LedStatus[5] = { RED, RED, RED, RED, RED };
#endif


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}



void LED_Init( void )
{
    // PIN settings:
#ifdef LED_ARDUINO
    pinMode(LED_ARDUINO, OUTPUT);
#endif

#ifdef LED_WHITE
    pinMode(LED_WHITE, OUTPUT);
#endif

#ifdef LED_RED
    pinMode(LED_RED, OUTPUT);
#endif

#ifdef LED_STATUS
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    colorWipe(strip.Color(LED_STATUS_MAX, 0, 0), 50); // Red
#endif
}

void LED_50Hz( void )
{
#ifdef LED_WHITE
    // Blink "aircraft beacon" LED
    if ((Beacon_LED_state == 51) || (Beacon_LED_state == 59) || (Beacon_LED_state == 67)) {
        digitalWrite(LED_WHITE, HIGH);
    } else {
        digitalWrite(LED_WHITE, LOW);
    }

    Beacon_LED_state++;

    if (Beacon_LED_state >= 100) {
        Beacon_LED_state = 0;
    }
#endif

}

void LED_10Hz( void )
{
	uint32_t status;
	uint16_t i, update=0;

#ifdef LED_ARDUINO
    // Blink integrated arduino LED
    Arduino_LED_state = !Arduino_LED_state;
    digitalWrite(LED_ARDUINO, Arduino_LED_state);

#endif

// A multi color LED Stripe take 20 x 40us = 0.8ms time to update, no interrupts (only used by sonar)
// LED-Stripe: MultiColor(5V, Status, 1Hz?)
#ifdef LED_STATUS
    // ARM Status
    if(flightMode == RATE_MODE) status = RED;
    else if (flightMode == ATTITUDE_MODE) status = GREEN;
    else status = BLACK;
    if( LedStatus[0] != status )
    {
    	LedStatus[0] = status;
    	update++;
    }
    // Gyro
    status = strip.Color( uint8_t(gyro[XAXIS] * 10 + 128), uint8_t(gyro[YAXIS] * 10 + 128), uint8_t(gyro[ZAXIS] * 10 + 128) );
    if( LedStatus[1] != status )
    {
    	LedStatus[1] = status;
    	update++;
    }
    // ACC
    status = strip.Color( uint8_t(accel[XAXIS] * 200 + 128), uint8_t(accel[YAXIS] * 200 + 128), uint8_t(accel[ZAXIS] * 200 + 128) );
    if( LedStatus[2] != status )
    {
    	LedStatus[2] = status;
    	update++;
    }
    // Battery
    status = GREEN;
    if( LedStatus[3] != status )
    {
    	LedStatus[3] = status;
    	update++;
    }
    // RX
    status = failsafeEnabled ? RED : GREEN;
    if( LedStatus[4] != status )
    {
    	LedStatus[4] = status;
    	update++;
    }


    if( update )
    {
    	for( i=0; i<5; i++ )
    	{
    		strip.setPixelColor(i,    LedStatus[i]);
    		strip.setPixelColor(i+5,  LedStatus[i]);
    		strip.setPixelColor(i+10, LedStatus[i]);
    		strip.setPixelColor(i+15, LedStatus[i]);
    	}
    	strip.show();	// this will take 700us with interrupts disabled!
    }
#endif

#ifdef LED_STATUS_TEST
    status = ((uint32_t)throttle - 1000) / 50;
   	for( i=0; i<20; i++ )
    {
    	strip.setPixelColor(i,(status>=i)?GREEN:BLUE);
    }
   	strip.show();	// this will take 700us with interrupts disabled!
#endif

// LED-Stripe: Red(12V), White(12V)
#ifdef LED_RED
    // Armed/ Dis-armed indicator
    if (armed) {
        digitalWrite(LED_RED, HIGH);
    } else {
        digitalWrite(LED_RED, LOW);
    }
#endif
#ifdef LED_WHITE
    // Armed/ Dis-armed indicator
    if (armed) {
        digitalWrite(LED_WHITE, HIGH);
    } else {
        digitalWrite(LED_WHITE, LOW);
    }
#endif

}

void LED_1Hz( void )
{
// LED-Stripe: Red(12V), White(12V)
#ifdef LED_RED
    // Armed/ Dis-armed indicator
    if (!armed) {
        digitalWrite(LED_RED, HIGH);
    } else {
        digitalWrite(LED_RED, LOW);
    }
#endif
#ifdef LED_WHITE
    // Armed/ Dis-armed indicator
    if (!armed) {
        digitalWrite(LED_WHITE, HIGH);
    } else {
        digitalWrite(LED_WHITE, LOW);
    }
#endif


}

#endif /* LED_H_ */
