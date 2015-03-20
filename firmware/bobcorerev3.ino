#include "application.h"

#include "Adafruit_GPS.h"
#include "ina219spark.h"
#include <math.h>
#include "MS5803_14.h"
//#include "SD.h"

// Lights based defines
#define PWM_FREQ 1000 // in Hertz (SET YOUR FREQUENCY)
#define WHITELIGHTS 0
#define BLUELIGHTS 1
uint16_t TIM_ARR = (uint16_t)(24000000 / PWM_FREQ) - 1; // Don't change! Calc's period.

//Popwer monitor declaration
Adafruit_INA219 ina219;

// GPS based defines
#define gpsSerial Serial1
Adafruit_GPS GPS( &gpsSerial );
bool readingGPS = true;
bool haveGPSLocation = false;
char GPSString[64];

float lastLat;
float lastLong;

// Depth and temperature defines
MS_5803 sensor = MS_5803(512);

// Servo / buoyancy based defines
Servo   mainServo;
#define DIVESPEED 120
#define SURFACESPEED 60
#define FULLSTOP 90

#define STOPPED 0
#define DIVING 1
#define SURFACING 2

#define MOTORSTOPPERIOD 70
#define MOTORDOWNOFFSET 1000

bool ascendOk = false;
bool descendOk = true;
bool ascending = false;
bool descending = false;
bool underWater = false;

int diveStatus = 0;

// SD Card based defines
// SOFTWARE SPI pin configuration - modify as required
// The default pins are the same as HARDWARE SPI
const uint8_t chipSelect = A2;    // Also used for HARDWARE SPI setup
const uint8_t mosiPin = A5;
const uint8_t misoPin = A4;
const uint8_t clockPin = A3;

// change this to match your SD shield or module;
//const int chipSelect = A2;
char dataString[100];
char sdata[10];
int lpcount=10;
bool recordingtoSD = false;

// Command based defines
#define CMDSTOP 0 // incase of emergency
#define CMDDIVE 1
#define CMDSURFACE 2 // prob not needed :)

#define CMDTEMPERATURE 3
#define CMDDEPTH 4
#define CMDLOCATION 5
#define CMDLIGHTS 6

#define CMDVOLTAGE 7
#define CMDAMPS 8

// Variables for the system
float temperature = 0;
float pressure = 0;
float depth = 0;
float prediveDepth = 0;
float targetDepth = 0;

unsigned long enddiveTime = 0;
bool autoMode = false;
unsigned long startdiveTime = 0;
unsigned long motorStop = 0;

float atmosPressure = 0; // set on first read of sensor
float depthOffset = 0;
float freshWaterDensity = 1.019716;
float saltWaterDensity = 0.9945;

bool inFreshWater = true;

float voltage = 0;
float current = 0;
float startingVoltage = 0;
float warningVoltage = 5.99;

char powerString[64];

// Hold the external arguments - spark core limited to 63 chars
char externalCommand[64];

// Cloud connection variables
bool connectedtoCloud = true;

/* Definitions */
void beginDive( int timePeriod );

// What mode are we using - non-demo will switch to manual
//SYSTEM_MODE(AUTOMATIC);
SYSTEM_MODE(MANUAL);

// The main code
void setup() {

    Serial.begin(9600);
    Serial.println("started!");

    delay(5000);

    // Force the I2C to stretch the clock
    Wire.setSpeed(CLOCK_SPEED_100KHZ);
    Wire.stretchClock(true);
    Wire.begin();

    // Set up the power monitor
    initialisePower();

    // Set the pins to output and test flash
    initialiseLights();

    // Set up the Depth sensor
    initialiseDepth();

    // Set up the GPS
    initialiseGPS();

    // Set up the servo for output
    initialiseServo();

    // Set up the SD card
    //initialiseSD();

    // Spark functions

    // First we'll connect now that everything else is done
    if( !Spark.connected() ) {
      Spark.connect();
    }

    // processexternal - due to limited number of functions, this handles most operations
    Spark.function("process", processCommand );

    // GPS string
    Spark.variable("GPS", &GPSString, STRING );
    // Power string
    Spark.variable("voltage", &powerString, STRING );
}

void loop() {

    readDepth();

    readPower();

   if( readingGPS ) {
       char c = GPS.read();
       //Serial.println( c);
       // if you want to debug, this is a good time to do it
       if( parseGPS() ) {
           // Do nothing for now - that's all we need for this version
       }
   }

  if( motorStop != 0 && motorStop < millis() ) {
    motorStop = 0;
    //fullStop();
    if( diveStatus == DIVING ) {
      stopDive();
    } else {
      stopSurface();
    }
  }

    // checks if we are underwater
   if( areWeUnderwater() || diveStatus == DIVING ) {
     // Battery power check
     if( !canWeDive() ) {
       // battery is running low - get us out of here
       beginSurface();
       // flash a bit
       errorFlash( 3 );
     }

     if( enddiveTime != 0 && enddiveTime < millis() ) {
       // We are underwater and have had a dive time set
       beginSurface();
     }

   }

   if( autoMode && startdiveTime !=0 && startdiveTime < millis() ) {
     beginDive( 240 );
   }


    // If we are connected to the cloud then we need to process commands on a regular basis
    if( Spark.connected() ) {
        Spark.process();
    }

}

/*
* Control code
*
* Commands some in the format
* codenumber:parameter1/parameter2/parameter3
*
* for multiple commands in a single send parameters can be split
*
* codenumber:parameter1/parameter2:parameter1/parameter2
*
*/

/*#define CMDSTOP 0 // incase of emergency
#define CMDDIVE 1
#define CMDSURFACE 2 // prob not needed :)

#define CMDTEMPERATURE 3
#define CMDDEPTH 4
#define CMDLOCATION 5
#define CMDLIGHTS 6

#define CMDVOLTAGE 7
#define CMDAMPS 8
*/

int processCommand( String args ) {

    //Serial.println( args );

    // Get things out of a string for processing
    args.toCharArray( externalCommand, 63 );

    char* receivedCommand = strtok(externalCommand, ":");
    //char* receivedArguments = strtook( NULL, ":" );

    int command = atoi( receivedCommand );

    switch(command) {
        case 0: { // Stop and reset
                Serial.println( "processing fullstop" );
                fullStop();

                ascendOk = true;
                descendOk = true;

                autoMode = false;
                break;
            }
        case 1: { // Dive
                Serial.println( "processing dive" );
                if( canWeDive() ) {
                    char* timePeriodCommand = strtok(NULL, ":");
                    int timePeriod = atoi( timePeriodCommand );
                    beginDive( timePeriod );
                } else {
                  // Warn some how
                  errorFlash( 3 );
                }
                break;
            }
        case 2: { // Surface
                Serial.println( "processing surface" );
                // In all likelyhood this will never get called in real life as no wifi underwater
                beginSurface();
                break;
            }
        case 3: { // Temperature
                Serial.println( "processing temperature" );
                return (int)(temperature * 100);
                //fullStop();
                break;
            }
        case 4: { // Depth
                Serial.println( "processing depth" );
                return (int)(depth * 100);
                break;
            }
        case 5: {
                Serial.println( "processing gps" );

                char* GPSCommand = strtok(NULL, ":");
                int gpsc = atoi( GPSCommand );

                if( gpsc == 0 ) {
                    // Put GPS to sleep
                    if( GPS.standby() ) {
                        return 1;
                    } else {
                        return 0;
                    }
                } else if ( gpsc == 1) {
                    // Wake GPS
                    if( GPS.wakeup() ) {
                        return 1;
                    } else {
                        return 0;
                    }
                } else {
                    // Just return the GPS information
                    if( returnGPS() ) {
                        return 1;
                    } else {
                        return 0;
                    }
                }
                break;
            }
        case 6: {
                Serial.println( "processing lights" );

                char* lightCommand = strtok(NULL, ":");
                int lightArray = atoi( lightCommand );

                char* lightNumberCommand = strtok(NULL, ":");
                int lightNumber = atoi( lightNumberCommand );

                if( lightNumber == 0 ) {
                    lightsOff( lightArray );
                } else {
                    lightsOn( lightArray, lightNumber );
                }
                break;
            }
        case 7: { // voltage
                Serial.println( "processing voltage" );
                return (int)(voltage * 100);
                break;
            }
        case 8: { // current
                Serial.println( "processing current" );
                return (int)(current * 100);
                break;
            }
        case 9: { // Auto dive mode
                autoMode = true;
                if( canWeDive() ) {
                    beginDive( 240 );
                } else {
                  // Warn some how
                  errorFlash( 3 );
                }
                return 1;
                break;
            }
    }

    return 1;

}

/*
* Depth sensor code
*/
void initialiseDepth() {

    if (sensor.initializeMS_5803()) {
        Serial.println( "MS5803 CRC check OK." );
    }
    else {
        Serial.println( "MS5803 CRC check FAILED!" );
    }

}

void readDepth() {

  // Read the sensor
  sensor.readSensor();

  // Store the values
  temperature = sensor.temperature();
  pressure = sensor.pressure();

  // We may be on the default - but set it to our pressure anyway
  if( atmosPressure == 0 ) {
    atmosPressure = pressure;
  }

  // calculate depth - based on openrov calculations
  if( inFreshWater ) {
    depth = (pressure - atmosPressure) * freshWaterDensity / 100;
  } else {
    depth = (pressure - atmosPressure) * saltWaterDensity / 100;
  }

  depth = depth - depthOffset;

}

/*
* Power monitor code
*/
void initialisePower() {

  // Set up the voltage monitor
  ina219.begin();

}

void readPower() {

  float shuntvoltage = ina219.getShuntVoltage_mV();

  voltage = ina219.getBusVoltage_V();
  current = ina219.getCurrent_mA();

  voltage = voltage + (shuntvoltage / 1000);

  if( startingVoltage == 0 ) {
    startingVoltage = voltage;
  }

  //powerString
  sprintf( powerString, "{\"voltage\":%f,\"amps\":%f}", voltage, current);
}

/*
* SD Card code
*/
void initialiseSD() {

    /*
    // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
      Serial.println("Card failed, or not present");
      recordingtoSD = false;
  }
  */

}

/*
* Motors code
*/
void initialiseServo() {

    mainServo.attach(A0);  // attaches the servo on the A0 pin to the servo object
    mainServo.write( FULLSTOP );

    pinMode( D2, INPUT );
    pinMode( D3, INPUT );

    /*attachInterrupt( D3, stopDive, FALLING);*/
    attachInterrupt( D3, stopSurface, FALLING);
    attachInterrupt( D2, stopSurface, FALLING);

    ascendOk = true;
    descendOk = true;

}

void beginDive( int timePeriod = 30 ) {

    if( descendOk ) {
        // stop the servo first
        fullStop();

        allLightsOn( 1 );

        // We'll store this for later on
        prediveDepth = returnDepth();
        // Need to store the time we should start surfacing again
        enddiveTime = millis() + (timePeriod * 1000);

        motorStop = millis() + (MOTORSTOPPERIOD * 1000);

        startdiveTime = 0;

        mainServo.write( DIVESPEED );

        ascendOk = true;

        diveStatus = DIVING;

        // turn the wifi off - we don't need it underwater
        closeExternalInterfaces();

        //descending = true;
        //ascending = false;
    }

}

void beginDiveNoTimeout() {

    if( descendOk ) {
        // stop the servo first
        fullStop();

        // We'll store this for later on
        prediveDepth = returnDepth();
        // Need to store the time we should start surfacing again
        enddiveTime = 0;

        mainServo.write( DIVESPEED );

        ascendOk = true;

        diveStatus = DIVING;

        // turn the wifi off - we don't need it underwater
        closeExternalInterfaces();

        //descending = true;
        //ascending = false;
    }

}

void beginSurface() {

    if( ascendOk ) {
        // stop the servo first
        fullStop();

        // Flash the lights to show that the surface is starting
        allLightsOn( 2 );

        // Set end dive time to 0 as we are no longer diving
        enddiveTime = 0;
        motorStop = 0;
        /*motorStop = millis() + ( (MOTORSTOPPERIOD) * 1000) + MOTORDOWNOFFSET;*/
        // start the surfacing
        mainServo.write( SURFACESPEED );

        descendOk = true;

        diveStatus = SURFACING;

        Serial.println( "Surfacing");

        //ascending = true;
        //descending = false;
    }

}

void stopDive() {

    // stop the servo first
    fullStop();

    // We can't descend anymore
    descendOk = false;
    // But we can ascend
    ascendOk = true;
    // Change the status
    //descending = false;

    Serial.println( "stopping dive" );


}

void closeExternalInterfaces() {

    Serial.println( "wifi off" );

    // Turn off the wifi
    WiFi.off();
    // Turn off the GPS
    GPS.standby();

}

void openExternalInterfaces() {

  Serial.println( "wifi on");

  // Switch the wifi back on and connect to the cloud
  Spark.connect();
  // Turn the GPS back on
  GPS.wakeup();

}

bool areWeUnderwater() {
  if( returnDepth() < prediveDepth ) {
    // we are under water
    return true;
  }
}

bool canWeDive() {
  if( returnVoltage() <= warningVoltage ) {
    return false;
  } else {
    return true;
  }

}

void stopSurface() {

    // stop the servo first
    fullStop();

    // We can't descend anymore
    ascendOk = false;
    // But we can ascend
    descendOk = true;
    // Change the status
    //ascending = false;
    motorStop = 0;

    Serial.println( "stop surface" );

    diveStatus = STOPPED;

    // Flash the lights to show that the surface is stopped
    allLightsOn( 1 );

    // We need to put the wifi back on at this point, so we'll do a spark connect to handle both
    openExternalInterfaces();

    if( autoMode ) {
      startdiveTime = millis() + (60  * 1000);
    }

}

void fullStop() {
    mainServo.write( FULLSTOP );
}

/*
*   Lights code
*/
void initialiseLights() {

    pinMode(A6, OUTPUT); // sets the pin as output
    pinMode(A7, OUTPUT); // sets the pin as output

    allLightsOn( 2 );
}

void lightsOn( int lightArray, int lightTimes ) {

    if( lightTimes == 99 ) {
        // We are using this as a one time switch on of lights
        if( lightArray == WHITELIGHTS ) {
            analogWrite2(A6, 255);
        } else {
            analogWrite2(A7, 255);
        }
    } else {

        if( lightTimes <= 9 ) {
            // restrict for sensible reasons
            // switch off first
            lightsOff( lightArray );
            for( int lightCount = 1; lightCount <= lightTimes; lightCount++ ) {
                if( lightArray == WHITELIGHTS ) {
                    analogWrite2(A6, 255);
                } else {
                    analogWrite2(A7, 255);
                }
                delay( 500 );
                lightsOff( lightArray );
                delay( 500 );
            }
        }

    }



}

void allLightsOn( int lightTimes ) {

    if( lightTimes == 99 ) {
        // We are using this as a one time switch on of lights
        analogWrite2(A6, 255);
        analogWrite2(A7, 255);
    } else {

        if( lightTimes <= 9 ) {
            // restrict for sensible reasons
            // switch off first
            allLightsOff();
            for( int lightCount = 1; lightCount <= lightTimes; lightCount++ ) {
                analogWrite2(A6, 255);
                analogWrite2(A7, 255);
                delay( 500 );
                allLightsOff();
                delay( 500 );
            }
        }

    }
}

void lightsOff( int lightArray ) {

    if( lightArray == WHITELIGHTS ) {
        analogWrite2(A6, 0);
    } else {
        analogWrite2(A7, 0);
    }
}

void allLightsOff() {
    analogWrite2(A6, 0);
    analogWrite2(A7, 0);
}

void errorFlash( int errorCode ) {
  allLightsOn( errorCode );
}

// Functions that read from the various sensors attached to the system

float returnDepth() {

  if( depth == 0 ) {
    // we need to grab it as for some reason it hasn't been set
    readDepth();
  }

  return depth;

}

float returnTemperature() {

  if( temperature == 0 ) {
    // we need to grab it as for some reason it hasn't been set - same function that sets the Depth
    readDepth();
  }

  return temperature;

}

float returnVoltage() {

  if( voltage == 0 ) {
    // we need to set it as for some reason it hasn't been set
    readPower();
  }

  return voltage;

}

float returnCurrent() {

  if( current == 0 ) {
    // we need to set it as for some reason it hasn't been set - same function as Voltage
    readPower();
  }

  return current;

}

bool returnGPS() {

    if( haveGPSLocation ) {

        sprintf( GPSString, "{\"lat\":%f,\"lng\":%f}", GPS.latitudeDegrees, GPS.longitudeDegrees);

      //GPSString
        Serial.print( "Lat:" );
        Serial.print( GPS.latitudeDegrees, 4 );
        Serial.println( GPS.lat );

        Serial.print( "Long:" );
        Serial.print( GPS.longitudeDegrees, 4 );
        Serial.println( GPS.lon );

        return true;
    } else {
        Serial.println( "No location");
        sprintf( GPSString, "{\"lat\":%f,\"lng\":%f}", 0, 0);

        return false;
    }

}

/*
* GPS code
*/
void initialiseGPS() {

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);
    gpsSerial.begin(9600);

    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz

    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_NOANTENNA);

}

bool parseGPS() {

    // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   {
        // this also sets the newNMEAreceived() flag to false
        return false;  // we can fail to parse a sentence in which case we should just wait for another
    } else {
        lastLat = GPS.latitude;
        lastLong = GPS.longitude;
        haveGPSLocation = true;
        return true;
    }

  }

  return false;

}

// User defined analogWrite() to gain control of PWM initialization
void analogWrite2(uint16_t pin, uint8_t value) {
  TIM_OCInitTypeDef TIM_OCInitStructure;

  if (pin >= TOTAL_PINS || PIN_MAP[pin].timer_peripheral == NULL) {
    return;
  }
  // SPI safety check
  if (SPI.isEnabled() == true && (pin == SCK || pin == MOSI || pin == MISO)) {
    return;
  }
  // I2C safety check
  if (Wire.isEnabled() == true && (pin == SCL || pin == SDA)) {
    return;
  }
  // Serial1 safety check
  if (Serial1.isEnabled() == true && (pin == RX || pin == TX)) {
    return;
  }
  if (PIN_MAP[pin].pin_mode != OUTPUT && PIN_MAP[pin].pin_mode != AF_OUTPUT_PUSHPULL) {
    return;
  }
  // Don't re-init PWM and cause a glitch if already setup, just update duty cycle and return.
  if (PIN_MAP[pin].pin_mode == AF_OUTPUT_PUSHPULL) {
    TIM_OCInitStructure.TIM_Pulse = (uint16_t)(value * (TIM_ARR + 1) / 255);
    if (PIN_MAP[pin].timer_ch == TIM_Channel_1) {
      PIN_MAP[pin].timer_peripheral-> CCR1 = TIM_OCInitStructure.TIM_Pulse;
    } else if (PIN_MAP[pin].timer_ch == TIM_Channel_2) {
      PIN_MAP[pin].timer_peripheral-> CCR2 = TIM_OCInitStructure.TIM_Pulse;
    } else if (PIN_MAP[pin].timer_ch == TIM_Channel_3) {
      PIN_MAP[pin].timer_peripheral-> CCR3 = TIM_OCInitStructure.TIM_Pulse;
    } else if (PIN_MAP[pin].timer_ch == TIM_Channel_4) {
      PIN_MAP[pin].timer_peripheral-> CCR4 = TIM_OCInitStructure.TIM_Pulse;
    }
    return;
  }

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  //PWM Frequency : PWM_FREQ (Hz)
  uint16_t TIM_Prescaler = (uint16_t)(SystemCoreClock / 24000000) - 1; //TIM Counter clock = 24MHz

  // TIM Channel Duty Cycle(%) = (TIM_CCR / TIM_ARR + 1) * 100
  uint16_t TIM_CCR = (uint16_t)(value * (TIM_ARR + 1) / 255);

  // AFIO clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  pinMode(pin, AF_OUTPUT_PUSHPULL);

  // TIM clock enable
  if (PIN_MAP[pin].timer_peripheral == TIM2)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  else if (PIN_MAP[pin].timer_peripheral == TIM3)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  else if (PIN_MAP[pin].timer_peripheral == TIM4)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  // Time base configuration
  TIM_TimeBaseStructure.TIM_Period = TIM_ARR;
  TIM_TimeBaseStructure.TIM_Prescaler = TIM_Prescaler;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(PIN_MAP[pin].timer_peripheral, & TIM_TimeBaseStructure);

  // PWM1 Mode configuration
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_Pulse = TIM_CCR;

  if (PIN_MAP[pin].timer_ch == TIM_Channel_1) {
    // PWM1 Mode configuration: Channel1
    TIM_OC1Init(PIN_MAP[pin].timer_peripheral, & TIM_OCInitStructure);
    TIM_OC1PreloadConfig(PIN_MAP[pin].timer_peripheral, TIM_OCPreload_Enable);
  } else if (PIN_MAP[pin].timer_ch == TIM_Channel_2) {
    // PWM1 Mode configuration: Channel2
    TIM_OC2Init(PIN_MAP[pin].timer_peripheral, & TIM_OCInitStructure);
    TIM_OC2PreloadConfig(PIN_MAP[pin].timer_peripheral, TIM_OCPreload_Enable);
  } else if (PIN_MAP[pin].timer_ch == TIM_Channel_3) {
    // PWM1 Mode configuration: Channel3
    TIM_OC3Init(PIN_MAP[pin].timer_peripheral, & TIM_OCInitStructure);
    TIM_OC3PreloadConfig(PIN_MAP[pin].timer_peripheral, TIM_OCPreload_Enable);
  } else if (PIN_MAP[pin].timer_ch == TIM_Channel_4) {
    // PWM1 Mode configuration: Channel4
    TIM_OC4Init(PIN_MAP[pin].timer_peripheral, & TIM_OCInitStructure);
    TIM_OC4PreloadConfig(PIN_MAP[pin].timer_peripheral, TIM_OCPreload_Enable);
  }

  TIM_ARRPreloadConfig(PIN_MAP[pin].timer_peripheral, ENABLE);

  // TIM enable counter
  TIM_Cmd(PIN_MAP[pin].timer_peripheral, ENABLE);
}
