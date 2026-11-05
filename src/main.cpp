/*
// Simple Arduino trasmisster
// Arduino Nano
// ELRS 2.4G TX moduel
// Custom PCB from JLCPCB
// const float codeVersion = 0.92; // Software revision
// https://github.com/kkbin505/Arduino-Transmitter-for-ELRS

 * This file is part of Simple TX
 *
 * Simple TX is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Simple TX is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "NonBlockingRtttl.h"
#include "EEPROM.h"
#include "config.h"
#include "crsf.h"
#include "led.h"
#include "tone.h"

#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!
//#define GIMBAL_CALIBRATION // if not commented out, Serial.print() is active! For debugging only!!

int Aileron_value = 0; // values read from the pot
int Elevator_value = 0;
int Throttle_value = 0;
int Rudder_value = 0;
int previous_throttle = 191;

uint32_t lastDebugPrintMillis = 0;
int loopCount = 0; // for ELRS seeting

int AUX1_Arm = 0; // switch values read from the digital pin
int AUX2_value_HIGH = 0;
int AUX2_value_LOW = 0;
int AUX2_value = 0;
int AUX3_value = 0;
int AUX4_value = 0;

uint16_t supplyVoltageMillivolts;
uint16_t crsfTelemetryVoltageMillivolts;
uint32_t lastVoltageSampleMillis = 0;

int currentPktRate = 0;
int currentPower = 0;
int currentDynamic = 0;
int currentSetting = 0;
int stickMoved = 0;
int stickInt = 0;
uint32_t stickMovedMillis = 0;

uint32_t currentMillis = 0;

uint8_t crsfPacket[CRSF_PACKET_SIZE];
uint8_t crsfCmdPacket[CRSF_CMD_PACKET_SIZE];
int16_t rcChannels[CRSF_MAX_CHANNEL];
uint32_t crsfTime = 0;

CRSF crsfClass;
Adafruit_NeoPixel ws2812Strip(WS2812_COUNT, DIGITAL_PIN_WS2812, NEO_GRB + NEO_KHZ800);

bool calStatus=false;

// -----------------------------------------------------------------------------------------------------
// Calibration

#define CALIB_MARK      0x56
#define CALIB_MARK_ADDR 0x00
#define CALIB_VAL_ADDR  CALIB_MARK_ADDR + 1

#define CALIB_CNT       5       // times of switch on/off
#define CALIB_CENT_TMO  5000    //ms
#define CALIB_TMO       20000   // ms
int     cal_reset = 0 ;

struct CalibValues {
    int aileronMin;
    int aileronMax;
    int aileronCenter;
    int elevatorMin;
    int elevatorMax;
    int elevatorCenter;
    int thrMin;
    int thrMax;
    int rudderMin;
    int rudderMax;
    int rudderCenter;
    int ch6Min;
    int ch6Max;
    int ch7Min;
    int ch7Max;
    int ch8Min;
    int ch8Max;
    int ch9Min;
    int ch9Max;
};

CalibValues calValues;
bool calibrationStarted = false;
unsigned long calibrationProcessStart;

bool calibrationPresent() {
    return EEPROM.read(CALIB_MARK_ADDR) == CALIB_MARK;
}

void calibrationSave() {
    EEPROM.update(CALIB_MARK_ADDR, (uint8_t)0xFF);
    EEPROM.put(CALIB_VAL_ADDR, calValues);
    EEPROM.update(CALIB_MARK_ADDR, CALIB_MARK);
}

void calibrationLoad() {
    EEPROM.get(CALIB_VAL_ADDR, calValues);
}

void calibrationReset() {
    calValues.aileronMin = ANALOG_CUTOFF;
    calValues.aileronMax = ADC_MAX - ANALOG_CUTOFF;
    calValues.aileronCenter = ADC_MID;
    calValues.elevatorMin = ANALOG_CUTOFF;
    calValues.elevatorMax = ADC_MAX - ANALOG_CUTOFF;
    calValues.elevatorCenter = ADC_MID;
    calValues.thrMin = ANALOG_CUTOFF;
    calValues.thrMax = ADC_MAX - ANALOG_CUTOFF;
    calValues.rudderMin = ANALOG_CUTOFF;
    calValues.rudderMax = ADC_MAX - ANALOG_CUTOFF;
    calValues.rudderCenter = ADC_MID;
    calValues.ch6Min = ADC_MIN;
    calValues.ch6Max = ADC_MAX;
    calValues.ch7Min = ADC_MIN;
    calValues.ch7Max = ADC_MAX;
    calValues.ch8Min = ADC_MIN;
    calValues.ch8Max = ADC_MAX;
    calValues.ch9Min = ADC_MIN;
    calValues.ch9Max = ADC_MAX;
}

uint8_t aux2cnt = 0;

unsigned long calibrationTimerStart;

//Flash LED and beep for (times)
void calibrationChirp(uint8_t times) {
    
    digitalWrite(DIGITAL_PIN_LED, HIGH);
    delay(1000);
    digitalWrite(DIGITAL_PIN_LED, LOW);
    delay(1000);

    for (uint8_t i = 0; i < times; i++) {
#ifdef ACTIVE_BUZZER
        digitalWrite(DIGITAL_PIN_BUZZER, HIGH);
#else
        tone(DIGITAL_PIN_BUZZER,5000,100);
#endif
        digitalWrite(DIGITAL_PIN_LED, HIGH);
        delay(100);
#ifdef ACTIVE_BUZZER
        digitalWrite(DIGITAL_PIN_BUZZER, LOW);
#endif
        digitalWrite(DIGITAL_PIN_LED, LOW);
        delay(100);
    }
    digitalWrite(DIGITAL_PIN_LED, LOW);
}

void calibrationCount(int aux1, int aux2) {
    static uint8_t preAux2 = 0;

    // Don't do anything if ARMED
    if (aux1 != 0) {
        aux2cnt = 0;
        preAux2 = 0;
        return;
    }

    else{
        // Start timer window // Arm = DISARMED
        if (aux2cnt == 0) {
            calibrationTimerStart = millis();
        }

        // Timeout
        if (calibrationTimerStart + CALIB_TMO > millis()) {
            aux2cnt = 0;
            preAux2 = 0;
            return;
        }

        // Analyze AUX2 switch, count only one side of the switch
        if (aux2 == 1 && preAux2 == 0) {
            aux2cnt++;
            preAux2 = 1;
        } else if (aux2 == 0 && preAux2 == 1) {
            preAux2 = 0;
        }
    }
}

bool calibrationRequested() {
    //return aux2cnt > CALIB_CNT;
    return calStatus;
}

bool calibrationProcess() {
    aux2cnt = 0;

    if (!calibrationStarted) {
        calibrationReset();
        calValues.aileronMin = ADC_MAX;
        calValues.aileronMax = ADC_MIN;
        calValues.elevatorMin = ADC_MAX;
        calValues.elevatorMax = ADC_MIN;
        calValues.thrMin = ADC_MAX;
        calValues.thrMax = ADC_MIN;
        calValues.rudderMin = ADC_MAX;
        calValues.rudderMax = ADC_MIN;
        calValues.ch6Min = ADC_MAX;
        calValues.ch6Max = ADC_MIN;
        calValues.ch7Min = ADC_MAX;
        calValues.ch7Max = ADC_MIN;
        calValues.ch8Min = ADC_MAX;
        calValues.ch8Max = ADC_MIN;
        calValues.ch9Min = ADC_MAX;
        calValues.ch9Max = ADC_MIN;
        calibrationProcessStart = millis();
        calibrationStarted = true;
    }

    currentMillis = millis() - calibrationProcessStart;

    if (currentMillis < CALIB_CENT_TMO) {
        // A Center
        int val = analogRead(analogInPinAileron);
        calValues.aileronCenter = val;
        
        // E Center
        val = analogRead(analogInPinElevator);
        calValues.elevatorCenter = val;

        // R Center
        val = analogRead(analogInPinRudder);
        calValues.rudderCenter = val;
    
     Serial.print("Aileron_Min:");
        Serial.print("Center Stick: AilerCenter:");
        Serial.print(calValues.aileronCenter);
        Serial.print(" ElevatorCenter:");
        Serial.print(calValues.elevatorCenter);
        Serial.print(" RudderCenter:");
        Serial.print(calValues.rudderCenter);
        Serial.println();
        
    }
    // 15 seconds for moving sticks
    else if (currentMillis < CALIB_TMO) {
    //while ((calibrationTimerStart + CALIB_TMO ) < millis()) {
        // A Min-Max
        int val = analogRead(analogInPinAileron);
        if (val < calValues.aileronMin) {
            calValues.aileronMin = val;
        } 
        if (val > calValues.aileronMax) {
            calValues.aileronMax = val;
        }
        // E Min-Max
        val = analogRead(analogInPinElevator);
        if (val < calValues.elevatorMin) {
            calValues.elevatorMin = val;
        } 
        if (val > calValues.elevatorMax) {
            calValues.elevatorMax = val;
        }

        // T Min-Max
        val = analogRead(analogInPinThrottle);
        if (val < calValues.thrMin) {
            calValues.thrMin = val;
        } 
        if (val > calValues.thrMax) {
            calValues.thrMax = val;
        }

        // R Min-Max
        val = analogRead(analogInPinRudder);
        if (val < calValues.rudderMin) {
            calValues.rudderMin = val;
        } 
        if (val > calValues.rudderMax) {
            calValues.rudderMax = val;
        }

        val = analogRead(analogInPinCH6);
        if (val < calValues.ch6Min) calValues.ch6Min = val;
        if (val > calValues.ch6Max) calValues.ch6Max = val;

        val = analogRead(analogInPinCH7);
        if (val < calValues.ch7Min) calValues.ch7Min = val;
        if (val > calValues.ch7Max) calValues.ch7Max = val;

        val = analogRead(analogInPinCH8);
        if (val < calValues.ch8Min) calValues.ch8Min = val;
        if (val > calValues.ch8Max) calValues.ch8Max = val;

        val = analogRead(analogInPinCH9);
        if (val < calValues.ch9Min) calValues.ch9Min = val;
        if (val > calValues.ch9Max) calValues.ch9Max = val;

        // Double beep and blink every 500ms
        if (millis() % 500 == 0) {
           // calibrationChirp(2);
        }
        Serial.print("Mover stick full range: Aileron_Min:");
        Serial.print(calValues.aileronMin);
        Serial.print(" Max:");
        Serial.print(calValues.aileronMax);
        Serial.print(" ElevatorMin:");
        Serial.print(calValues.elevatorMin);
        Serial.print(" Max:");
        Serial.print(calValues.elevatorMax);
        Serial.print(" RudderMin:");
        Serial.print(calValues.rudderMin);
        Serial.print(" Max:");
        Serial.print(calValues.rudderMax);
        Serial.print(" ThrottleMin:");
        Serial.print(calValues.thrMin);
        Serial.print(" Max:");
        Serial.print(calValues.thrMax);
        Serial.print(" CH6Min:");
        Serial.print(calValues.ch6Min);
        Serial.print(" Max:");
        Serial.print(calValues.ch6Max);
        Serial.print(" CH7Min:");
        Serial.print(calValues.ch7Min);
        Serial.print(" Max:");
        Serial.print(calValues.ch7Max);
        Serial.print(" CH8Min:");
        Serial.print(calValues.ch8Min);
        Serial.print(" Max:");
        Serial.print(calValues.ch8Max);
        Serial.print(" CH9Min:");
        Serial.print(calValues.ch9Min);
        Serial.print(" Max:");
        Serial.print(calValues.ch9Max);
        Serial.println();


    }
    else {
        if (calValues.aileronMax <= calValues.aileronMin) {
            calValues.aileronMin = ANALOG_CUTOFF;
            calValues.aileronMax = ADC_MAX - ANALOG_CUTOFF;
        }
        if (calValues.elevatorMax <= calValues.elevatorMin) {
            calValues.elevatorMin = ANALOG_CUTOFF;
            calValues.elevatorMax = ADC_MAX - ANALOG_CUTOFF;
        }
        if (calValues.thrMax <= calValues.thrMin) {
            calValues.thrMin = ANALOG_CUTOFF;
            calValues.thrMax = ADC_MAX - ANALOG_CUTOFF;
        }
        if (calValues.rudderMax <= calValues.rudderMin) {
            calValues.rudderMin = ANALOG_CUTOFF;
            calValues.rudderMax = ADC_MAX - ANALOG_CUTOFF;
        }
        if (calValues.ch6Max <= calValues.ch6Min) {
            calValues.ch6Min = ADC_MIN;
            calValues.ch6Max = ADC_MAX;
        }
        if (calValues.ch7Max <= calValues.ch7Min) {
            calValues.ch7Min = ADC_MIN;
            calValues.ch7Max = ADC_MAX;
        }
        if (calValues.ch8Max <= calValues.ch8Min) {
            calValues.ch8Min = ADC_MIN;
            calValues.ch8Max = ADC_MAX;
        }
        if (calValues.ch9Max <= calValues.ch9Min) {
            calValues.ch9Min = ADC_MIN;
            calValues.ch9Max = ADC_MAX;
        }
        Serial.println("Calibration Done");
        calibrationSave();
        calStatus = false;
        calibrationStarted = false;
    }
    return true;

}

void calibrationRun(int aux1, int aux2) {
    if (calibrationRequested()) {
        //calibrationReset();
        if (calibrationProcess()) {
            //calibrationSave();
            //calibrationChirp(3);    // ok
           // Serial.println("Calibration OK");
        } else {
            //calibrationReset();
           // calibrationChirp(10);   // error
            Serial.println("Calibration error!");
        }
    } else {
        calibrationCount(aux1, aux2);
    }
}

// -----------------------------------------------------------------------------------------------------

void selectSetting() {
    // startup stick commands (rate/power selection / initiate bind / turn on tx module wifi)
    // Right stick:
    // Up Left - Rate/Power setting 1 (250hz / 100mw / Dynamic)
    // Up Right - Rate/Power setting 2 (50hz / 100mw)
    // Down Left - Start TX bind (for 3.4.2 it is now possible to bind to RX easily). Power cycle after binding
    // Down Right - Start TX module wifi (for firmware update etc)

    if (rcChannels[AILERON] < RC_MIN_COMMAND && rcChannels[ELEVATOR] > RC_MAX_COMMAND) { // Elevator up + aileron left
        currentPktRate = SETTING_1_PktRate;
        currentPower = SETTING_1_Power;
        currentDynamic = SETTING_1_Dynamic;
        currentSetting = 1;
    } else if (rcChannels[AILERON] > RC_MAX_COMMAND && rcChannels[ELEVATOR] > RC_MAX_COMMAND) { // Elevator up + aileron right
        currentPktRate = SETTING_2_PktRate;
        currentPower = SETTING_2_Power;
        currentDynamic = SETTING_2_Dynamic;
        currentSetting = 2;
    } else if (rcChannels[AILERON] < RC_MIN_COMMAND && rcChannels[ELEVATOR] < RC_MIN_COMMAND) { // Elevator down + aileron left
        currentSetting = 3;  // Bind
    } else if (rcChannels[AILERON] > RC_MAX_COMMAND && rcChannels[ELEVATOR] < RC_MIN_COMMAND) { // Elevator down + aileron right
        currentSetting = 4;  // TX Wifi
    } else {
        currentSetting = 0;
    }
}

bool checkStickMove(){
    // check if stick moved, warring after 10 minutes
    if(abs(previous_throttle - rcChannels[THROTTLE]) < 30){
        stickMoved = 0;
        //Serial.println(abs(previous_throttle - rcChannels[THROTTLE]));
    }else{
        previous_throttle = rcChannels[THROTTLE];
        stickMovedMillis = millis();
        stickMoved = 1;
    }

    if (millis() - stickMovedMillis > STICK_ALARM_TIME){
       // Serial.println((millis() - stickMovedMillis));
        return true;
    }else{
        return false;
    }
}

static int16_t mapToCrsfValue(int rawValue) {
    rawValue = constrain(rawValue, ADC_MIN, ADC_MAX);
    return map(rawValue, ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
}

static int mapCalibratedAnalogValue(int rawValue, int minimum, int maximum) {
    if (maximum <= minimum) {
        return constrain(rawValue, ADC_MIN, ADC_MAX);
    }

    rawValue = constrain(rawValue, minimum, maximum);
    return map(rawValue, minimum, maximum, ADC_MIN, ADC_MAX);
}

static int mapCalibratedStickValue(int rawValue, int minimum, int center, int maximum) {
    if (maximum <= minimum) {
        return constrain(rawValue, ADC_MIN, ADC_MAX);
    }

    rawValue = constrain(rawValue, minimum, maximum);
    center = constrain(center, minimum, maximum);
    if (rawValue <= center) {
        return center <= minimum
            ? ADC_MID
            : map(rawValue, minimum, center, ADC_MIN, ADC_MID);
    }
    return maximum <= center
        ? ADC_MID
        : map(rawValue, center, maximum, ADC_MID + 1, ADC_MAX);
}

static int applyAnalogOffsetAndDeadband(int rawValue, int offset, int deadband) {
    rawValue = constrain(rawValue + offset, ADC_MIN, ADC_MAX);
    if (abs(rawValue - ADC_MID) <= deadband) {
        return ADC_MID;
    }
    return rawValue;
}

static uint16_t readSupplyVoltageMillivolts() {
    const uint8_t previousAdmux = ADMUX;
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2);

    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC)) {}
    const uint8_t lowByte = ADCL;
    const uint8_t highByte = ADCH;
    const uint16_t bandgapReading = (static_cast<uint16_t>(highByte) << 8) | lowByte;

    ADMUX = previousAdmux;
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC)) {}
    (void)ADCL;
    (void)ADCH;

    if (bandgapReading == 0) {
        return 0;
    }
    return (BANDGAP_REFERENCE_MILLIVOLTS * 1023UL) / bandgapReading;
}

static uint32_t batteryColorFromVoltage(uint16_t voltageMillivolts, uint16_t minimumVoltageMillivolts, uint16_t maximumVoltageMillivolts) {
    if (voltageMillivolts == 0) {
        return 0;
    }

    if (voltageMillivolts < 5000U) {
        return ws2812Strip.Color(255, 0, 0);
    }

    const uint16_t minVoltage = minimumVoltageMillivolts;
    const uint16_t maxVoltage = maximumVoltageMillivolts;
    const uint16_t range = (maxVoltage > minVoltage) ? (maxVoltage - minVoltage) : 1;
    const uint16_t clamped = constrain(voltageMillivolts, minVoltage, maxVoltage);
    const uint16_t normalized = (clamped - minVoltage) * 255U / range;

    const uint8_t green = 255U - normalized;
    const uint8_t red = normalized;
    return ws2812Strip.Color(red, green, 0);
}

static void updateWs2812Status() {
    ws2812Strip.clear();

    ws2812Strip.setPixelColor(0, batteryColorFromVoltage(
        supplyVoltageMillivolts,
        4700U,
        5000U
    ));

    ws2812Strip.setPixelColor(1, batteryColorFromVoltage(
        crsfTelemetryVoltageMillivolts,
        10000,
        14000
    ));

    if (loopCount > 500) {
        ws2812Strip.setPixelColor(2, ws2812Strip.Color(0, 255, 0));
    } else {
        ws2812Strip.setPixelColor(2, ws2812Strip.Color(255, 0, 0));
    }

    ws2812Strip.show();
}

void setup()
{
    // inialize rc data
    for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
        rcChannels[i] = CRSF_DIGITAL_CHANNEL_MIN;
    }

    ws2812Strip.begin();
    ws2812Strip.setBrightness(WS2812_BRIGHTNESS);
    ws2812Strip.clear();
    ws2812Strip.show();

    analogReference(DEFAULT);
    supplyVoltageMillivolts = readSupplyVoltageMillivolts();
    crsfTelemetryVoltageMillivolts = 0;
    lastVoltageSampleMillis = millis();
    pinMode(DIGITAL_PIN_SWITCH_ARM, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_SWITCH_AUX2_HIGH, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_SWITCH_AUX2_LOW, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_SWITCH_AUX3_HIGH, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_SWITCH_AUX3_LOW, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_SWITCH_AUX4_HIGH, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_SWITCH_AUX4_LOW, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_LED, OUTPUT);    // LED
    pinMode(DIGITAL_PIN_BUZZER, OUTPUT); // BUZZER
#ifdef PASSIVE_BUZZER
    digitalWrite(DIGITAL_PIN_BUZZER, LOW); // BUZZER OFF
    if (STARTUP_MELODY != nullptr && STARTUP_MELODY[0] != '\0') {
      rtttl::begin(DIGITAL_PIN_BUZZER, STARTUP_MELODY);
    }
#endif
#ifdef PPMOUTPUT
    pinMode(ppmPin, OUTPUT);
    digitalWrite(ppmPin, !onState); // set the PPM signal pin to the default state (off)
    cli();
    TCCR1A = 0; // set entire TCCR1 register to 0
    TCCR1B = 0;
    OCR1A = 100;             // compare match register, change this
    TCCR1B |= (1 << WGM12);  // turn on CTC mode
    TCCR1B |= (1 << CS11);   // 8 prescaler: 0,5 microseconds at 16mhz
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    sei();
#endif

#ifdef PASSIVE_BUZZER
    if (STARTUP_MELODY != nullptr && STARTUP_MELODY[0] != '\0') {
      while( !rtttl::done() ) {
        rtttl::play();
      }
    }
    else {
     delay(2000); // Give enough time for uploading firmware (2 seconds)     
    }
#else  
    digitalWrite(DIGITAL_PIN_BUZZER, LOW); //BUZZER ON
    delay(100); // beep
    digitalWrite(DIGITAL_PIN_BUZZER, HIGH); //BUZZER OFF
    delay(200);
    digitalWrite(DIGITAL_PIN_BUZZER, LOW); //BUZZER ON
    delay(100); // beep
    digitalWrite(DIGITAL_PIN_BUZZER, HIGH); //BUZZER OFF
    delay(200);
    digitalWrite(DIGITAL_PIN_BUZZER, LOW); //BUZZER ON
    delay(200); // beep
    digitalWrite(DIGITAL_PIN_BUZZER, HIGH); //BUZZER OFF
    delay(2000); // Give enough time for uploading firmware (2 seconds)
#endif

    #ifdef DEBUG
        Serial.begin(115200);

    #else
        crsfClass.begin();
    #endif

    #ifdef GIMBAL_CALIBRATION
        Serial.begin(115200);
        //calibrationReset();
        calStatus = true;
        Serial.println("Start Calibration"); 
    #endif

    digitalWrite(DIGITAL_PIN_LED, HIGH); // LED ON
    
    if (calibrationPresent()) {
        calibrationLoad();
    } else {
        calibrationReset();
    }
}

void loop()
{
    // melody player needs to be first in the loop in order to play correctly.
    rtttl::play();

    #ifdef GIMBAL_CALIBRATION
        if(calStatus ){
            calibrationRun(AUX1_Arm, AUX2_value);
        }
    #endif
    uint32_t currentMicros = micros();

    const uint32_t voltageSampleMillis = millis();
    if (voltageSampleMillis - lastVoltageSampleMillis >= 30000) {
        supplyVoltageMillivolts = readSupplyVoltageMillivolts();
        lastVoltageSampleMillis = voltageSampleMillis;
    }
    crsfTelemetryVoltageMillivolts = crsfClass.readBatteryVoltageMillivolts();
#ifdef DEBUG
/*
    Serial.print("VCC:");
    Serial.print(supplyVoltageMillivolts);
    Serial.print("mV ");
    Serial.print("CRSF:");
    Serial.print(crsfTelemetryVoltageMillivolts);
    Serial.print("mV ");
    */
#endif
    if (supplyVoltageMillivolts < VCC_WARNING_MILLIVOLTS && supplyVoltageMillivolts >= VCC_BEEPING_MILLIVOLTS) {
        blinkLED(DIGITAL_PIN_LED, 1000);
    }else if(supplyVoltageMillivolts < VCC_BEEPING_MILLIVOLTS && supplyVoltageMillivolts > 0){
        blinkLED(DIGITAL_PIN_LED, 250);
        playingTones(2);
    }else if(currentSetting == 3){
        blinkLED(DIGITAL_PIN_LED, 100);  // Bind (fast flash)
    }else if(currentSetting == 4){
        blinkLED(DIGITAL_PIN_LED, 1000); // Wifi (slow flash)
    }

    if (checkStickMove() == true){
        blinkLED(DIGITAL_PIN_LED, 100);
        playingTones(5);
    }
#ifdef PASSIVE_BUZZER
    else { // Stop the stick warning tone if you move the sticks
      if ( rtttl::isPlaying() ) {
        rtttl::stop();
      }
    }
#endif
    updateWs2812Status();

    /*
     * Handle analog input
     */
    // constrain to avoid overflow
    const int rawAileron = analogRead(analogInPinAileron);
    int analogVal = applyAnalogOffsetAndDeadband(rawAileron, Aileron_OFFSET, ANALOG_DEADBAND);
    Aileron_value = mapCalibratedStickValue(analogVal, calValues.aileronMin, calValues.aileronCenter, calValues.aileronMax);
    
    const int rawElevator = analogRead(analogInPinElevator);
    analogVal = applyAnalogOffsetAndDeadband(rawElevator, Elevator_OFFSET, ANALOG_DEADBAND);
    Elevator_value = mapCalibratedStickValue(analogVal, calValues.elevatorMin, calValues.elevatorCenter, calValues.elevatorMax);

    const int rawThrottle = analogRead(analogInPinThrottle);
    analogVal = applyAnalogOffsetAndDeadband(rawThrottle, Throttle_OFFSET, ANALOG_DEADBAND);
    Throttle_value = ADC_MAX - mapCalibratedAnalogValue(analogVal, calValues.thrMin, calValues.thrMax);

    const int rawRudder = analogRead(analogInPinRudder);
    analogVal = applyAnalogOffsetAndDeadband(rawRudder, Rudder_OFFSET, ANALOG_DEADBAND);
    Rudder_value = mapCalibratedStickValue(analogVal, calValues.rudderMin, calValues.rudderCenter, calValues.rudderMax);
    
    //Constrain value to avoid overflow
    Aileron_value  = constrain(Aileron_value,  ADC_MIN, ADC_MAX); 
    Elevator_value = constrain(Elevator_value, ADC_MIN, ADC_MAX); 
    Throttle_value = constrain(Throttle_value, ADC_MIN, ADC_MAX); 
    Rudder_value   = constrain(Rudder_value,   ADC_MIN, ADC_MAX); 

    rcChannels[AILERON]   = map(Aileron_value,  ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); 
    rcChannels[ELEVATOR]  = map(Elevator_value, ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    rcChannels[THROTTLE]  = map(Throttle_value, ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    rcChannels[RUDDER]    = map(Rudder_value,   ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);

    /*
     * Handel digital input
     */
    AUX1_Arm = digitalRead(DIGITAL_PIN_SWITCH_ARM);

    AUX2_value_HIGH = digitalRead(DIGITAL_PIN_SWITCH_AUX2_HIGH);
    AUX2_value_LOW = digitalRead(DIGITAL_PIN_SWITCH_AUX2_LOW);

    int AUX3_value_HIGH = digitalRead(DIGITAL_PIN_SWITCH_AUX3_HIGH);
    int AUX3_value_LOW = digitalRead(DIGITAL_PIN_SWITCH_AUX3_LOW);
    int AUX4_value_HIGH = digitalRead(DIGITAL_PIN_SWITCH_AUX4_HIGH);
    int AUX4_value_LOW = digitalRead(DIGITAL_PIN_SWITCH_AUX4_LOW);
    AUX3_value = (AUX3_value_HIGH == 0) ? 0 : (AUX3_value_LOW == 0 ? 1 : 0);
    // CH5..CH12 follow the custom Nano mapping in the README.
    rcChannels[AUX1] = (AUX1_Arm == 1) ? CRSF_DIGITAL_CHANNEL_MIN : CRSF_DIGITAL_CHANNEL_MAX;
    rcChannels[AUX2] = mapToCrsfValue(mapCalibratedAnalogValue(
        applyAnalogOffsetAndDeadband(analogRead(analogInPinCH6), CH6_Offset, ANALOG_DEADBAND),
        calValues.ch6Min, calValues.ch6Max));
    rcChannels[AUX3] = mapToCrsfValue(mapCalibratedAnalogValue(
        applyAnalogOffsetAndDeadband(analogRead(analogInPinCH7), CH7_Offset, ANALOG_DEADBAND),
        calValues.ch7Min, calValues.ch7Max));
    rcChannels[AUX4] = mapToCrsfValue(mapCalibratedAnalogValue(
        applyAnalogOffsetAndDeadband(analogRead(analogInPinCH8), CH8_Offset, ANALOG_DEADBAND),
        calValues.ch8Min, calValues.ch8Max));
    rcChannels[AUX5] = mapToCrsfValue(mapCalibratedAnalogValue(
        applyAnalogOffsetAndDeadband(analogRead(analogInPinCH9), 0, ANALOG_DEADBAND),
        calValues.ch9Min, calValues.ch9Max));
    rcChannels[AUX6] = (AUX4_value_HIGH == 0) ? CRSF_DIGITAL_CHANNEL_MAX :
                       (AUX4_value_LOW == 0) ? CRSF_DIGITAL_CHANNEL_MIN :
                       CRSF_DIGITAL_CHANNEL_MID;
    rcChannels[AUX7] = (AUX3_value_HIGH == 0) ? CRSF_DIGITAL_CHANNEL_MAX :
                       (AUX3_value_LOW == 0) ? CRSF_DIGITAL_CHANNEL_MIN :
                       CRSF_DIGITAL_CHANNEL_MID;
    rcChannels[AUX8] = (AUX2_value_HIGH == 0) ? CRSF_DIGITAL_CHANNEL_MAX :
                       (AUX2_value_LOW == 0) ? CRSF_DIGITAL_CHANNEL_MIN :
                       CRSF_DIGITAL_CHANNEL_MID;

    // Unused extended channels remain centered unless extra hardware is added.
    rcChannels[AUX9] = CRSF_DIGITAL_CHANNEL_MID;
    rcChannels[AUX10] = CRSF_DIGITAL_CHANNEL_MID;
    rcChannels[AUX11] = CRSF_DIGITAL_CHANNEL_MID;
    rcChannels[AUX12] = CRSF_DIGITAL_CHANNEL_MID;

    for (uint8_t channel = 0; channel < CRSF_MAX_CHANNEL; channel++) {
        if (Is_Channel_Reverse[channel] == 1) {
            rcChannels[channel] = CRSF_DIGITAL_CHANNEL_MIN + CRSF_DIGITAL_CHANNEL_MAX - rcChannels[channel];
        }
    }

    if(stickInt == 0){
        previous_throttle=rcChannels[THROTTLE];
        stickInt=1;
    }
    if (loopCount == 0) {
        // Check if sticks are held in specific position on startup (bind/wifi/packet rate select)
        selectSetting();
    }

    if (currentMicros > crsfTime) {
        #ifdef DEBUG
            if (millis() - lastDebugPrintMillis >= 1000UL) {
                Serial.print("IN1-8:\t");
                Serial.print(rawAileron);
                Serial.print("\t");
                Serial.print(rawElevator);
                Serial.print("\t");
                Serial.print(rawThrottle);
                Serial.print("\t");
                Serial.print(rawRudder);
                Serial.print("\t");
                Serial.print(analogRead(analogInPinCH6));
                Serial.print("\t");
                Serial.print(analogRead(analogInPinCH7));
                Serial.print("\t");
                Serial.print(analogRead(analogInPinCH8));
                Serial.print("\t");
                Serial.print(analogRead(analogInPinCH9));

                Serial.print("\tOUT1-8:\t");
                for (uint8_t channel = 0; channel < 8; channel++) {
                    Serial.print(rcChannels[channel]);
                    if (channel + 1 < 8) {
                        Serial.print("\t");
                    }
                }

                Serial.print("\tOTHER:\t");
                for (uint8_t channel = 8; channel < CRSF_MAX_CHANNEL; channel++) {
                    Serial.print(rcChannels[channel]);
                    if (channel + 1 < CRSF_MAX_CHANNEL) {
                        Serial.print("\t");
                    }
                }

                Serial.print("\t");
                Serial.print(stickMoved);
                Serial.print("\t");
                Serial.print(previous_throttle);
                Serial.println();
                lastDebugPrintMillis = millis();
            }
        #else
            if (loopCount <= 500) { // repeat 500 packets to build connection to TX module
                // Build commond packet
                crsfClass.crsfPrepareDataPacket(crsfPacket, rcChannels);
                crsfClass.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
                loopCount++;
            }

            if (loopCount > 500 && loopCount <= 505) { // repeat 5 packets to avoid bad packet, change rate setting
                // Build commond packet
                if (currentSetting == 1 || currentSetting == 2) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_PKT_RATE_COMMAND, currentPktRate);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                } else if (currentSetting == 3) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_BIND_COMMAND, ELRS_START_COMMAND);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                } else if (currentSetting == 4) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_WIFI_COMMAND, ELRS_START_COMMAND);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                }
                loopCount++;
            } else if (loopCount > 505 && loopCount <= 510) { // repeat 5 packets to avoid bad packet, change TX power level
                if (currentSetting == 1 || currentSetting == 2) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_POWER_COMMAND, currentPower);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                }
                loopCount++;
            } else if (loopCount > 510 && loopCount <= 515) { // repeat 5 packets to avoid bad packet, change TX dynamic power setting
                if (currentSetting == 1 || currentSetting == 2) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_DYNAMIC_POWER_COMMAND, currentDynamic);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                }
                loopCount++;
            } else {
                crsfClass.crsfPrepareDataPacket(crsfPacket, rcChannels);
                crsfClass.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
            }

        #endif
        crsfTime = currentMicros + CRSF_TIME_BETWEEN_FRAMES_US;
    }
}

ISR(TIMER1_COMPA_vect) { // leave this alone
    static boolean state = true;

    TCNT1 = 0;

    if (state) { // start pulse
        digitalWrite(ppmPin, onState);
        OCR1A = PULSE_LENGTH * 2;
        state = false;
    } else { // end pulse and calculate when to start the next pulse
        static byte cur_chan_numb;
        static unsigned int calc_rest;

        digitalWrite(ppmPin, !onState);
        state = true;

        if (cur_chan_numb >= CHANNEL_NUMBER) {
            cur_chan_numb = 0;
            calc_rest = calc_rest + PULSE_LENGTH; //
            OCR1A = (FRAME_LENGTH - calc_rest) * 2;
            calc_rest = 0;
        } else {
            OCR1A = (map(rcChannels[cur_chan_numb], CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX, 1000, 2000) - PULSE_LENGTH) * 2;
            calc_rest = calc_rest + map(rcChannels[cur_chan_numb], CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX, 1000, 2000);
            cur_chan_numb++;
        }
    }
}
