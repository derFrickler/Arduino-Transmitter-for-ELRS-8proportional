/*
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

//#define USE_M7

/*
 =======================================================================================================
 Simple TX CONFIG OPTIONS (comment out unneeded options)
 =======================================================================================================
 */
// Define analogy input limite

#define ADC_MIN 0
#define ADC_MID 511
#define ADC_MAX 1023

#define ANALOG_CUTOFF 150 // cut off lower and upper end to avoid un-symmetric joystick range in trade off resolution
#define ANALOG_DEADBAND 20 // center deadband in ADC counts for all analog inputs

// stat war
int Aileron_OFFSET    = 0;
int Elevator_OFFSET   = 0;
int Throttle_OFFSET   = 0;
int Rudder_OFFSET     = 0;
int CH6_Offset        = 0;
int CH7_Offset        = 0;
int CH8_Offset        = 0;

// Set an entry to 1 to reverse the corresponding output channel (CH1 through CH16).
int Is_Channel_Reverse[16] = {
    1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

// IO setup
// User hardware mapping:
// A0 = Aileron CH1
// A1 = Elevator CH2
// A2 = Throttle CH3
// A3 = Rudder CH4
// A4 = CH6 analog input
// A5 = CH7 analog input
// A6 = CH8 analog input
// A7 = CH9 analog input

// D5 = CH5 toggle switch
// D6 + D7 = CH12 3-position switch pair
// D10 + D11 = CH11 3-position switch pair (AUX3)
// D8 + D9 = CH10 3-position switch pair

// A7 is reserved for analog CH9.

// D2 = LED output
// D3 = Buzzer output

const int analogInPinAileron = A0;
const int analogInPinElevator = A1;
const int analogInPinThrottle = A2;
const int analogInPinRudder = A3;

const int analogInPinCH6  = A4; // CH6
const int analogInPinCH7  = A5; // CH7
const int analogInPinCH8  = A6; // CH8
const int analogInPinCH9 = A7; // CH9

// pins used for switch inputs
const int DIGITAL_PIN_SWITCH_ARM = 5;            // CH5 toggle switch
const int DIGITAL_PIN_SWITCH_AUX2_HIGH = 6;     // CH12 3-position switch high pin
const int DIGITAL_PIN_SWITCH_AUX2_LOW = 7;      // CH12 3-position switch low pin
const int DIGITAL_PIN_SWITCH_AUX3_HIGH = 10;    // CH11 3-position switch high pin (AUX3)
const int DIGITAL_PIN_SWITCH_AUX3_LOW = 11;     // CH11 3-position switch low pin (AUX3)
const int DIGITAL_PIN_SWITCH_AUX4_HIGH = 8;     // CH10 3-position switch high pin
const int DIGITAL_PIN_SWITCH_AUX4_LOW = 9;      // CH10 3-position switch low pin
// pins used for output
const int DIGITAL_PIN_LED = 2;  

// optional WS2812 status strip on the free D4 pin
const int DIGITAL_PIN_WS2812 = 4;
const int WS2812_COUNT = 3;
const uint8_t WS2812_BRIGHTNESS = 32;

// pins that used for buzzer
// Active buzzer only plays one tone (often used on Flight controllers as a beeper)
//#define ACTIVE_BUZZER
#define PASSIVE_BUZZER
const int DIGITAL_PIN_BUZZER = 3;

// If using a passive buzzer, you can enjoy an RTTTL melody on startup (set to "" to disable)
// if using other melodies from bluejay etc. note that the first three parameters need to be in this order:
// d=,o=,b= (bluejay melodies are often in different order)
#ifdef PASSIVE_BUZZER
const char * STARTUP_MELODY = "Melody:d=32,o=4,b=570:4b,p,4e5,p,4b,p,4f#5,2p,4e5,2b5,8b5,1p,8e4,2p,8f#5";
const char * STICK_MOVE_WARNING ="tetris:d=4,o=5,b=160:e6,8b,8c6,8d6,16e6,16d6,8c6,8b,a,8a,8c6,e6,8d6,8c6,b,8b,8c6,d6,e6,c6,a,2a,8p,d6,8f6,a6,8g6,8f6,e6,8e6,8c6,e6,8d6,8c6,b,8b,8c6,d6,e6,c6,a,a";
#endif
//----- Voltage monitoring -------------------------
// Internal VCC thresholds; these do not directly measure VIN.
const unsigned int BANDGAP_REFERENCE_MILLIVOLTS = 1100;
const unsigned int VCC_WARNING_MILLIVOLTS = 4900;
const unsigned int VCC_BEEPING_MILLIVOLTS = 4700;

// Define Commond for start Up Setting
#define RC_MIN_COMMAND 600
#define RC_MAX_COMMAND 1400

// Define stick unmove alarm time
#define STICK_ALARM_TIME 300000 // 300s or 5 minutes

// ELRS 3.x (ESP8266 based TX module): with thanks to r-u-t-r-A (https://github.com/r-u-t-r-A/STM32-ELRS-Handset/tree/v4.5)
//  1 : Set Lua [Packet Rate]= 0 - 50Hz / 1 - 100Hz Full / 2- 150Hz / 3 - 250Hz / 4 - 333Hz Full / 5 - 500Hz
//  2 : Set Lua [Telem Ratio]= 0 - Std / 1 - Off / 2 - 1:128 / 3 - 1:64 / 4 - 1:32 / 5 - 1:16 / 6 - 1:8 / 7 - 1:4 / 8 - 1:2 / 9 - Race
//  3 : Set Lua [Switch Mode]=0 -> Hybrid;Wide
//  4 : Set Lua [Model Match]=0 -> Off;On
//  5 : Set Lua [TX Power]=0 Submenu
// 6 : Set Lua [Max Power]=0 - 10mW / 1 - 25mW / 2 - 50mW /3 - 100mW/4 - 250mW  // dont force to change, but change after reboot if last power was greater
// 7 : Set Lua [Dynamic]=0 - Off / 1 - Dyn / 2 - AUX9 / 3 - AUX10 / 4 - AUX11 / 5 - AUX12
// 8 : Set Lua [VTX Administrator]=0 Submenu
// 9 : Set Lua [Band]=0 -> Off;A;B;E;F;R;L
// 10:  Set Lua [Channel]=0 -> 1;2;3;4;5;6;7;8
// 11 : Set Lua [Pwr Lvl]=0 -> -;1;2;3;4;5;6;7;8
// 12 : Set Lua [Pitmode]=0 -> Off;On 
// 13 : Set Lua [Send VTx]=0 sending response for [Send VTx] chunk=0 step=2
// 14 : Set Lua [WiFi Connectivity]=0 Submenu
// 15 : Set Lua [Enable WiFi]=0 sending response for [Enable WiFi] chunk=0 step=0
// 16 : Set Lua [Enable Rx WiFi]=0 sending response for [Enable Rx WiFi] chunk=0 step=2
// //17 : Set Lua [BLE Joystick]=0 sending response for [BLE Joystick] chunk=0 step=0  // not on ESP8266??
// //    Set Lua [BLE Joystick]=1 sending response for [BLE Joystick] chunk=0 step=3
// //    Set Lua [BLE Joystick]=2 sending response for [BLE Joystick] chunk=0 step=3
// 17: Set Lua [Bind]=0 -> 

// 3 Default Settings
#define SETTING_1_PktRate 1 // 100Hz Full (16 full channels)
#define SETTING_1_Power 1   // 25mW
#define SETTING_1_Dynamic 1 // Dynamic power on

#define SETTING_2_PktRate 1 // 100Hz Full (16 full channels)
#define SETTING_2_Power 3   // 100mW
#define SETTING_2_Dynamic 0 // Dynamic power off

enum chan_order
{
    AILERON,  // CH1
    ELEVATOR, // CH2
    THROTTLE, // CH3
    RUDDER,   // CH4
    AUX1,     // CH5  ARM switch for ExpressLRS
    AUX2,     // CH6  mode / arm change
    AUX3,     // CH7  flip after crash
    AUX4,     // CH8
    AUX5,     // CH9
    AUX6,     // CH10
    AUX7,     // CH11
    AUX8,     // CH12
    AUX9,     // CH13
    AUX10,    // CH14
    AUX11,    // CH15
    AUX12     // CH16
};


//#define PPMOUTPUT
//-------------------- PPM Config-------------------------
#define CHANNEL_NUMBER 12          // set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1500 // set the default servo value
#define FRAME_LENGTH 22500         // set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300           // set the pulse length
#define onState 1                  // set polarity of the pulses: 1 is positive, 0 is negative
#define ppmPin 2                   // set PPM signal output pin on the arduino
