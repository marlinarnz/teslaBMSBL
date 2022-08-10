#pragma once

#include <Arduino.h>

//enable testing mode that simply cycles through all states instead of triggers
//#define STATECYCLING 1
#define TESTING_MODE 0

#define FAULT_DEBOUNCE_COUNT 3

///////////////////////////////////
// Teensy pin configuration      //
///////////////////////////////////
//side 1
//#define SERIAL1_RX          0
//#define SERIAL1_TX          1
#define OUTH_12V_BAT_CHRG     2     //Drive high to turn the DC2DC converter to charge the 12V battery       
#define CAN_TX                3
#define CAN_RX                4
#define OLED_PIN_DC           5
#define INL_SOFT_RST          6     // soft reset of the teensy
#define SERIAL3_RX            7     // from Tesla BMS
#define SERIAL3_TX            8     // to Tesla BMS
#define OUTPWM_PUMP           9     // PWM to coolant pump
#define OLED_PIN_CS           10
#define OLED_PIN_MOSI         11
#define INL_WATER_SENS1       12    // battery enclosure water sensor 1 

//side 2
#define OLED_PIN_SCK          13    //LED
#define INL_WATER_SENS2       14    // battery enclosure water sensor 2 
#define OLED_PIN_RESET        15
#define INL_BAT_PACK_FAULT    16    //Tesla Battery pack fault.
#define OUTL_VALVE_OPEN       17    //Tesla 4-way valve: switch between open system with motor (GND) and closed heating loop (+12V)
#define OUTH_BAT_HEATER       18    //Battery heater relay
#define INH_RUN               19    //RUN signal from VCU with voltage divider from 12V to 3.3V.
#define INH_CHARGING          20    //CHARGING signal from OBC with voltage divider from 12V to 3.3V.
#define INA_12V_BAT           A7    //PIN21 12v battery monitor. Analog input with 12V to 3.3V voltage divider.
#define OUTH_OBC_ON           22    //drive high to unblock the OBC from charging
#define OUTH_RUN              23    //drive high to signal no fault that prevents discharging.

// bottom side
#define INA_VALVE_FEEDBACK    A10   //Feedback from a Tesla 4-way valve with voltage divider from 12V to 3.3V.

//short P and G to reset board into program mode using push button

/*
 * State machine
 */
// Loop periods for the state machine of the controller
#define LOOP_PERIOD_ACTIVE_MS 200
#define LOOP_PERIOD_STANDBY_MS 2500
//cell balancing period once the function is called
//balance for 1 second given that the controller wakes up every second.
#define BALANCE_CELL_PERIOD_S 5

/*
 * Communication
 */
// Either communicate via CAN bus or exclusively via analogue signals from/to VCU and OBC
#define COMMUNICATE_VIA_CAN 1
//Set to the proper port for your USB connection - SerialUSB on Due (Native) or Serial for Due (Programming) or Teensy
#define SERIALCONSOLE   Serial
//CAN bus communication settings
#define CANBUS_SPEED 250000
#define CANBUS_ADDRESS 1

/*
 * Charging
 */
#define N_CHARGERS 1
#define OVER_V_SETPOINT 4.2f
#define UNDER_V_SETPOINT 3.0f
//stop charging 
#define MAX_CHARGE_V_SETPOINT 4.1f
// charge amperage
#define MAX_CHARGE_A_SETPOINT 30
// reduce charger amperage at high average cell voltages (unit: ampere per volt)
#define CHARGE_A_REDUCTION_FACTOR 100.0f

/*
 * Cell balancing
 */
//issue a warning on OLED and serial console if a cell is that close to a OV or UV fault.
#define WARN_CELL_V_OFFSET 0.1f
//start precision balancing when highest cell reaches this setpoint (taken from tom debree)
#define PRECISION_BALANCE_V_SETPOINT 3.9f
//precision balance all cells above the lowest cell by this offset (taken from tom debree)
#define PRECISION_BALANCE_CELL_V_OFFSET 0.04f
//start rough balancing when highest cell reaches this setpoint
#define ROUGH_BALANCE_V_SETPOINT 3.4f
//rough balance all cells above the lowest cell by this offset
#define ROUGH_BALANCE_CELL_V_OFFSET 0.10f

/*
 * cooling/heating system setings
 * Research on ideal temperatures: https://www.evcreate.nl/ideal-battery-temperature/
 */
#define FLOOR_DUTY_COOLANT_PUMP 0.25f // 0.0 - 1.0
#define COOLING_LOWT_SETPOINT 25.0f  //threashold at wich coolant pump gradually increases duty up to max.
#define COOLING_HIGHT_SETPOINT 35.0f //threshold at wich coolant pump is at maximum duty
// Temperatures in °C
#define OVER_T_SETPOINT 45.0f       //Tesla seam to allow reaching 45C while supercharging
#define UNDER_T_SETPOINT -10.0f
#define HEATING_T_SETPOINT 15.0f //threshold under wich battery heating starts
// The valve gives analogue feedback of its state which allows fault detection
#define VALVE_FEEDBACK_SCALING_DIVISOR 68.0f //TODO verify
#define VALVE_CLOSED_V_THRESH 2.8f
#define VALVE_OPEN_V_THRESH 9.0f
//issue a warning on OLED and serial console if T is that close to a OT or UT fault.
#define WARN_T_OFFSET 5.0f

/*
 * DC2DC and 12V battery
 */
// DC2DC 12V battery voltage setpoints to trigger charging
#define DC2DC_ON_V_SETPOINT 12.5f
#define DC2DC_OFF_V_SETPOINT 14.0f
//12V battery OV setpoint
#define BAT12V_OVER_V_SETPOINT 14.5f
//12V battery UV setpoint
#define BAT12V_UNDER_V_SETPOINT 10.0f
//12V battery ADC devisor 0-1023 -> 0-15V
//#define BAT12V_SCALING_DIVISOR 68.0f
#define BAT12V_SCALING_DIVISOR 61.78f
