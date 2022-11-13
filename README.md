# teslaBMSBL

![Tesla BMS BL](misc/20190319_221311.jpg)

This project is based on https://github.com/Vigeant/teslaBMSBL

It supports native Tesla model S/X module drivers, which come with cell-level voltage monitoring, active cell balancing, redundant high-speed communication, module fault detection, and two temperature sensors. They allow any number of modules up to 63, independent of the pack configuration.

Some changes are applied in this fork to suit electric car conversion purposes:
- Implement CAN bus communication with Elcon OBC
- TODO: Implement CAN bus communication with VCU
- TODO: Implement CAN bus fault logic (code B)
- Add post-charge to the controller states (balancing)
- Remove auto-charging logic from the controller
- TODO: Regulate charger amperage based on cell voltage
- Replace DC2DC 12V battery charging cycle with on- and off-setpoints
- Add battery heating logic and valve feedback fault detection
- TODO: PCB features
	- Add circuitry and 12V relays for battery heater and valve (incl. valve feedback at A10)
	- Remove battery monitor fault circuitry and EVCC disconnect (use pins for valve and battery heating)
	- Move elements away from Teensy USB connector
	- Use SMD basic parts for easy manufacturing
	- Replace fault-pin relays with transistor switches
	- Use automotive-standard connectors

## dependencies

- TeensyView libs
	- https://github.com/sparkfun/SparkFun_TeensyView_Arduino_Library/tree/master/examples
- Snooze for lower power consumption
	- https://github.com/duff2013/Snooze
	
## Error codes on teensyView

| code | definition | 
|:----:|------------|
| A | Modules Fault Loop |
| B | CAN bus Fault |
| C | BMS Serial communication Fault |
| D | BMS Cell Over Voltage Fault |
| E | BMS Cell Under Voltage Fault |
| F | BMS Over Temperature Fault |
| G | BMS Under Temperature Fault |
| H | BMS 12V Battery Over Voltage Fault |
| I | BMS 12V Battery Under Voltage Fault |
| J | BMS Water Sensor 1 Fault |
| K | BMS Water Sensor 2 Fault |
| L | Coolant Loop Valve Fault |

## Connection to USB serial console

Serial Line: COMX (X typically = 7)
Speed: 115200

## particularities
Due to the 5s deepsleep mode in standby, it is hard to connect the serial console. To make it easier, either connect within 1 minute of a reset or place the bms in run mode and connect.
Due to the deepsleep mode, it is impossible to simply reprogram the teensy following the first sleep. To facilitate reprogramming, the board will not sleep for 1 minute following a reset.
The teensyview cannot be shut down as it is connected straight to the VDD pins.

	
