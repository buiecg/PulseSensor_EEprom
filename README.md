# PulseSensor_EEprom
Based on WorldFamousElectronics/PulseSensorPlayground project I did a BPM data logger using EEprom memory
What is new: 
Please add a switch from digital pin 3 to ground (DIP switch is good, NOT push button)
When the switch, grounded the pin 3, EEPROM datalogger is in function and do next events: 
At power on or reset, start sending EEPROM values, from Arduino to computer (serial) and using PLX_DAQ v2.11, can import in a excel file as Current number / BPM columns. EEPROM is read till MARKER = 255, is found in EEPROM. Marker help to read just last recorded values, and prevent reading previous, recorded data.
After EEPROM position to marker are transfered, arduino processor go to sleep.
And remain there till is actionate the switch and PIN 3 go pullup or HIGH
From here, programm acting as WorldFamousElectronics/PulseSensorPlayground - PulseSensor_BPM example or PulseSensor_Speaker
except pin 13 (with onboard LED) is changed and do not light, I change the function, of this pin 13, for EEPROM write event
Add LED on D5 for fade on BPM detect
Add Speaker (Piezo) on D2.
Datalogger start when two condition is satisfied:
1)program is running, and 
2)switch is moved back to ground (ON) the pin 3
If switch let the pin goes pullup (OFF), EEPROM write is stopped, and if is moved back to ground (ON)
The EEprom is write from the last address where stopped.

You can use this datalogger to see your Beats per minute (heart rate), in time, when you do fittness, run, or during your sleep.
Up to 3 hours of records
One EEPROM record at every 16 BPM ! 
If no BPM, no EEPROM records.
