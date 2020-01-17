/*
   Code to detect pulses from the PulseSensor,
   using an interrupt service routine.

   >>>>  THIS EXAMPLE OUTPUTS USES TONE COMMAND <<<<
   >>>>  TO MAKE A SPEAKER BEEP WITH HEARTBEAT! <<<<

   Here is a link to the tutorial
   https://pulsesensor.com/pages/pulse-sensor-speaker-tutorial

   Copyright World Famous Electronics LLC - see LICENSE
   Contributors:
     Joel Murphy, https://pulsesensor.com
     Yury Gitman, https://pulsesensor.com
     Bradford Needham, @bneedhamia, https://bluepapertech.com

   Licensed under the MIT License, a copy of which
   should have been included with this software.

   This software is not intended for medical use.
*/

/*
   Every Sketch that uses the PulseSensor Playground must
   define USE_ARDUINO_INTERRUPTS before including PulseSensorPlayground.h.
   Here, #define USE_ARDUINO_INTERRUPTS true tells the library to use
   interrupts to automatically read and process PulseSensor data.

   See ProcessEverySample.ino for an example of not using interrupts.
*/
#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>  //Tools/ Manage Libraries... add PulseSensorPlayground
#include <EEPROM.h>
#include <avr/sleep.h>
/*
   The format of our output.

   Set this to PROCESSING_VISUALIZER if you're going to run
    the Processing Visualizer Sketch.
    See https://github.com/WorldFamousElectronics/PulseSensor_Amped_Processing_Visualizer

   Set this to SERIAL_PLOTTER if you're going to run
    the Arduino IDE's Serial Plotter.
*/
const int OUTPUT_TYPE = PROCESSING_VISUALIZER;

/*
   Pinout:
     PULSE_INPUT = Analog Input. Connected to the pulse sensor
      purple (signal) wire.
     PULSE_BLINK = digital Output. Connected to an LED (and 220 ohm resistor)
      that will flash on each detected pulse.
     PULSE_FADE = digital Output. PWM pin onnected to an LED (and resistor)
      that will smoothly fade with each pulse.
      NOTE: PULSE_FADE must be a pin that supports PWM. Do not use
      pin 9 or 10, because those pins' PWM interferes with the sample timer.
*/
const int PULSE_INPUT = A0;
const int PULSE_BLINK = 13;    // Pin 13 is the on-board LED
const int PULSE_FADE = 5;
const int ON_OFF_PULSE = 6;
const int THRESHOLD = 650;   // Adjust this number to avoid noise when idle
int stay,  //for delay BPM
    adr_EPR = 0;//EEPROM address
/*
   All the PulseSensor Playground functions.
*/
PulseSensorPlayground pulseSensor;

/*
  Setup the things we need for driving the Speaker
  NOTE: Speaker MUST be AC coupled! Connect PIN_SPEAKER to red speaker wire.
        Then connect black speaker wire to + side of electrolytic capacitor.
        Then connect - side of electrolytic capacitor to GND.
        Capacitor value should be 1uF or higher!
        Follow this tutorial:
        [link]
*/

const int  BUTTON_PIN = 3;     // EEPROM condition button + intrerupt state
bool buttonState = 0;         // variable for reading the switch button status
const int PIN_SPEAKER = 2;    // speaker on pin2 makes a beep with heartbeat


void setup() {
  /*
     Use 115200 baud because that's what the Processing Sketch expects to read,
     and because that speed provides about 11 bytes per millisecond.

     If we used a slower baud rate, we'd likely write bytes faster than
     they can be transmitted, which would mess up the timing
     of readSensor() calls, which would make the pulse measurement
     not work properly.
  */
  pinMode(BUTTON_PIN, INPUT_PULLUP); // check for LOW = EEPROM read
  pinMode(4, INPUT_PULLUP);//useless setting, conected with BUTTON_PIN for easy fit DIP switch on the board NANO
  pinMode(ON_OFF_PULSE, OUTPUT);
  digitalWrite(ON_OFF_PULSE, LOW);
   Serial.begin(115200);
   buttonState = digitalRead(BUTTON_PIN);
   if (buttonState == LOW) {
      attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), wakeUpNow, RISING); // use interrupt 1 (pin 3) and run function
                                      // wakeUpNow(function at the end of sketch) when pin 3 gets HIGH 
      while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
      }
      Serial.println(F("CLEARDATA")); //for PLX-DAQ2 (EXCEL file)
      Serial.println(F("LABEL, Data, Curent_number, BPM"));
      for ( int index = 0 ; index < EEPROM.length(); index++) { // send data from EEPROM to Excel file
        byte val_EPR = EEPROM.read(index);
        delay(80);//test between 80 and 100 for optimal transfer to excel PLX-DAQ2
        if (val_EPR == 255){//find the marker and stop data transfer 
          goto Final;
        }
        Serial.println( (String) "DATA,DATE, " + index + "," + val_EPR + ",AUTOSCROLL_20" );
        
      }
 Final: 
      Serial.println(F("BEEP"));
      Serial.println(F("DONE"));//Will flush the Serial port on Excel side PLX-DAQ2
      Serial.println(F("STOPLOGGING")); //disconnect PLX-DAQ2
      Serial.end();
   set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode set 
   sleep_enable();
   sleep_cpu();
    } 
  // Configure the PulseSensor manager.
  
  digitalWrite(ON_OFF_PULSE, HIGH);
    Serial.begin(115200);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
     }
  pulseSensor.analogInput(PULSE_INPUT);
//  pulseSensor.blinkOnPulse(PULSE_BLINK);
  pulseSensor.fadeOnPulse(PULSE_FADE);

  pulseSensor.setSerial(Serial);
  pulseSensor.setOutputType(OUTPUT_TYPE);
  pulseSensor.setThreshold(THRESHOLD);

  // Now that everything is ready, start reading the PulseSensor signal.
  if (!pulseSensor.begin()) {
    /*
       PulseSensor initialization failed,
       likely because our particular Arduino platform interrupts
       aren't supported yet.

       If your Sketch hangs here, try changing USE_ARDUINO_INTERRUPTS to false.
       which doesn't use interrupts.
    */
    for(;;) {
      // Flash the led to show things didn't work.
      digitalWrite(PULSE_BLINK, LOW);
      delay(50);
      digitalWrite(PULSE_BLINK, HIGH);
      delay(50);
    }
  }
}

void loop() {
  /*
     Wait a bit.
     We don't output every sample, because our baud rate
     won't support that much I/O.
  */
  delay(18);
   buttonState = digitalRead(BUTTON_PIN);
  // write the latest sample to Serial.
  pulseSensor.outputSample();

  /*
     If a beat has happened since we last checked,
     write the per-beat information to Serial.
     write a frequency to the PIN_SPEAKER
     NOTE: Do not set the optional duration of tone! That is blocking!
   */
  if (pulseSensor.sawStartOfBeat()) {
    pulseSensor.outputBeat();
    tone(PIN_SPEAKER,1500);              // tone(pin,frequency)
    if ((buttonState == LOW) && (adr_EPR < EEPROM.length())){//check for EEPROM write
      stay += 1;
    }
     if (stay >= 15){     // save in EEPROM ones in 16th BPM reads
        stay = 0;          //aprox 3 hours BPM in 1024 memories
        EEPROM.write(adr_EPR, (pulseSensor.getBeatsPerMinute()));
        adr_EPR += 1;
        digitalWrite(PULSE_BLINK, bitRead(adr_EPR, 0));//pin pt LED 16 BPM OFF, 16 BPM ON, for EEPROM check
        EEPROM.write(adr_EPR,255);//stop marker - for partial new logs over previous logs
      }
  }

  /*
    The Pulse variable is true only for a short time after the heartbeat is detected
    Use this to time the duration of the beep
  */
  if(pulseSensor.isInsideBeat() == false){
    noTone(PIN_SPEAKER);
  }

}
void wakeUpNow()        // here the interrupt is handled after wakeup
{
      Serial.begin(115200);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
     }
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
     sleep_disable();       // disable sleep...                   
    detachInterrupt(1);      // disables interrupt 1 on pin 3
}
