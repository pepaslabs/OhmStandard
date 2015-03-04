/*
  componentOvenSketch.ino: A PID-controlled component oven with serial console.
  Created by Jason M Pepas (of Pepas Labs, LLC), 2014/07/18
  Released under the terms of the MIT License.  See http://opensource.org/licenses/MIT
*/

// thanks to the following useful article: http://www.dyadica.co.uk/journal/simple-serial-string-parsing/

// note: I ran into a bunch of relocation truncation errors which I solved via http://forum.arduino.cc/index.php/topic,60649.0.html
// note that simply replacing my Arduino/hardware/tools/avr with a modern version of WinAVR was sufficient to solve the problem.
// I did not have to futz with the avrdud.conf, etc (yet...)

// --- includes

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <PID_v1.h>

#include <SoftwareSerial.h>
#include <SoftSPI.h>
#include <MCP4801SoftSPI.h>

// --- ATTiny pins

#define RX_pin 1
#define MCP9701_pin 2
#define TX_pin 3
#define SoftSPI_MOSI_pin 3
#define SoftSPI_MISO_pin 4
#define SoftSPI_SCK_pin 5
#define MCP4801_SLAVESELECTLOW_pin 6
#define MCP4801_LDACLOW_pin 7
#define MCP4801_SHUTDOWNLOW_pin 8

// --- defines

#define VREF (5.0)
#define ADC_COUNTS (1024.0)
#define V_PER_C (0.18107) // determined via LTSpice
#define V_AT_ZERO_C (-3.759) // determined via LTSpice

#define DAC_GAIN 34
#define RSENSE_OHMS 0.333

// --- SPI / MCP4801 DAC setup

SoftSPI mySPI(SoftSPI_MOSI_pin, SoftSPI_MISO_pin, SoftSPI_SCK_pin);

MCP4801SoftSPI voltageDAC(MCP4801_SLAVESELECTLOW_pin,
                          MCP4801_LDACLOW_pin,
                          MCP4801_SHUTDOWNLOW_pin,
                          &mySPI
                         );

// --- serial port setup

SoftwareSerial mySerial(RX_pin, TX_pin);

// --- PID setup

double input = 0;
double output = 0;
double setpoint = 40.0; // degrees C
float kp = 0.7;
float ki = 0.3;
float kd = 0.01;
float kd_backup = 0;
PID myPID = PID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

uint16_t pidControlLoopPeriodInMillis = 1000;
unsigned long timeOfLastPIDControlLoopIterationInMillis = 0;

// ---

void setup()
{
  mySerial.begin(9600);
  // Serial.print("\nTemperature PID controller starting up!\n");
  
  //start up the SPI bus                   
  mySPI.begin();
  mySPI.setBitOrder(MSBFIRST);
  mySPI.setDataMode(SPI_MODE0);

  //start controlling the voltage supply
  voltageDAC.begin();
  
  analogReference(DEFAULT); // 5V
  
  input = analogRead(MCP9701_pin);

  //turn the PID on
  // myPID.SetMode(AUTOMATIC);
//  myPID.SetOutputLimits(0.0, 2.0);
}

void loop()
{
  possiblyHandleInputChar();
  possiblyIteratePIDLoop();
}

// --- PID implemetation

void possiblyIteratePIDLoop()
{
  unsigned long currentTimeInMillis = millis();
  // supposedly this is sufficient to handle overflow?  see http://www.baldengineer.com/blog/2012/07/16/arduino-how-do-you-reset-millis/
  unsigned long elapsedTime = (unsigned long)(currentTimeInMillis - timeOfLastPIDControlLoopIterationInMillis);
  if (elapsedTime >= pidControlLoopPeriodInMillis)
  {
    iteratePIDLoop();
    timeOfLastPIDControlLoopIterationInMillis = currentTimeInMillis;
  }
}

void iteratePIDLoop()
{
  // 64x oversampling
  uint16_t accumulator = 0;
  for (uint8_t count=0; count < 64; count++)
  {
    accumulator += analogRead(MCP9701_pin);
  }
  input = accumulator >> 6;
  
  mySerial.println(input);
//  input = (((input / ADC_COUNTS) * VREF) - V_AT_ZERO_C) / V_PER_C; // convert to degrees celcius

//  mySerial.print("Input: ");
  // mySerial.print(" (");
//  mySerial.print(input, 3);
  // mySerial.print("C)");


  myPID.Compute();

  if (output <= 0.001)
  {
    voltageDAC.setVoltageOutputBits(0, 0);
  }
  else
  {
    voltageDAC.setVoltageOutput(output);  
  }


//  float amps = output / DAC_GAIN / RSENSE_OHMS;

  // mySerial.print(" Output: ");
  // mySerial.print(Output, 3);
  // mySerial.print("V DAC");

  // mySerial.print(" (");
//  mySerial.println(amps, 3);
  // mySerial.print("A)");

//  mySerial.println(".");
}

// --- serial terminal interface

void possiblyHandleInputChar()
{
  if (mySerial.available() == 0)
  {
    return;
  }

/* serial terminal interface (all single-character commands):

? (or /): (help) print a help message.
\: (dump) print out all settings.

w: (warm): turn on the heater and start the PID loop.
q: (quit) turn off the heater and stop the PID loop.

+ (or =): increase the temperature setpoint by 1 degree celcius
-: decrease the temperature setpoint by 1 degree celcius

'(': (calibrate low) set the lower end of the temperature linear interpolation scale to the current temperature sensor reading.
')': (calibrate high) set the upper end of the temperature linear interpolation scale to the current temperature sensor reading.

p: increase the proportional constant by 1%.
l ('ell'): decrease the proportional constant by 1%.

i: increase the integral constant by 1%.
j: decrease the integral constant by 1%.

d: increase the derivative constant by 1%.
x: decrease the derivative constant by 1%.
D: toggle derivative control on/off.

*/

  char ch = mySerial.read();
  switch(ch)
  {
    case '?':
    case '/':
      print_help_message();
      break;
    
    case '\\':
      dump();
      break;
    
    case 'w':
      turn_on_heater_and_start_PID_loop();
      break;
    
    case 'q':
      turn_off_heater_and_stop_PID_loop();
      break;
    
    case '+':
    case '=':
      setpoint += 1;
      break;
    
    case '-':
      setpoint -= 1;
      break;
    
    case '(':
      calibrate_low();
      break;
    
    case ')':
      calibrate_high();
      break;
    
    case 'p':
      kp *= 1.01;
      break;
    
    case 'l':
      kp *= 0.99;
      break;
    
    case 'i':
      ki *= 1.01;
      break;
    
    case 'j':
      ki *= 0.99;
      break;
    
    case 'd':
      kd *= 1.01;
      break;
    
    case 'x':
      kd *= 0.99;
      break;
    
    case 'D':
      toggle_derivative_control();
      break;
  }

}

void print_help_message() {}
void dump() {}
void calibrate_low() {}
void calibrate_high() {}
void turn_on_heater_and_start_PID_loop() { }
void turn_off_heater_and_stop_PID_loop() { }

void toggle_derivative_control()
{
  if (kd == 0)
  {
    kd = kd_backup;
  }
  else
  {
    kd_backup = kd;
    kd = 0;
  }
}

