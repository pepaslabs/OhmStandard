/*
  componentOvenSketch.ino: A PID-controlled component oven with serial console.
  Created by Jason M Pepas (of Pepas Labs, LLC), 2014/07/18
  Released under the terms of the MIT License.  See http://opensource.org/licenses/MIT
*/

// thanks to the following useful article: http://www.dyadica.co.uk/journal/simple-serial-string-parsing/

// note: I ran into a bunch of relocation truncation errors which I solved via http://forum.arduino.cc/index.php/topic,60649.0.html
// note that simply replacing my Arduino/hardware/tools/avr with a modern version of WinAVR was sufficient to solve the problem.
// I did not have to futz with the avrdud.conf, etc (yet...)

#include <stdlib.h>
#include <string.h> // for strcmp, strtod
#include <stdint.h> // for uint16_t

#include <SoftwareSerial.h>

#include <SoftSPI.h>
#include <MCP4801SoftSPI.h>
#include <PID_v1.h>

#include "Oversampler.h"

#define SoftSPI_MOSI_pin 3
#define SoftSPI_MISO_pin 4
#define SoftSPI_SCK_pin 5
SoftSPI mySPI(SoftSPI_MOSI_pin, SoftSPI_MISO_pin, SoftSPI_SCK_pin);

//configure the DAC chip
#define MCP4801_SLAVESELECTLOW_pin 6
#define MCP4801_LDACLOW_pin 7
#define MCP4801_SHUTDOWNLOW_pin 8

MCP4801SoftSPI voltageDAC(MCP4801_SLAVESELECTLOW_pin,
                        MCP4801_LDACLOW_pin,
                        MCP4801_SHUTDOWNLOW_pin,
                        &mySPI
                       );

double setpoint = 40.0; // degrees C
double input = 0;
double output = 0;
float kp = 0.7;
float ki = 0.3;
float kd = 0.01;
PID myPID = PID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

//uint16_t pidControlLoopPeriodInMillis = 1000;
#define pidControlLoopPeriodInMillis 1000
unsigned long timeOfLastPIDControlLoopIterationInMillis = 0;

OversamplerData oversampler;

#define RX_pin 1
#define TX_pin 2
SoftwareSerial mySerial(RX_pin, TX_pin);

#define VREF (5.0)
#define ADC_COUNTS (1024.0)
#define V_PER_C (0.18107) // determined via LTSpice
#define V_AT_ZERO_C (-3.759) // determined via LTSpice

#define DAC_GAIN 34
#define RSENSE_OHMS 0.333

// === serial console ===

#define MAX_INPUT_BUFFER_LENGTH 16
char inputBuffer[MAX_INPUT_BUFFER_LENGTH + sizeof('\0')] = {'\0'};
char *inputBufferPointer = inputBuffer;
boolean isInBufferOverflowState = false;
boolean completedInputLineIsReady = false;
char completedLineOfInput[sizeof(inputBuffer)] = {'\0'};

void setup()
{
  initOversamplerData(&oversampler);

  mySerial.begin(9600);
  // Serial.print("\nTemperature PID controller starting up!\n");
  
  //start up the SPI bus                   
  mySPI.begin();
  mySPI.setBitOrder(MSBFIRST);
  mySPI.setDataMode(SPI_MODE0);

  //start controlling the voltage supply
  voltageDAC.begin();
  
  analogReference(DEFAULT); // 5V
  
  input = analogRead(5);

  //turn the PID on
  // myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0.0, 2.0);  
}

void loop()
{
  possiblyHandleInputChar();
  possiblyHandleCompletedLineOfInput();
  possiblyIteratePIDLoop();
}

void possiblyHandleInputChar()
{
  if (mySerial.available() == 0)
  {
    return;
  }

  char ch = mySerial.read();
  handleInputChar(ch);
}

void handleInputChar(char ch)
{
  if (isInBufferOverflowState == true)
  {
    if (ch == '\n')
    {
      isInBufferOverflowState = false;
    }

    return;
  }

  if (ch == '\n')
  {
    *inputBufferPointer = '\0';
    strncpy(completedLineOfInput, inputBuffer, sizeof(inputBuffer));
    completedInputLineIsReady = true;
    inputBufferPointer = inputBuffer;
  }
  else if (ch != '\n')
  {
    *inputBufferPointer = ch;
    inputBufferPointer++;

    if (inputBufferPointer == inputBuffer + MAX_INPUT_BUFFER_LENGTH)
    {
      isInBufferOverflowState = true;
      inputBufferPointer = inputBuffer;
    }    
  }
}

/*

serial terminal interface:

dump

  print out all settings.

stop

  turn off the heater and stop the PID loop.

start

  turn on the heater and start the PID loop.

=== x1 inputs: ===

p 1000

  set the period of the PID control loop to 1000 milliseconds.

=== x10 inputs: ===

t 453

  set the temperature setpoint to 45.3 degrees celcius.

callow 253

  use the current temperature sensor reading to calibrate the lower end of the temperature linear interpolation scale to 25.3 degrees celcius.

calhigh 512

  use the current temperature sensor reading to calibrate the upper end of the temperature linear interpolation scale to 51.2 degrees celcius.

=== x1000 inputs: ===

kp 13

  set the proportional constant to 0.013.

ki 1788

  set the integral constant to 1.788.

kd 24100

  set the derivative constant to 24.1.

*/

char *start_command = "start";
char *t_command = "t";

void possiblyHandleCompletedLineOfInput()
{
  if (completedInputLineIsReady == false)
  {
    return;
  }

  handleCompletedLineOfInput(completedLineOfInput);
  completedLineOfInput[0] = '\0';
  completedInputLineIsReady = false;
}

void handleCompletedLineOfInput(char *inputLine)
{
  char *p = inputLine;
  char *command = strtok_r(p, " ", &p);

  // if (command == NULL)
  // {
  //   unknown_command();
  //   return;
  // }

//  if (strcmp(command, start_command) == 0)
//  {
//    start();
//    return;
//  }

  // if (strcmp(command, "stop") == 0)
  // {
  //   stop();
  //   return;
  // }

  char *tailptr = p;
  // float value = strtod(p, &tailptr);
  float value = (float)atoi(p);
  if (tailptr == p)
  {
    // see http://www.gnu.org/software/libc/manual/html_node/Parsing-of-Floats.html
    // see http://forum.arduino.cc/index.php/topic,42770.0.html
//    unknown_command();
    return;
  }

//  if (strcmp(command, t_command) == 0)
//  {
//    set_t(value);
//    return;
//  }

  // if (strcmp(command, "p") == 0)
  // {
  //   set_p((uint16_t)value);
  //   return;
  // }

  // if (strcmp(command, "kp") == 0)
  // {
  //   set_kp(value);
  //   return;
  // }

  // if (strcmp(command, "ki") == 0)
  // {
  //   set_ki(value);
  //   return;
  // }

  // if (strcmp(command, "kd") == 0)
  // {
  //   set_kd(value);
  //   return;
  // }

  // if (strcmp(command, "callow") == 0)
  // {
  //   cal_low(value);
  //   return;
  // }

  // if (strcmp(command, "calhigh") == 0)
  // {
  //   cal_high(value);
  //   return;
  // }

  // unknown_command();
}

/*
void dump()
{

}
*/

/*
void stop()
{
  // serial.print("\nEnabling heater and starting PID control loop.\n");
}

void start()
{
  mySerial.print("\n");
  mySerial.println(start_command);
  // mySerial.print("\nStopping PID control loop and disabling heater.\n");
}
*/

/*
void set_t(float newValue)
{
  setpoint = newValue;
  // ok();
  serialPrintSettingTo(t_command, newValue);
}

void set_p(uint16_t newValue)
{
  pidControlLoopPeriodInMillis = newValue;
  // serialPrintSettingTo("control loop period (in milliseconds)", newValue);
}
*/

void set_kp(float newValue)
{
  kp = newValue;
  // serialPrintSettingTo("proportional constant", newValue);
}

void set_ki(float newValue)
{
  ki = newValue;
  // serialPrintSettingTo("integral constant", newValue);
}

void set_kd(float newValue)
{
  kd = newValue;
  serialPrintSettingTo("derivative constant", newValue);
}

void serialPrintSettingTo(char *what, float toValue)
{
  mySerial.print("\nSetting ");
  mySerial.print(what);
  mySerial.print(" to ");
  mySerial.print(toValue);
  mySerial.print(".\n");
}

/*
void cal_low(float newValue)
{
  // serialPrintSettingTo("lower calibration point to", newValue);
}

void cal_high(float newValue)
{
  // serialPrintSettingTo("upper calibration point to", newValue);
}
*/

/*
void unknown_command()
{
  mySerial.print("\nUnknown command.\n");
}

void ok()
{
  mySerial.print("\nOK.\n");
}
*/

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
  delay(1);
  input = analogRead64x(&oversampler, 5);
  delay(1);

  int adc_input = input;  
  input = (((input / ADC_COUNTS) * VREF) - V_AT_ZERO_C) / V_PER_C; // convert to degrees celcius

  // mySerial.print("Input: ");
  // mySerial.print(adc_input);
  // mySerial.print(" (");
  // mySerial.print(Input, 3);
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

  // float amps = Output / dac_gain / rsense_ohms;

  // mySerial.print(" Output: ");
  // mySerial.print(Output, 3);
  // mySerial.print("V DAC");

  // mySerial.print(" (");
  // mySerial.print(amps, 3);
  // mySerial.print("A)");

  // mySerial.println();
}

