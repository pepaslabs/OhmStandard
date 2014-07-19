/*
  componentOvenSketch.ino: A PID-controlled component oven with serial console.
  Created by Jason M Pepas (of Pepas Labs, LLC), 2014/07/18
  Released under the terms of the MIT License.  See http://opensource.org/licenses/MIT
*/

// thanks to the following useful article: http://www.dyadica.co.uk/journal/simple-serial-string-parsing/

#include <stdlib.h>
#include <string.h> // for strcmp, strtod

#include <stdint.h> // for uint16_t

#include <SPI.h>
#include <MCP4801.h>
#include <PID_v1.h>

#include "Oversampler.h"
// Oversampler uses about 176 bytes (of 8K)

//configure the DAC chip
MCP4801Class voltageDAC(10,  //slaveSelectLowPin
                        8,  //ldacLowPin
                        9  //shutdownLowPin
                       );

double setpoint = 40.0; // degrees C
double input = 0;
double output = 0;
float kp = 0.7;
float ki = 0.3;
float kd = 0.01;
PID myPID = PID(&input, &output, &setpoint, kp, ki, kd, DIRECT);


uint16_t pidControlLoopPeriodInMillis = 1000;
unsigned long timeOfLastPIDControlLoopIterationInMillis = 0;

OversamplerData oversampler;

void setup()
{
  initOversamplerData(&oversampler);

  Serial.begin(9600, SERIAL_8N1);
  // Serial.print("\nTemperature PID controller starting up!\n");
  
  //start up the SPI bus                   
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  //start controlling the voltage supply
  voltageDAC.begin();
  
  analogReference(DEFAULT); // 5V
  
  input = analogRead(5);

  //turn the PID on
  // myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0.0, 2.0);  
}

const float vref = 5.0;
const float adc_counts = 1024.0;
const float v_per_c = 0.18107; // determined via LTSpice
const float v_at_zero_c = -3.759; // determined via LTSpice

const float dac_gain = 34;
const float rsense_ohms = 0.333;


// === serial console ===


#define MAX_INPUT_BUFFER_LENGTH 16
char inputBuffer[MAX_INPUT_BUFFER_LENGTH + sizeof('\0')] = {'\0'};
char *inputBufferPointer = inputBuffer;
boolean isInBufferOverflowState = false;
boolean completedInputLineIsReady = false;
char completedLineOfInput[sizeof(inputBuffer)] = {'\0'};

void possiblyHandleInputChar()
{
  if (Serial.available() == 0)
  {
    return;
  }

  char ch = Serial.read();
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

  if (strcmp(command, start_command) == 0)
  {
    start();
    return;
  }

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
    unknown_command();
    return;
  }

  if (strcmp(command, t_command) == 0)
  {
    set_t(value);
    return;
  }

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

void dump()
{

}

void stop()
{
  // Serial.print("\nEnabling heater and starting PID control loop.\n");
}

void start()
{
  Serial.print("\n");
  Serial.println(start_command);
  // Serial.print("\nStopping PID control loop and disabling heater.\n");
}

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
  Serial.print("\nSetting ");
  Serial.print(what);
  Serial.print(" to ");
  Serial.print(toValue);
  Serial.print(".\n");
}

void cal_low(float newValue)
{
  // serialPrintSettingTo("lower calibration point to", newValue);
}

void cal_high(float newValue)
{
  // serialPrintSettingTo("upper calibration point to", newValue);
}

void unknown_command()
{
  Serial.print("\nUnknown command.\n");
}

void ok()
{
  Serial.print("\nOK.\n");
}

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
  input = (((input / adc_counts) * vref) - v_at_zero_c) / v_per_c; // convert to degrees celcius

  // Serial.print("Input: ");
  // Serial.print(adc_input);
  // Serial.print(" (");
  // Serial.print(Input, 3);
  // Serial.print("C)");

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

  // Serial.print(" Output: ");
  // Serial.print(Output, 3);
  // Serial.print("V DAC");

  // Serial.print(" (");
  // Serial.print(amps, 3);
  // Serial.print("A)");

  // Serial.println();
}

void loop()
{
  possiblyHandleInputChar();
  possiblyHandleCompletedLineOfInput();
  possiblyIteratePIDLoop();
}

