/*
  componentOvenSketch.ino: A PID-controlled component oven with serial console.
  Created by Jason M Pepas (of Pepas Labs, LLC), 2014/07/18
  Released under the terms of the MIT License.  See http://opensource.org/licenses/MIT
*/

// thanks to the following useful article: http://www.dyadica.co.uk/journal/simple-serial-string-parsing/

#include <SPI.h>
#include <MCP4801.h>
#include <PID_v1.h>
#include "Oversampler.h"

//configure the DAC chip
MCP4801Class voltageDAC(10,  //slaveSelectLowPin
                        8,  //ldacLowPin
                        9  //shutdownLowPin
                       );

double Setpoint, Input, Output;
float kp = 0.7;
float ki = 0.3;
float kd = 0.01;
PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);

OversamplerData *oversampler = NULL;

void setup()
{
  Serial.begin(9600, SERIAL_8N1);
  Serial.println("Temperature PID controller startup!");
  
  //start up the SPI bus                   
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  //start controlling the voltage supply
  voltageDAC.begin();
  
  analogReference(DEFAULT); // 5V
  
  Input = analogRead(5);
  Setpoint = 40.0; // degrees C

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0.0, 2.0);
  
  oversampler = createOversamplerData();
}

const float vref = 5.0;
const float adc_counts = 1024.0;
const float v_per_c = 0.18107; // determined via LTSpice
const float v_at_zero_c = -3.759; // determined via LTSpice

const float dac_gain = 34;
const float rsense_ohms = 0.333;


// === serial console ===


#define MAX_INPUT_BUFFER_LENGTH 16
char inputBuffer[MAX_INPUT_BUFFER_LENGTH + sizeof('\0')];
char *inputBufferPointer = inputBuffer;
boolean isInBufferOverflowState = false;
boolean completedInputLineIsReady = false;
String completedLineOfInput;

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
    inputBufferPointer = '\0';
    completedLineOfInput = String(inputBuffer)
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

stop

  turn off the heater and stop the PID loop

start

  turn on the heater and start the PID loop

kp 1.0

  set the proportional constant to 1.0

ki 1.0

  set the integral constant to 1.0

kd 1.0

  set the derivative constant to 1.0

callow 25.0

  use the current temperature sensor reading to calibrate the lower end of the temperature linear interpolation scale to 25.0 degrees celcius

calhigh 50.0

  use the current temperature sensor reading to calibrate the upper end of the temperature linear interpolation scale to 50.0 degrees celcius

*/

void possiblyHandleCompletedLineOfInput()
{
  if (completedLineOfInput == NULL)
  {
    return;
  }

  handleCompletedLineOfInput(completedLineOfInput);
  completedLineOfInput = NULL;
}

void handleCompletedLineOfInput(String inputLine)
{
  char *p = inputLine;
  char *command = strtok_r(p, " ", &p);

  switch(command)
  {
    case 'start':
    {
      start();
      return;
    }

    case 'stop':
    {
      stop();
      return;
    }
  }

  char *tailptr = p;
  float value = strtof(p, &tailptr);
  if (tailptr == p)
  {
    // see http://www.gnu.org/software/libc/manual/html_node/Parsing-of-Floats.html
    // see http://forum.arduino.cc/index.php/topic,42770.0.html
    unkonwn_command();
    return;
  }

  switch(command)
  {
    case 'kp':
    {
      set_kp(value);
      return;
    }

    case 'ki':
    {
      set_ki(value);
      return;
    }

    case 'kd':
    {
      set_kd(value);
      return;
    }

    case 'callow':
    {
      cal_low(value);
      return;
    }

    case 'calhigh':
    {
      cal_high(value);
      return;
    }
  }

  unknown_command();
}

void set_kp(float newValue)
{
  kp = newValue;
  serialPrintSettingTo("proportional constant", newValue);
}

void set_ki(float newValue)
{
  ki = newValue;
  serialPrintSettingTo("integral constant", newValue);
}

void set_kd(float newValue)
{
  kd = newValue;
  serialPrintSettingTo("derivative constant", newValue);
}

void serialPrintSettingTo(String what, float toValue)
{
  Serial.print("\nSetting ");
  Serial.print(what);
  Serial.print(" to ");
  Serial.print(toValue);
  Serial.print(".\n");
}

void cal_low(float newValue)
{
  serialPrintSettingTo("lower calibration point to", newValue);
}

void cal_high(float newValue)
{
  serialPrintSettingTo("upper calibration point to", newValue);
}

void unkonwn_command()
{
  Serial.print("\nUnknown command.\n");
}



void loop()
{
  possiblyHandleInputChar();
  possiblyHandleCompletedLineOfInput();
  possiblyIteratePIDLoop();

  delay(10);
  Input = analogRead64x(oversampler, 5);
  delay(10);
  
  int adc_input = Input;  
  Input = (((Input / adc_counts) * vref) - v_at_zero_c) / v_per_c; // convert to degrees celcius

  Serial.print("Input: ");
  Serial.print(adc_input);
  Serial.print(" (");
  Serial.print(Input, 3);
  Serial.print("C)");

  myPID.Compute();

  if (Output <= 0.001)
  {
    voltageDAC.setVoltageOutputBits(0, 0);
  }
  else
  {
    voltageDAC.setVoltageOutput(Output);  
  }

  float amps = Output / dac_gain / rsense_ohms;

  Serial.print(" Output: ");
  Serial.print(Output, 3);
  Serial.print("V DAC");

  Serial.print(" (");
  Serial.print(amps, 3);
  Serial.print("A)");

  Serial.println();
  delay(500);
}

