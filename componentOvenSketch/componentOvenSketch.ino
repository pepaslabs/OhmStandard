/*
  componentOvenSketch.ino: A PID-controlled component oven with serial console.
  Created by Jason M Pepas (of Pepas Labs, LLC), 2014/07/18
  Released under the terms of the MIT License.  See http://opensource.org/licenses/MIT
*/

// --- includes

#include <SoftwareSerial.h>
#include <SoftSPI.h>
#include <MCP4801SoftSPI.h>
#include <PID_v1.h>

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
float kd = 0.0;
PID myPID = PID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
bool heaterEnabled = true;

// ---

void setup()
{
  mySerial.begin(9600);
  
  // start up the SPI bus                   
  mySPI.begin();
  mySPI.setBitOrder(MSBFIRST);
  mySPI.setDataMode(SPI_MODE0);

  // start controlling the voltage supply
  voltageDAC.begin();
  
  analogReference(DEFAULT); // 5V
  
  input = analogRead(MCP9701_pin);

  // turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0.01, 2.0);
}

void loop()
{
  handleInputChar();
  iteratePIDLoop();
  dump();
  delay(1000);
}

// --- serial terminal interface

void handleInputChar()
{
  if (mySerial.available() == 0)
  {
    return;
  }

/* serial terminal interface (all single-character commands):

p: increase the proportional constant by 1%.
l ('ell'): decrease the proportional constant by 1%.

i: increase the integral constant by 1%.
j: decrease the integral constant by 1%.

q: stop the PID loop and turn off the heater.
s: turn on the heater and start the PID loop.

*/

  char ch = mySerial.read();
  switch(ch)
  {
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
      
    case 'q':
      heaterEnabled = false;
      voltageDAC.setVoltageOutput(0);
      break;
    
    case 's':
      heaterEnabled = true;
      break;
  }
}

// --- PID implemetation

void iteratePIDLoop()
{
  // 64x oversampling
  uint16_t accumulator = 0;
  for (uint8_t count=0; count < 64; count++)
  {
    accumulator += analogRead(MCP9701_pin);
  }
  input = accumulator >> 6;

  myPID.Compute();
  
  if (heaterEnabled == true)
  {
    voltageDAC.setVoltageOutput(output);
  }
}

void dump()
{
  // example dump:
  // kp 132 ki 243 h 298
  
  char buff[16];

  mySerial.print("kp: ");
  itoa((uint16_t)kp, buff, 10);
  mySerial.print(buff);

  mySerial.print(" \tki: ");
  itoa((uint16_t)ki, buff, 10);
  mySerial.print(buff);
  
  mySerial.print(" \tt: ");
  itoa((uint16_t)input, buff, 10);
  mySerial.println(buff);
  
  mySerial.print(" \th: ");
  itoa((uint16_t)(output*1000), buff, 10);
  mySerial.print(buff);
  mySerial.println("mV");
}

