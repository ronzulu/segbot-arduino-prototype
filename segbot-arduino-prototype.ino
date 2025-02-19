/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/
#include "Wire.h"
#include "DFRobot_MCP4725.h"

int sensorPin = A0;   // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor
int analogOutput = 0;
int count = 0;
#define  MCP4725_REF_VOLTAGE    5000
#define SEGWAY_LEFT   811
#define SEGWAY_CENTER   511
#define SEGWAY_RIGHT   211
#define SEGWAY_RANGE ((long)SEGWAY_LEFT - SEGWAY_RIGHT)
#define SEGWAY_MIN  SEGWAY_RIGHT

// NMV = "nominal millivolts"
// Steering sensor voltages determined empirically when powered by the Ninebot at 4.6 volts
#define NINEBOT_VOLTAGE_NMV  4600
#define NINEBOT_LEFT_NMV  2900
#define NINEBOT_CENTER_NMV  2300
#define NINEBOT_RIGHT_NMV  1700
#define NINEBOT_RANGE_NMV ((long)NINEBOT_LEFT_NMV - NINEBOT_RIGHT_NMV)

DFRobot_MCP4725 DAC;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  DAC.init(MCP4725A0_IIC_Address0, NINEBOT_VOLTAGE_NMV);

  delay(2000);

  Serial.println("segbot-arduino-prototype: v2");
}

// 
int convertSegwaySensorVoltageToNinebot(int segwaySensor) {
  if (segwaySensor < SEGWAY_RIGHT)
    segwaySensor = SEGWAY_RIGHT;
  else if (segwaySensor > SEGWAY_LEFT)
    segwaySensor = SEGWAY_LEFT;

  // Convert to Left=1000, Right=0
  long scaled = (((long)segwaySensor - SEGWAY_MIN) * 1000) / SEGWAY_RANGE;

  // long t = ((scaled * NINEBOT_RANGE_NMV) / 1000) + NINEBOT_RIGHT_NMV;
  long t = (scaled * NINEBOT_RANGE_NMV);
  long t2 = t / 1000;
  return t2 + NINEBOT_RIGHT_NMV;
}

// the loop function runs over and over again forever
void loop() {
  sensorValue = analogRead(sensorPin);
  analogOutput = convertSegwaySensorVoltageToNinebot(sensorValue);

  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  DAC.outputVoltage(analogOutput);
  Serial.print("Count: A: ");
  Serial.print(String(count++, 10));
  Serial.print(", Input: ");
  Serial.print(String(sensorValue, 10));
  Serial.print(", Output: ");
  Serial.println(String(analogOutput, 10));

  delay(1000);                      // wait for a second

  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  // DAC.outputVoltage(3500);

  delay(1000);                      // wait for a second
}
