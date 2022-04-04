#include <Arduino.h>

#include <ros.h>
#include <std_msgs/Empty.h>

/* BOARD */
int latchPin = 8;
int clockPin = 12;
int dataPin = 11;

const int REGISTERS = 2;
const int PINS = 8;
const int MAXPINS = 8 * REGISTERS;

byte *reg_state;

void regWrite(int pin, bool state);
void activateWaitDeactivate(int *pins, int size, int waitTime);
void resetRegisters();


/* ROS */
ros::NodeHandle node;

void test_msgCallBack(const std_msgs::Empty& msg);
ros::Subscriber<std_msgs::Empty> test_subscriber("toggle", &test_msgCallBack);



void setup()
{
  /* BOARD*/
  reg_state = new byte[REGISTERS];
  for (size_t i = 0; i < REGISTERS; i++)
  {
    reg_state[i] = 0;
  }

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  resetRegisters();

  /* ROS */

  node.initNode();
  node.subscribe(test_subscriber);
  
}

void loop()
{
  node.spinOnce();
  delay(1);
}

void test_msgCallBack(const std_msgs::Empty& msg)
{
  int testPins[] = {2, 3, 4, 8, 12, 14};
  int s = sizeof(testPins) / sizeof(int);
  activateWaitDeactivate(testPins, s, 1000);
}

void activateWaitDeactivate(int *pins, int size, int waitTime)
{

  for (int i = 0; i < size; i++)
  {
    regWrite(pins[i], HIGH);
  }
  delay(waitTime);
  for (int i = 0; i < size; i++)
  {
    regWrite(pins[i], LOW);
  }
  delay(100);
}

void resetRegisters()
{
  digitalWrite(latchPin, LOW);
  for (int i = 0; i < REGISTERS; i++)
  {

    byte *states = &reg_state[i];
    for (int i = 0; i < MAXPINS; i++)
    {
      bitWrite(*states, i, 0);
    }
    shiftOut(dataPin, clockPin, MSBFIRST, *states);
  }
  digitalWrite(latchPin, HIGH);
}

void regWrite(int pin, bool state)
{

  int reg = pin / PINS;
  int registerPin = pin - (PINS * reg);

  digitalWrite(latchPin, LOW);

  for (int i = 0; i < REGISTERS; i++)
  {

    byte *states = &reg_state[i];

    if (i == reg)
    {
      bitWrite(*states, registerPin, state);
    }
    shiftOut(dataPin, clockPin, MSBFIRST, *states);
  }

  digitalWrite(latchPin, HIGH);
}