#include <Arduino.h>

#include <ros.h>
// #include <std_msgs/Empty.h>
#include <std_msgs/UInt8MultiArray.h>
//#include <std_srvs/Trigger.h>

#define REGISTERS 2
#define PINS 8
#define MAXPINS 16

/* BOARD */
#define latchPin 8
#define clockPin 12
#define dataPin 11



byte reg_state[REGISTERS];

void regWrite(int pin, bool state);
void activateWaitDeactivate(int *pins, int size, int waitTime);
void resetRegisters();


/* ROS */
ros::NodeHandle node;

// Test board subscriber
// void test_msgCallBack(const std_msgs::Empty& msg);
// ros::Subscriber<std_msgs::Empty> testSubscriber("toggle", &test_msgCallBack);

// Register State subscrier
void registerStateCallBack(const std_msgs::UInt8MultiArray& msg);
ros::Subscriber<std_msgs::UInt8MultiArray> registerStateSubscriber("register_state", &registerStateCallBack);

// using std_srvs::Trigger;
// // Reset Service
// void resetCallback(const Trigger::Request &req, Trigger::Response &res);
// ros::ServiceServer<Trigger::Request, Trigger::Response> resetService("reset_srv",&resetCallback);

// Register state publisher 
// publishing -> [0,...,0] states of pins 
// std_msgs::UInt8MultiArray registerState;
// void setupRegBuffer();
// ros::Publisher pubResisterState("register_state", &registerState);

void setup()
{
  /* BOARD*/
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
  // setupRegBuffer();

  // node.subscribe(testSubscriber);
  node.subscribe(registerStateSubscriber);
  // node.advertiseService(resetService);
}

void loop()
{
  // publish register state
  // for (size_t i = 0; i < REGISTERS; i++)
  // {
  //   registerState.data[i] = reg_state[i];
  // }
  // memcpy(registerState.data, reg_state, REGISTERS);
  // pubResisterState.publish(&registerState);

  node.spinOnce();
  delay(1);
}

// void setupRegBuffer()
// {
//   registerState.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension));
//   registerState.layout.dim[0].label = "states";
//   registerState.layout.dim[0].size = REGISTERS;
//   registerState.layout.dim[0].stride = 1;
//   registerState.layout.data_offset = 0;
//   registerState.data = (byte *)malloc(sizeof(byte)*REGISTERS);
//   registerState.data_length = REGISTERS;
//   node.advertise(pubResisterState);
// }

// void test_msgCallBack(const std_msgs::Empty& msg)
// {
//   int testPins[] = {2, 3, 4, 8, 12, 14};
//   int s = sizeof(testPins) / sizeof(int);
//   activateWaitDeactivate(testPins, s, 1000);
// }

// void resetCallback(const Trigger::Request &req, Trigger::Response &res)
// {
//   resetRegisters();
//   res.success = true;
// }

void registerStateCallBack(const std_msgs::UInt8MultiArray& msg)
{
  for (size_t i = 0; i < MAXPINS; i++)
  {
    if (msg.data[i]==1)
    {
      regWrite(i, HIGH);
    }
    else
    {
      regWrite(i,LOW);
    }
  }
  delay(10);
}

// void activateWaitDeactivate(int *pins, int size, int waitTime)
// {

//   for (int i = 0; i < size; i++)
//   {
//     regWrite(pins[i], HIGH);
//   }
//   delay(waitTime);
//   for (int i = 0; i < size; i++)
//   {
//     regWrite(pins[i], LOW);
//   }
//   delay(100);
// }

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