//#include <FIR.h>

#include <TeensyThreads.h>

#include <ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <geometry_msgs/WrenchStamped.h>
#define SAMPLING_RATE_KHZ 10
IntervalTimer myTimer;


float phase = 0.0;
float freq = 100;
float value = 0.0;


float d_phase = 0.05;


float twopi = 3.13159 * 2;


float voltage;
float LastVoltage;
float rate;
float curT;

float filtR = 0.0;

unsigned long lastTime = 0;

unsigned long dt = 5; // dt in milliseconds





ros::NodeHandle nh;
//geometry_msgs::WrenchStamped msg;
float varData = 0;
float rotData = 0;

bool thereIsAChange = false;

float normRate = 0;

float sensorValue;

int curState = 0;

String modeString;
int modeInt = 10  ;

float eventData = 0.0;


  float maximum=0.0;
  float minimum=1023.0;

  
void messageCb(const std_msgs:: Float32 &msg) {
  eventData = round(msg.data*100.0)/100.0;
  //eventData = msg.data;

  //char result2[13];


  
  //dtostrf(eventData , 13, 10, result2);
  //nh.loginfo("I see the topic");
  //nh.loginfo(result2);
}


std_msgs::Float64 str_msg;


////complete subscriber

ros::Subscriber<std_msgs::Float32> s("chatter", &messageCb);



static float filterloop(float in) {
  static float flt = 0.;
  static float a = .01;
  flt = a * in + (1. - a) * flt;
  return flt;

}


void setup() {
  analogWriteResolution(12);
  analogReadResolution(12);

  //Serial1.begin(9600);
  nh.initNode();



  int time_us;
  time_us = 1000 / SAMPLING_RATE_KHZ;
  myTimer.begin(teensy, time_us);

}

void loop() {

  nh.subscribe(s);
  nh.spinOnce();

  //delay(2);
  if (millis() - lastTime  >= dt)   // wait for dt milliseconds
  {
    lastTime = millis();
    if (eventData >= 0.0 && eventData <= 8.0) {
      sensorValue = eventData;
    } else {
      sensorValue = 0;

    }
    voltage = sensorValue; // this line changed !!

    rate = (voltage - LastVoltage);
    char result3[13];


    if (rate!=0.0){
      dtostrf(rate , 0, 5, result3);
      nh.loginfo(result3);
    }


    LastVoltage = voltage;

    normRate = abs(((rate + 3.5) / 7) - 0.5) ;

  }
  

}

void teensy(void) {
  if (rate != 0.0) {
    filtR = filterloop(rate);
  } else {
    filtR = 0.0;
  }



  //value = sin(phase)*varData*50;

  //coefficients for d_phase=0.1
  //4000 max = 7.1V (PtP) no amp
  //900-950 max = 11.1 - 11.3V(PtP) DC RMS-FS = 3.47V with amp


  //abs(rate)<= 23 &&
  //if (abs(rate) != 0.0 && normRate >= 0.01 && normRate <= 0.3) {
  
    //VIBRO IS HERE
    
    if (sensorValue!=0 && abs(rate)!=0.0 && abs(rate) > 0.1){
          value = sin(phase * abs(1 + filtR) + twopi) * 300 + 300; //+2048;
    }



   // thereIsAChange = true;
//  } else {
//    value = 0;
//    thereIsAChange = false;
//  }



  phase = phase + d_phase;
  if (phase > twopi) {
    phase = 0;
  }
  analogWrite(A21, (int)value);
  analogWrite(A22, (int)value);


}
