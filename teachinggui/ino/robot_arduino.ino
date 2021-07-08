#include <ArduinoHardware.h>
#include "ros.h"
#include <std_msgs/Int16MultiArray.h>

#include <SoftwareSerial.h>
#include "MeAuriga.h"

MeSmartServo mysmartservo(PORT5); 

ros::NodeHandle nh;

float servospeed = 1;

int deltaDeg = 3;
int T1Soll = 0, T2Soll = 0, T3Soll = 0, saugerRelais, actPos = -1;
int T1Ist = 0, T2Ist = 0, T3Ist = 0;

// Initialise Message-Data
std_msgs::Int16MultiArray mbMsgPub;

// Initialise Publishers
ros::Publisher servoAnglesPublisher("/delta3d_arduino/pose", &mbMsgPub);

// Subscriber Callback Functions
void servoAnglesSubscriberCallback(const std_msgs::Int16MultiArray& poseMsg)
{
    T1Soll = poseMsg.data[0];
    T2Soll = poseMsg.data[1];
    T3Soll = poseMsg.data[2];
    saugerRelais = poseMsg.data[3];
    actPos = poseMsg.data[4];
    
    mysmartservo.moveTo(1, (long) (poseMsg.data[0]), servospeed);
    mysmartservo.moveTo(2, (long) (poseMsg.data[1]), servospeed);
    mysmartservo.moveTo(3, (long) (poseMsg.data[2]), servospeed);
    
    if (saugerRelais == 1)
    {
      digitalWrite(2, HIGH);
    }
    else
    {
      digitalWrite(2, LOW);
    }
}

// Initialise Subscribers
ros::Subscriber <std_msgs::Int16MultiArray> servoAnglesSubscriber ("/delta3d/pose", servoAnglesSubscriberCallback);

bool withinRange(int soll, int ist)
{
  if( (ist < 0 && soll < 0 && ist < soll && abs(ist) - abs(soll) < deltaDeg) ||
    (ist < 0 && soll < 0 && soll <= ist && abs(soll) - abs(ist) < deltaDeg) ||
    (ist > 0 && soll > 0 && ist < soll && soll - ist < deltaDeg) ||
    (ist > 0 && soll > 0 && soll <= ist && ist - soll < deltaDeg) || 
    (ist == 0 && soll > 0 && soll - ist < deltaDeg) ||
    (ist > 0 && soll == 0 && ist - soll < deltaDeg) ||
    (ist == 0 && soll < 0 && abs(soll) - ist < deltaDeg) ||
    (ist < 0 && soll == 0 && abs(ist) - soll < deltaDeg) ||
    (ist == 0 && soll == 0))
    {
      return true;
    }
  return false;
}

void setup()
{
  mysmartservo.begin(115200);
  delay(5);
  mysmartservo.assignDevIdRequest();
  delay(50);

  nh.getHardware()->setBaud(115200);
  nh.initNode();

  mbMsgPub.data = (int16_t *)malloc(sizeof(int16_t)*4);
  mbMsgPub.layout.dim_length = 0;
  mbMsgPub.data_length = 4;

  
  nh.advertise(servoAnglesPublisher);
  nh.subscribe(servoAnglesSubscriber);
  
  pinMode(2, OUTPUT);
}

void loop()
{
  mbMsgPub.data[0] = (int16_t) mysmartservo.getAngleRequest(1);
  mbMsgPub.data[1] = (int16_t) mysmartservo.getAngleRequest(2);
  mbMsgPub.data[2] = (int16_t) mysmartservo.getAngleRequest(3);

  T1Ist = mbMsgPub.data[0];
  T2Ist = mbMsgPub.data[1];
  T3Ist = mbMsgPub.data[2];

  
  if (withinRange(T1Soll, T1Ist) && withinRange(T2Soll, T2Ist) && withinRange(T3Soll, T3Ist) && actPos != -1)
  //if (actPos != -1)
  {
    mbMsgPub.data[3] = actPos + 1;
  }
  else
  {
    mbMsgPub.data[3] = actPos;  
  }  
 
  servoAnglesPublisher.publish( &mbMsgPub );
  nh.spinOnce();
  
  delay(1);
}
