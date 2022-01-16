#include <PID_v1.h>

#include<ros.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/UInt16.h>

#define Renc 2
#define Lenc 3
#define Rpwm 6
#define Lpwm 4


std_msgs:: UInt16 rspeed_msg;
std_msgs:: UInt16 lspeed_msg;
std_msgs:: UInt16 setpoint_msg;
std_msgs:: UInt16 lpid_msg;
std_msgs:: UInt16 rpid_msg;

ros:: Publisher right_motor("/motor_speeds/right_motor", &rspeed_msg);
ros:: Publisher left_motor("/motor_speeds/left_motor", &lspeed_msg);
ros::Publisher setpoint_pub("/motor_ctrl/setpoint", & setpoint_msg);
ros::Publisher rpid_pub("/motor_ctrl/pid_output", &rpid_msg);
ros::Publisher lpid_pub("/motor_ctrl/pid_output", &lpid_msg);


ros:: NodeHandle nd;




volatile unsigned long Rcounter = 0, Lcounter = 0; int RRPM = 0, LRPM = 0;
int dt = 200; double ppr = 3657 * 2 ; // 20 * 32  --> double 640 for wc motors 3657 by experiment 
  
double setpoint, rinput, loutput, kp = 5, kd = 0.0002, ki = 1;
double routput, linput;
unsigned long prev_time = millis();
unsigned long new_time = millis();


PID right_mtr_ctrl (&rinput, &routput, &setpoint, kp, ki, kd, DIRECT);

PID left_mtr_ctrl (&linput, &loutput, &setpoint, kp, ki, kd, DIRECT);






void rencoder()
{

  Rcounter ++;

}

void lencoder()
{

  Lcounter ++;

}




void setup() {
  // put your setup code here, to run once:
  pinMode(Renc, INPUT_PULLUP); pinMode(Lenc, INPUT_PULLUP); pinMode(Rpwm, OUTPUT); pinMode(Lpwm, OUTPUT);
  pinMode(5, OUTPUT); 
  attachInterrupt(digitalPinToInterrupt(2), rencoder, RISING);
  attachInterrupt(digitalPinToInterrupt(3), lencoder, CHANGE);

  Serial.begin(57600);


  right_mtr_ctrl.SetMode(AUTOMATIC);
  left_mtr_ctrl.SetMode(AUTOMATIC);
  right_mtr_ctrl.SetSampleTime(20);
 

  nd.initNode();
  nd.advertise(right_motor);
  nd.advertise(left_motor);
  nd.advertise(rpid_pub);
  nd.advertise(lpid_pub);
  nd.advertise(setpoint_pub);


}

void loop() {
  // put your main code here, to run repeatedly:

  new_time = millis();
  if ((new_time - prev_time) >= dt)
  { 
    RRPM = (float) Rcounter / (ppr * dt* 0.001) * 60UL;
    LRPM =  (float) Lcounter / ppr / (dt / 1000.0) * 60UL;

    int speed_reading = analogRead(A5);
    setpoint = map(speed_reading, 0, 1023, 0, 140);

    rinput = RRPM; linput = LRPM;
    right_mtr_ctrl.Compute();
    left_mtr_ctrl.Compute();

    analogWrite(Rpwm, routput);
    analogWrite(Lpwm, loutput);
    Serial.print("setpoint is ");
       Serial.print(setpoint);
      Serial.print(" speed is:   ");
       Serial.print(RRPM);
        Serial.print(" pid is:   ");
         Serial.println(routput);
      Serial.print(" left pid is:   ");
         Serial.println(loutput);

    rspeed_msg.data = Rcounter;
    lspeed_msg.data = LRPM;
    setpoint_msg.data = setpoint;
    rpid_msg.data = routput;
   // lpid_msg.data = loutput;
    lpid_msg.data = loutput;

    right_motor.publish(&rspeed_msg);
    left_motor.publish(&lspeed_msg);
    rpid_pub.publish(&rpid_msg);
    lpid_pub.publish(&lpid_msg);
    setpoint_pub.publish(&setpoint_msg);
    Lcounter = 0;
    Rcounter = 0;
    prev_time= new_time;



 nd.spinOnce();
delay(1);
  }



 

}
