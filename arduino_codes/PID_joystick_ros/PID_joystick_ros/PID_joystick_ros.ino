#include <PID_v1.h>

#include<ros.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Float64.h>

#define Renc 2
#define Lenc 3 
#define Rpwm 6
#define Lpwm 5
 
#define Rdir 7
#define Ldir 8


std_msgs:: Float64 rspeed_msg;
std_msgs:: Float64 lspeed_msg;
std_msgs:: Float64 lpid_msg;
std_msgs:: Float64 rpid_msg;
std_msgs:: Float64 setpoint_msg;
geometry_msgs:: Twist cmd;

bool dir = 0;

std_msgs::Float64 &effort_msg
void callback (const geometry_msgs::Twist &cmd)
{
int s = cmd.linear.x;
if (s>0)dir=0;
else dir=1;
setpoint = map(abs(s),0,0.7,0,140);
  
  }

ros:: Subscriber<std_msgs::Float64> get_cmd("cmd_vel", &callback);



ros:: Publisher right_motor("/motor_speeds/right_motor", &rspeed_msg);
ros:: Publisher left_motor("/motor_speeds/left_motor",&lspeed_msg);
ros::Publisher setpoint_pub("/setpoint",& setpoint_msg);



ros:: NodeHandle nd;




volatile unsigned long Rcounter =0, Lcounter =0; int RRPM =0, LRPM =0;
int dt = 100; //double ppr = 3657; // 20 * 32  --> double  3657--> experiment 
double ppr = 20*32;
double setpoint, rinput, loutput, kp = 0.06, ki = 0.2,  kd = 0, 
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
pinMode(Renc,INPUT_PULLUP);pinMode(Lenc,INPUT_PULLUP);pinMode(Rpwm,OUTPUT);pinMode(Lpwm,OUTPUT);

attachInterrupt(digitalPinToInterrupt(2),rencoder,CHANGE);
attachInterrupt(digitalPinToInterrupt(3),rencoder,CHANGE);

//Serial.begin(57600);


 right_mtr_ctrl.SetMode(AUTOMATIC);
 left_mtr_ctrl.SetMode(AUTOMATIC);

nd.initNode();
nd.advertise(right_motor);
nd.advertise(left_motor);

nd.advertise(setpoint_pub);
 nd.subscribe(get_cmd);


}

void loop() {
  // put your main code here, to run repeatedly:


  
 if(dir)
 {digitalWrite(Ldir,0); digitalWrite(Rdir,0);}
 else {digitalWrite(Ldir,1); digitalWrite(Rdir,1);}
 
new_time = millis();
if (new_time-prev_time >= dt)
{
  RRPM = Rcounter / ppr / (dt /1000.0) * 60.0;
  LRPM = Rcounter / ppr / (dt /1000.0) * 60.0;


  rinput = RRPM; linput = LRPM;

right_mtr_ctrl.Compute();
 
 left_mtr_ctrl.Compute();
 
analogWrite(Rpwm,routput);
analogWrite(Lpwm,loutput);
  /* Serial.print("setpoint is ");
     Serial.print(setpoint);
    Serial.print(" speed is:   ");
     Serial.print(RRPM);
      Serial.print(" pid is:   ");
       Serial.println(routput);
  Serial.print(" left pid is:   ");
       Serial.println(loutput);*/
  
rspeed_msg.data = RRPM;
lspeed_msg.data = LRPM;
setpoint_msg.data = setpoint;
rpid_msg.data = routput;
lpid_msg.data = loutput;

right_motor.publish(&rspeed_msg);
left_motor.publish(&lspeed_msg);

setpoint_pub.publish(&setpoint_msg);
Lcounter =0;
Rcounter = 0;

  
  }
nd.spinOnce();




}
