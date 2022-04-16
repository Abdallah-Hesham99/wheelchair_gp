#include <ros.h>
#include<geometry_msgs/Twist.h>
//#include<std_msgs/Float64.h>
#include<std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>

#define rpwm 6
#define lpwm 5
#define hall_1a 2
#define hall_1b 7
#define hall_2a 3
#define hall_2b 8
#define dir1 9
#define dir2 11

double lsetpoint=0, linput=0, loutput=0, kp = 120, kd = 0, ki = 10;
double asetpoint=0, ainput=0, aoutput=0; volatile long rcounter = 0; volatile long lcounter=0;
unsigned long rRPM,lRPM;
float dt;/*
float ratio = 0.0024;
float ppr = 20*32;
float dist_per_tick = (3.14*0.3)/ppr;*/
unsigned long prev_time = 0, cur_time = 0;



bool rstate = 0;
bool lstate = 0;
void cmd_callback(const geometry_msgs::Twist&cmd_msg)
{/*
    if(cmd_msg.linear.x<0)
    {rstate=0; lstate=0;}
    else {rstate=1; lstate=1;}
*/
    lsetpoint = cmd_msg.linear.x;
    
    asetpoint = cmd_msg.angular.z;
  
    
}




void state_callback(const geometry_msgs::Twist&state_msg)
{


    linput = state_msg.linear.x;
    ainput = state_msg.angular.z;
   
    
    
}


std_msgs::Int16 pid_msg;
ros::Publisher pid_publisher("pid_output", &pid_msg );
//ros::Publisher speed_publisher("Current_speed", &sped_msg);*/
ros:: Subscriber<geometry_msgs::Twist> get_cmd("cmd_vel", &cmd_callback);
ros:: Subscriber<geometry_msgs::Twist> get_state("control/current_speed", &state_callback);


std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("/wheel_encoders/right_ticks", &right_wheel_tick_count );

std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("/wheel_encoders/left_ticks", &left_wheel_tick_count );







ros:: NodeHandle nd;




PID linear_control(&linput, &loutput, &lsetpoint, kp, ki, kd, DIRECT);
PID angular_control(&ainput, &aoutput, &asetpoint, kp, ki, kd, DIRECT);

void ISRA()
{
  int val= digitalRead(hall_1b);
  if(val)
  {
    rcounter++;
    }
   else{
   rcounter--;}
}
void ISRB()
{
  int val= digitalRead(hall_2b);
  if(val)
  {
    lcounter++;
    }
   else{
   lcounter--;}
}






void setup() {
  // put your setup code here, to run once:
  pinMode(rpwm, OUTPUT);
  pinMode(lpwm, OUTPUT);
  pinMode(hall_1a, INPUT_PULLUP);
  pinMode(hall_1b, INPUT_PULLUP);
  pinMode(hall_2a, INPUT_PULLUP);
  pinMode(hall_2b, INPUT_PULLUP);
  
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(hall_1a), ISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(hall_2a), ISRB, RISING);
  prev_time = millis();
  dt = 200.0;
  linear_control.SetMode(AUTOMATIC);
  linear_control.SetOutputLimits(-125,125);
  
  angular_control.SetMode(AUTOMATIC);
  angular_control.SetOutputLimits(-125,125);
//  setpoint = 80.0;
  //Serial.begin(9600);

  //analogWrite(pwm, 150);
  //Serial.print("start....");
 // delay(1000);
  linear_control.SetSampleTime(200);
  angular_control.SetSampleTime(200);
  nd.initNode();
  nd.advertise(rightPub);
  nd.advertise(leftPub);
  nd.advertise(pid_publisher);
  nd.subscribe(get_cmd);
  nd.subscribe(get_state);
  

}

void loop() {
  // put your main code here, to run repeatedly:

  cur_time = millis();
  if ((cur_time - prev_time) > dt)
  {
    
    
    linear_control.Compute();
    angular_control.Compute();

    int vr= loutput + aoutput * (0.15 + 0.605);
    if (vr>=0){
      vr = vr>255? 255:vr;
      
      digitalWrite(dir1,1);}
      else {
        vr = vr<-255? -255:vr;
        digitalWrite(dir1,0);
        vr = abs(vr);
        
      }
    
    //vr = vr>255? 255:vr;
    int vl= loutput + aoutput * (0.15 - 0.605);

if (vl>=0){
      vl = vl>255? 255:vl;
      
      digitalWrite(dir2,1);}
      else {
        vl = vl<-255? -255:vl;
        digitalWrite(dir2,0);
        vl = abs(vl);
        
      }
    
    
    analogWrite(rpwm, vr);
    analogWrite(lpwm, vl);
  pid_msg.data = loutput + aoutput;

    right_wheel_tick_count.data = rcounter;
    left_wheel_tick_count.data = lcounter;
   
    delay(3);
    // Serial.print(counter);
    //Serial.print(" RPM is:   ");
    // Serial.print(RPM);
    //  Serial.print(" pid is:   ");
    //   Serial.println(output);
    prev_time = cur_time;
    rcounter = 0;
    lcounter = 0;


    // if (Serial.read()=='1')
    //
    //{
    //  setpoint=setpoint + 50;
    //  if (setpoint >= 350)
    //{setpoint = 70;}
    rightPub.publish(&right_wheel_tick_count);
    leftPub.publish(&left_wheel_tick_count );
    
    pid_publisher.publish(&pid_msg );

    nd.spinOnce();

  }





}
