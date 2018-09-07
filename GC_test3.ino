#include <PID_v1.h>
#include <MsTimer2.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

#define LONG_RANGE 
#define HIGH_SPEED
#define HIGH_ACCURACY

//类型定义
#define u8 unsigned char
#define u16 unsigned int

//常量定义8
//引脚
#define ENCONDER_A_PIN 2
#define LRS_LeftFront_PIN 4
#define LRS_RightFront_PIN 5
#define LRS_LeftRear_PIN 6
#define LRS_RightRear_PIN 7

#define SERVO_FRONT_PIN 9
#define SERVO_REAR_PIN 10 

//舵机角度
#define SERVO_FRONT_MIDDLE 62
#define SERVO_FRONT_LOWERLIMIT_1 58//小车往左转
#define SERVO_FRONT_UPPERLIMIT_1 66//小车往右转

#define SERVO_FRONT_LOWERLIMIT_2 60//小车往左转
#define SERVO_FRONT_UPPERLIMIT_2 64//小车往右转

#define SERVO_FRONT_LOWERLIMIT_3 55//小车往左转
#define SERVO_FRONT_UPPERLIMIT_3 67//小车往右转

#define SERVO_REAR_ENABLE 60
#define SERVO_REAR_DISABLE 71

//PID参数
#define PID_Servo_P 0.5
#define PID_Servo_I 0
#define PID_Servo_D 1

#define VELOCITY_SAMPLE_TIME 100 //速度采样时间
#define DIRECTION_SAMPLE_TIME 50 //方向采样时间


//全局变量定义
Servo servoFront;
Servo servoRear;
VL53L0X LRS_LeftFront;
VL53L0X LRS_LeftRear;
VL53L0X LRS_RightFront;
VL53L0X LRS_RightRear;

int Distance=0;
int Distance_LeftFront=0,Distance_RightFront=0;
int Distance_LeftRear=0,Distance_RightRear=0;
int Distance_LF_RF_1=0,Distance_LF_RF_2=0;
int Distance_LR_RR = 0;
int PosFront=0;

volatile u16 encoderPos=0;
u16 velocity=0;

//用于pid
unsigned long now=0;
unsigned long lastTime=0;
int setpoint=0;
double Err=0;
double lastErr=0;
double dErr=0;
double pid_output;

void goPID(int up,int low)
{
  if(Distance_LF_RF_1<100&&Distance_LF_RF_1>-100){
    PosFront=62;
    }
        
   else{
      now=millis();
      Err=setpoint-Distance_LF_RF_1;
      dErr=(Err-lastErr)/(now-lastTime);//误差微分
      pid_output=PID_Servo_P*Err+PID_Servo_D*dErr;//PID计算
      lastErr=Err;
      lastTime=now;
      PosFront=(int)(pid_output*18/500+62);//PID转换成方向角
  
      if(PosFront<low)
        PosFront=low; 
      else if(PosFront>up)
         PosFront=up;
   }
   Serial.println(PosFront);
}

//外部中断,编码器计数
void doEncoder(){
  encoderPos++;//测量速度时的脉冲采样
}

//速度函数
void dovelocity(){
  velocity=encoderPos;
  Distance=Distance+velocity;
  encoderPos=0;
}


void setup() {
  
  //初始化编码器
  pinMode(ENCONDER_A_PIN,INPUT);
  digitalWrite(ENCONDER_A_PIN,HIGH);//上拉
  attachInterrupt(0, doEncoder, FALLING);//引脚2 中断0 
  
  //初始化舵机
  servoFront.attach(SERVO_FRONT_PIN);
  servoRear.attach(SERVO_REAR_PIN);
  servoFront.write(SERVO_FRONT_MIDDLE);
  servoRear.write(SERVO_REAR_ENABLE);
  
  //初始化激光传感器
  pinMode(LRS_LeftFront_PIN, OUTPUT);
  pinMode(LRS_RightFront_PIN,OUTPUT);
  pinMode(LRS_LeftRear_PIN,  OUTPUT);
  pinMode(LRS_RightRear_PIN, OUTPUT);
  digitalWrite(LRS_LeftFront_PIN, LOW);
  digitalWrite(LRS_RightFront_PIN,LOW);
  digitalWrite(LRS_LeftRear_PIN,  LOW);
  digitalWrite(LRS_RightRear_PIN, LOW);

  Serial.begin(38400);
  Wire.begin();//IIC初始化 SDA:A4, SCL:A5

  pinMode(LRS_LeftFront_PIN, INPUT);
  LRS_LeftFront.init(true);
  LRS_LeftFront.setAddress((uint8_t)22);
  pinMode(LRS_RightFront_PIN, INPUT);
  LRS_RightFront.init(true);
  LRS_RightFront.setAddress((uint8_t)25);
  pinMode(LRS_LeftRear_PIN, INPUT);
  LRS_LeftRear.init(true);
  LRS_LeftRear.setAddress((uint8_t)28);
  pinMode(LRS_RightRear_PIN, INPUT);
  LRS_RightRear.init(true);
  LRS_RightRear.setAddress((uint8_t)31);
  
  Serial.println("addresses set");



  #if defined LONG_RANGE
  {
  LRS_LeftFront.setSignalRateLimit(0.1);
  LRS_LeftFront.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  LRS_LeftFront.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  LRS_RightFront.setSignalRateLimit(0.1);
  LRS_RightFront.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  LRS_RightFront.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  LRS_LeftRear.setSignalRateLimit(0.1);
  LRS_LeftRear.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  LRS_LeftRear.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  LRS_RightRear.setSignalRateLimit(0.1);
  LRS_RightRear.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  LRS_RightRear.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);}
#endif

#if defined HIGH_SPEED
  LRS_LeftFront.setMeasurementTimingBudget(20000);
  LRS_RightFront.setMeasurementTimingBudget(20000);
  LRS_LeftRear.setMeasurementTimingBudget(20000);
  LRS_RightRear.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  LRS_LeftFront.setMeasurementTimingBudget(200000);
  LRS_RightFront.setMeasurementTimingBudget(200000);
  LRS_LeftRear.setMeasurementTimingBudget(200000);
  LRS_RightRear.setMeasurementTimingBudget(200000);
#endif

  //初始化定时器,
  MsTimer2::set(VELOCITY_SAMPLE_TIME,dovelocity);//定时器2中断设置函数，方向采样，100ms
  MsTimer2::start(); //开始计时

 }


void loop()
{
  Distance_LeftFront=LRS_LeftFront.readRangeSingleMillimeters();
  Distance_RightFront=LRS_RightFront.readRangeSingleMillimeters(); 
  Distance_LeftRear=LRS_LeftRear.readRangeSingleMillimeters();
  Distance_RightRear=LRS_RightRear.readRangeSingleMillimeters();

  Distance_LR_RR=Distance_LeftRear-Distance_RightRear;
  Distance_LF_RF_1=Distance_LeftFront-Distance_RightFront; //左前-右前  

  //发车
  if(Distance<520)
  { 
      if(velocity>3)
        servoRear.write(SERVO_REAR_DISABLE);
      else
        servoRear.write(SERVO_REAR_ENABLE);

     goPID(SERVO_FRONT_UPPERLIMIT_1,SERVO_FRONT_LOWERLIMIT_1);
    }


   //冲坡
   else if(Distance<1450)
   {
    servoRear.write(SERVO_REAR_ENABLE);
    goPID(SERVO_FRONT_UPPERLIMIT_2,SERVO_FRONT_LOWERLIMIT_2);
   }

   //下坡
   else if(Distance<2800)
   {
      servoRear.write(SERVO_REAR_DISABLE);
      if(Distance_LR_RR>80)
    {
      PosFront=47;
      servoFront.write(PosFront);
      }
    else if(Distance_LR_RR<-80){
      PosFront=77;
      servoFront.write(PosFront);
      }
      goPID(SERVO_FRONT_UPPERLIMIT_1,SERVO_FRONT_LOWERLIMIT_1);
   }

   //避障
   else{
      if(velocity>3)
        servoRear.write(SERVO_REAR_DISABLE);
      else
        servoRear.write(SERVO_REAR_ENABLE);

    if(Distance_LR_RR>80)
    {
      PosFront=47;
      servoFront.write(PosFront);
      }
    else if(Distance_LR_RR<-80){
      PosFront=77;
      servoFront.write(PosFront);
      }
    else{
      goPID(SERVO_FRONT_UPPERLIMIT_3,SERVO_FRONT_LOWERLIMIT_3);
      }
   }
    servoFront.write(PosFront);
   
 

  Serial.println();
    Serial.print ("pid= ");
    Serial.println(pid_output);
    Serial.print ("pos= ");
    Serial.println(PosFront);
   
 // Serial.println(Distance);

}

