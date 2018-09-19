#include <PID_v1.h>
#include <MsTimer2.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>
#include<PID_v1.h>

#define LONG_RANGE 
#define HIGH_SPEED
#define HIGH_ACCURACY

//类型定义
#define u8 unsigned char
#define u16 unsigned int

//引脚
#define ENCONDER_A_PIN 2 //编码器
#define LRS_LeftFront_PIN 4 //四个激光传感器
#define LRS_RightFront_PIN 5
#define LRS_LeftRear_PIN 6
#define LRS_RightRear_PIN 7
#define SERVO_FRONT_PIN 9//舵机引脚
#define SERVO_REAR_PIN 10

//舵机角度
#define SERVO_FRONT_MIDDLE 50 //直行角度

//PID输出角度极值
#define SERVO_FRONT_LOWERLIMIT_1 -5.0  //小车往左转
#define SERVO_FRONT_UPPERLIMIT_1 5.0   //小车往右转
#define SERVO_FRONT_LOWERLIMIT_2 -3.0
#define SERVO_FRONT_UPPERLIMIT_2 3.0
#define SERVO_FRONT_LOWERLIMIT_3 -10.0
#define SERVO_FRONT_UPPERLIMIT_3 10.0

//后舵机角度
#define SERVO_REAR_ENABLE 60
#define SERVO_REAR_DISABLE 71

//转弯角度
#define SERVO_LEFT SERVO_FRONT_MIDDLE-15
#define SERVO_RIGHT SERVO_FRONT_MIDDLE+15

//采样时间
#define VELOCITY_SAMPLE_TIME 100 //速度采样时间
#define DIRECTION_SAMPLE_TIME 50 //方向采样时间

//转弯参数
#define CURVE_DISTANCE 150 //转弯状态持续距离
#define CURVE_THRESHOLD 1200 //转弯状态触发阈值
#define ANGLE_TURNLEFT SERVO_FRONT_MIDDLE-28 //左转弯
#define ANGLE_TURNRIGHT SERVO_FRONT_MIDDLE+28 //右转弯

//距离参数
#define DISTANCE_START 500
#define DISTANCE_UPHILL 1600

//刹车阈值
#define VELOCITY_LIMITED 3

//是否刹车
#define BRAKE(V) V>VELOCITY_LIMITED?servoRear.write(SERVO_REAR_DISABLE):servoRear.write(SERVO_REAR_ENABLE)

//全局变量定义
Servo servoFront;
Servo servoRear;
VL53L0X LRS_LeftFront;
VL53L0X LRS_LeftRear;
VL53L0X LRS_RightFront;
VL53L0X LRS_RightRear;

//pid参数
float Kp = 0.005;
float Ki = 0.01;
float Kd = 0.1;
double Setpoint, Input, Output, ServoOutput; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int Distance_LeftFront=0,Distance_RightFront=0;
int Distance_LeftRear=0,Distance_RightRear=0;
int Distance_LF_RF=0;
int Distance_LR_RR=0;
int PosFront=0;

int Distance=0;//持续测量的距离
int singleDis=0;//每圈的距离
volatile u16 encoderPos=0;
u16 velocity=0;//速度
int dis=-1;//转弯距离兼标志位
u8 curve_i=0;//转弯个数

int dis1=-1;

//外部中断,编码器计数
void doEncoder()
{
  encoderPos++;//测量速度时的脉冲采样
}

//速度函数
void dovelocity()
{
  velocity=encoderPos;
  Distance=Distance+velocity;
  singleDis+=velocity;
  encoderPos=0;
}

double getPID(double low,double up)
{
    myPID.SetOutputLimits(low,up);
    Input = Distance_LF_RF;
    myPID.Compute();
    return Output;
}

void initLRS();


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
  Serial.begin(38400);
  //初始化激光传感器
  initLRS();
//初始化定时器,
  MsTimer2::set(VELOCITY_SAMPLE_TIME,dovelocity);//定时器2中断设置函数，方向采样，100ms
  MsTimer2::start(); //开始计时
  //初始化pid
  myPID.SetMode(AUTOMATIC);//自动计算
  myPID.SetSampleTime(300);//设置采样时间
  //发车位置不在转弯处
  singleDis=200;
}


void loop()
{
  Distance_LeftFront=LRS_LeftFront.readRangeSingleMillimeters();
  Distance_RightFront=LRS_RightFront.readRangeSingleMillimeters(); 
  Distance_LeftRear=LRS_LeftRear.readRangeSingleMillimeters();
  Distance_RightRear=LRS_RightRear.readRangeSingleMillimeters();

  Distance_LR_RR=Distance_LeftRear-Distance_RightRear;
  Distance_LF_RF=Distance_LeftFront-Distance_RightFront; //左前-右前  

  
  
  //出现拐弯
  if(Distance_LeftFront>CURVE_THRESHOLD&&dis1==-1)
  {
    dis=Distance+CURVE_DISTANCE;
    dis1=Distance+2000;
    curve_i++;
    if(curve_i%2==0)singleDis=0;//单圈距离重置
  }

  if(Distance>dis1&&dis1!=-1)
    dis1=-1;

  //转弯状态
  if(dis!=-1)
  {
    BRAKE(velocity);
    if(dis>Distance){
      PosFront=ANGLE_TURNLEFT;
      servoFront.write(PosFront);
    }
    else
        dis=-1;
    Serial.println("CURVE***");
  }
  else
  {//非转弯
    
    if(curve_i%2==0)//当小车起点为坡底时,curve_i为偶数表示为有坡的半圈
    {//上坡半圈
      if(singleDis<DISTANCE_START)//发车
      { 
         BRAKE(velocity);//速度限制
         PosFront=SERVO_FRONT_MIDDLE+getPID(SERVO_FRONT_LOWERLIMIT_1,SERVO_FRONT_UPPERLIMIT_1);
       }
       else if(Distance<DISTANCE_UPHILL)//冲坡
       {
          servoRear.write(SERVO_REAR_ENABLE);//后舵机刹车
          PosFront=SERVO_FRONT_MIDDLE+getPID(SERVO_FRONT_LOWERLIMIT_2,SERVO_FRONT_UPPERLIMIT_2);
       }
       else//下坡
       {
          BRAKE(velocity);
          PosFront=SERVO_FRONT_MIDDLE+getPID(SERVO_FRONT_LOWERLIMIT_1,SERVO_FRONT_UPPERLIMIT_1);
       }
       Serial.println("FIRST***");
    }//上坡半圈
    
    //避障半圈
    else
    
    {
      Serial.println("SENCOND***");
      BRAKE(velocity);  
      if(Distance_LR_RR>80)
        PosFront=SERVO_LEFT;
      else if(Distance_LR_RR<-80)
        PosFront=SERVO_RIGHT;
      else
        PosFront=SERVO_FRONT_MIDDLE+getPID(SERVO_FRONT_LOWERLIMIT_3,SERVO_FRONT_UPPERLIMIT_3);
    }//避障半圈
    
    servoFront.write(PosFront);
    
  }//非转弯
  
  Serial.print("LF:");
  Serial.println(Distance_LeftFront);
  Serial.print("RF:");
  Serial.println(Distance_RightFront);
  Serial.print("LR:");
  Serial.println(Distance_LeftRear);
  Serial.print("RR:");
  Serial.println(Distance_RightRear);
  Serial.print("dis:");
  Serial.println(dis);
  Serial.print("dis1:");
  Serial.println(dis1);
  Serial.print("Posfront:");
  Serial.println(PosFront);
  Serial.print("distance:");
  Serial.println(Distance);
  Serial.print("velocity:");
  Serial.println(velocity);
}


void initLRS()
{
//初始化激光传感器
  pinMode(LRS_LeftFront_PIN, OUTPUT);
  pinMode(LRS_RightFront_PIN,OUTPUT);
  pinMode(LRS_LeftRear_PIN,  OUTPUT);
  pinMode(LRS_RightRear_PIN, OUTPUT);
  digitalWrite(LRS_LeftFront_PIN, LOW);
  digitalWrite(LRS_RightFront_PIN,LOW);
  digitalWrite(LRS_LeftRear_PIN,  LOW);
  digitalWrite(LRS_RightRear_PIN, LOW);

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
}

