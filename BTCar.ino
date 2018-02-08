#include<Servo.h>

/*在电路驱动板的情况下 4、7针脚用HIGH和LOW分别是M1、M2的的前进和后退 5,6针脚用PWM控制M1,M2的速度*/

/*********************************************预处理命令****************************************/
//管脚定义
#define PIN_MOTOR_RIGHT_DIRECTION 4
#define PIN_MOTOR_RIGHT_SPEED 5
#define PIN_MOTOR_LEFT_DIRECTION 7
#define PIN_MOTOR_LEFT_SPEED 6
#define PIN_US_TRIG 8
#define PIN_US_ECHO 9
#define PIN_SERVO_PANTILT_BUTTON 2
#define PIN_SERVO_PANTILT_TOP 3
#define PIN_SERVO_HOLDER_HAND 13
#define PIN_SERVO_HOLDER_TOP 12
#define PIN_SERVO_HOLDER_MIDDLE 11
#define PIN_SERVO_HOLDER_BUTTON 10

//舵机默认角度
#define ANGLE_PANTILT_BUTTON 90
#define ANGLE_PANTILT_TOP 35
#define ANGLE_HOLDER_HAND 5
#define ANGLE_HOLDER_TOP 75
#define ANGLE_HOLDER_MIDDLE 30
#define ANGLE_HOLDER_BUTTON 0
#define DEFAULT_SPEED 50//电机默认速度

/*********************************************常量定义****************************************/
//手动控制模式下每按一次速度、角度的该变量
const int ST=4,AT=4;
const int EACH_FT=30,EACH_BT=50,EACH_RT=100,EACH_LT=100;
//速度的最大值
const int MAX_SPEED=150;
//自动导航最小感应距离
const int MIN_DISTANCE=10;
//遥控器键值
const char KEY_CH_AUTO='a',KEY_CH_MANUAL='b',KEY_CH_RESET='c',KEY_CH_GRAVITY='d';
const char KEY_LEFT='e',KEY_RIGHT='f',KEY_STOP='g',KEY_FORWARD='h',KEY_BACK='i',KEY_GRAB='j',KEY_ADD='k',KEY_SUB='l';
const char KEY_HAND='o',KEY_ARMTS='p',KEY_ARMMS='q',KEY_ARMBS='r',KEY_PTTS='s',KEY_PTBS='t';
const char KEY_NUM_START='m',KEY_NUM_END='n';

/*********************************************变量定义****************************************/
Servo ptButtonServo,holderHandServo,holderTopServo,holderMiddleServo,holderButtonServo,ptTopServo;

//当前命令，一级命令，二级命令
char code='z',channel='z',dowhat='z',speedMode='z';

//停止键收到的次数
int stopAutoNavigation=0;
int stopGravity=0;

//用于记录电机速度和舵机角度
int autoNavigationSpeed=DEFAULT_SPEED,manualModeSpeed=DEFAULT_SPEED,gravityModeSpeed=DEFAULT_SPEED;
int ptButtonServoAngle=ANGLE_PANTILT_BUTTON,ptTopServoAngle=ANGLE_PANTILT_TOP,holderHandServoAngle=ANGLE_HOLDER_HAND,holderTopServoAngle=ANGLE_HOLDER_TOP,holderButtonServoAngle=ANGLE_HOLDER_BUTTON,holderMiddleServoAngle=ANGLE_HOLDER_MIDDLE;
int s=0;
int number=0;

/*********************************************初始化函数****************************************/
void setup() {
  pinMode(PIN_MOTOR_RIGHT_DIRECTION,OUTPUT);
  pinMode(PIN_MOTOR_LEFT_DIRECTION,OUTPUT);
  pinMode(PIN_US_TRIG,OUTPUT);
  pinMode(PIN_US_ECHO,INPUT);
  
  ptButtonServo.attach(PIN_SERVO_PANTILT_BUTTON);
  ptTopServo.attach(PIN_SERVO_PANTILT_TOP);
  holderHandServo.attach(PIN_SERVO_HOLDER_HAND);
  holderTopServo.attach(PIN_SERVO_HOLDER_TOP);
  holderMiddleServo.attach(PIN_SERVO_HOLDER_MIDDLE);
  holderButtonServo.attach(PIN_SERVO_HOLDER_BUTTON);
  
  Serial.begin(9600);
  holderInitialize();
}

/*********************************************LOOP函数****************************************/
void loop() {
  remoteMode();
}



/***********************************通过超声波传感器获得与障碍物距离********************************/
int getDistance(){
  digitalWrite(PIN_US_TRIG,LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG,LOW);
/*发一个10ms的高脉冲就能触发TrigPin，该模块被触发后，超声波发射头将发出8个40kHz周期电平，同时检测回波。一旦检测到有回波信号则输出回响信号。
回响信号是一个脉冲的宽度成正比的距离对象。可通过发射信号到收到的回响信号时间间隔可以计算得到距离。公式: uS/58=厘米；或者uS/148=英寸。*/
  float distanceTime=pulseIn(PIN_US_ECHO,HIGH);//接收高电平时间
  distanceTime=distanceTime/58.0;
  return distanceTime;
}

/*****************************************自动夹持东西*************************************/
void Hold(){
  Serial.println("doHold");
  int angle=0;
  holderHandServo.write(ANGLE_HOLDER_HAND);
  holderTopServo.write(ANGLE_HOLDER_TOP);
  holderButtonServo.write(ANGLE_HOLDER_BUTTON);
  holderMiddleServo.write(ANGLE_HOLDER_MIDDLE);
  delay(300);
  holderMiddleServo.write(65);
  delay(300);
  holderHandServo.write(150);
  delay(500);
  for(int i=150;i>ANGLE_HOLDER_HAND;i-=4){
   holderHandServo.write(angle);
   delay(20); 
  }
  holderMiddleServo.write(ANGLE_HOLDER_MIDDLE);
  delay(500);
}

/*****************************************初始化舵机角度*************************************/
void holderInitialize(){
  Serial.println("doInitial");
  holderHandServo.write(ANGLE_HOLDER_HAND);
  holderTopServo.write(ANGLE_HOLDER_TOP);
  holderButtonServo.write(ANGLE_HOLDER_BUTTON);
  holderMiddleServo.write(ANGLE_HOLDER_MIDDLE);
  ptButtonServo.write(ANGLE_PANTILT_BUTTON);
  ptTopServo.write(ANGLE_PANTILT_TOP);
  delay(300);
}
