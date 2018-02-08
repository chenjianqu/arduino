/****************************此文件中包含自动导航模式、手动操作模式、重力感应模式、复位模式的代码********************************/

/******************************自动导航模式******************************************/
void autoMode(){//自动导航模式,可用四个功能：速度加、减，自动抓取，停止/继续
  Serial.print("doAuto");
  Serial.println(autoNavigationSpeed);
  switch(code){
    case KEY_ADD:
      autoNavigationSpeed=autoNavigationSpeed+ST;
  Serial.println(autoNavigationSpeed);
      break;
    case KEY_SUB:
      autoNavigationSpeed=autoNavigationSpeed-ST;
  Serial.println(autoNavigationSpeed);
      break;
    case KEY_STOP:
      stopAutoNavigation++;//用正负数来判断是暂停还是继续
      break;
    default:
      break;
  }
  if(stopAutoNavigation%2==1)
      carStop();
  else 
      doAutoNavigation(autoNavigationSpeed);
}

/******************************手动操作模式******************************************/
void manualMode(){
  Serial.println("doManual");
  if(code==KEY_FORWARD||code==KEY_BACK||code==KEY_STOP||code==KEY_LEFT||code==KEY_RIGHT||code==KEY_GRAB||code==KEY_HAND||code==KEY_ARMTS||code==KEY_ARMMS||code==KEY_ARMBS||code==KEY_PTTS||code==KEY_PTBS)dowhat=code;//二级命令
  switch(dowhat){
    case KEY_FORWARD:
      if(code==KEY_ADD)manualModeSpeed+=ST;
      else if(code==KEY_SUB)manualModeSpeed-=ST;
      carForward(manualModeSpeed);
      break;
    case KEY_BACK:
      if(code==KEY_ADD)manualModeSpeed+=ST;
      else if(code==KEY_SUB)manualModeSpeed-=ST;
      if(code==KEY_BACK)carForward(manualModeSpeed);
      break;
    case KEY_LEFT:
      if(code==KEY_ADD)manualModeSpeed+=ST;
      else if(code==KEY_SUB)manualModeSpeed-=ST;
      if(code==KEY_LEFT)carForward(manualModeSpeed);
      break;
    case KEY_RIGHT:
      if(code==KEY_ADD)manualModeSpeed+=ST;
      else if(code==KEY_SUB)manualModeSpeed-=ST;
      if(code==KEY_RIGHT)carForward(manualModeSpeed);
      break;
    case KEY_STOP:
      carStop();
      break;
    case KEY_GRAB:
      if(code==KEY_GRAB)Hold();
      break;
    case KEY_HAND:
      if(code==KEY_ADD)holderHandServoAngle+=AT;
      else if(code==KEY_SUB)holderHandServoAngle-=AT;
      holderHandServo.write(holderHandServoAngle);
      Serial.print("Hand:");
      Serial.println(holderHandServoAngle);
      break;
    case KEY_ARMTS:
      if(code==KEY_ADD)holderTopServoAngle+=AT;
      else if(code==KEY_SUB)holderTopServoAngle-=AT;
      holderTopServo.write(holderTopServoAngle);
      Serial.print("ARMTS:");
      Serial.println(holderTopServoAngle);
      break;
    case KEY_ARMMS:
      if(code==KEY_ADD)holderMiddleServoAngle+=AT;
      else if(code==KEY_SUB)holderMiddleServoAngle-=AT;
      holderMiddleServo.write(holderMiddleServoAngle);
      Serial.print("ARMMS:");
      Serial.println(holderMiddleServoAngle);
      break;
    case KEY_ARMBS:
      if(code==KEY_ADD)holderButtonServoAngle+=AT;
      else if(code==KEY_SUB)holderButtonServoAngle-=AT;
      holderButtonServo.write(holderButtonServoAngle);
      Serial.print("ARMBS:");
      Serial.println(holderButtonServoAngle);
      break;
    case KEY_PTTS:
      if(code==KEY_ADD)ptTopServoAngle+=AT;
      else if(code==KEY_SUB)ptTopServoAngle-=AT;
      ptTopServo.write(ptTopServoAngle);
      Serial.print("PTTS:");
      Serial.println(ptTopServoAngle);
      break;
    case KEY_PTBS:
      if(code==KEY_ADD)ptButtonServoAngle+=AT;
      else if(code==KEY_SUB)ptButtonServoAngle-=AT;
      ptButtonServo.write(ptButtonServoAngle);
      Serial.print("PTBS:");
      Serial.println(ptButtonServoAngle);
      break;
  } 
}

/******************************重力感应模式******************************************/
int speed_gravity=0;
void gravityMode(){
  Serial.print("doGrivity:");
  Serial.println(gravityModeSpeed);
  if(code==KEY_STOP)stopGravity++;
  speed_gravity=2*abs(gravityModeSpeed-50);
  if(stopGravity%2==1)carStop();
  else{
    if(gravityModeSpeed<50)carForwardLeft(DEFAULT_SPEED,speed_gravity);
    else if(gravityModeSpeed>50)carForwardRight(speed_gravity,DEFAULT_SPEED);
    else carForward(DEFAULT_SPEED);
  }
}

/******************************复位模式******************************************/
void resetMode(){//复位模式
      holderInitialize();
      carStop();
      dowhat=0;
      stopAutoNavigation=0;//暂停功能复位
      manualModeSpeed=DEFAULT_SPEED;//手动模式速度复位
      Serial.println("doReset");
}
