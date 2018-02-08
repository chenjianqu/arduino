/**************************接收蓝牙字符指令，进行进入相应的模式*******************************/
void remoteMode(){
  code='z';
  s=0;
  while(Serial.available()){
    code=Serial.read();
    if('0'<=code&&code<='9')
    {
      while('0'<=code&&code<='9')
      {
        s=s*10+(code-'0');
        Serial.flush();
        code=Serial.read();
       }
       
      if(channel==KEY_CH_AUTO)
          autoNavigationSpeed=s;
      else if(channel==KEY_CH_MANUAL){
        switch(dowhat){
          case KEY_HAND:
            holderHandServoAngle=s;
            break;
          case KEY_ARMTS:
            holderTopServoAngle=s;
            break;
          case KEY_ARMMS:
            holderMiddleServoAngle=s;
            break;
          case KEY_ARMBS:
            holderButtonServoAngle=s;
            break;
          case KEY_PTTS:
            ptTopServoAngle=s;
            break;
          case KEY_PTBS:
            ptButtonServoAngle=s;
            break;
          default:
            manualModeSpeed=s;
            break;
          }
        }
        else if(channel==KEY_CH_GRAVITY)
            gravityModeSpeed=s;
      }
    }
    if(channel=='z'||code==KEY_CH_AUTO||code==KEY_CH_RESET||code==KEY_CH_MANUAL||code==KEY_CH_GRAVITY){//收到一级命令
      channel=code;//一级
      holderInitialize();
      carStop();
    }
    switch(channel){
      case KEY_CH_AUTO:
        autoMode();
        break;
      case KEY_CH_RESET:
        resetMode();
        break;
       case KEY_CH_MANUAL:
        manualMode();
        break;
       case KEY_CH_GRAVITY:
        gravityMode();
        break;
    }
}
