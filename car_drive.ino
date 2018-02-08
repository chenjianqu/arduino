//电机运行函数

void carForward(int n){
   digitalWrite(PIN_MOTOR_RIGHT_DIRECTION,LOW);
   digitalWrite(PIN_MOTOR_LEFT_DIRECTION,LOW);
   analogWrite(PIN_MOTOR_RIGHT_SPEED,n);
   analogWrite(PIN_MOTOR_LEFT_SPEED,n);
   Serial.print("docarForward:");
   Serial.println(n);
   delay(EACH_FT);
}
void carBack(int n){
   digitalWrite(PIN_MOTOR_RIGHT_DIRECTION,HIGH);
   digitalWrite(PIN_MOTOR_LEFT_DIRECTION,HIGH);
   analogWrite(PIN_MOTOR_RIGHT_SPEED,n);
   analogWrite(PIN_MOTOR_LEFT_SPEED,n);
   Serial.print("docarBack:");
   Serial.println(n);
   delay(EACH_BT);
}
void carLeft(int n){
   digitalWrite(PIN_MOTOR_RIGHT_DIRECTION,LOW);
   digitalWrite(PIN_MOTOR_LEFT_DIRECTION,HIGH);
   analogWrite(PIN_MOTOR_RIGHT_SPEED,n);
   analogWrite(PIN_MOTOR_LEFT_SPEED,n);
   Serial.print("docarLeft:");
   Serial.println(n);
   delay(EACH_LT);
}
void carRight(int n){
   digitalWrite(PIN_MOTOR_RIGHT_DIRECTION,HIGH);
   digitalWrite(PIN_MOTOR_LEFT_DIRECTION,LOW);
   analogWrite(PIN_MOTOR_RIGHT_SPEED,n);
   analogWrite(PIN_MOTOR_LEFT_SPEED,n);
   Serial.print("docarRight:");
   Serial.println(n);
   delay(EACH_RT);
}
void carStop(){
   digitalWrite(PIN_MOTOR_RIGHT_DIRECTION,LOW);
   digitalWrite(PIN_MOTOR_LEFT_DIRECTION,LOW);
   analogWrite(PIN_MOTOR_RIGHT_SPEED,0);
   analogWrite(PIN_MOTOR_LEFT_SPEED,0);
   Serial.println("docarStop");
}
void carForwardLeft(int leftS,int rightS){
  Serial.print("docarForwardLeft_rightS");
  Serial.println(rightS);
   digitalWrite(PIN_MOTOR_RIGHT_DIRECTION,LOW);
   digitalWrite(PIN_MOTOR_LEFT_DIRECTION,LOW);
   analogWrite(PIN_MOTOR_RIGHT_SPEED,rightS);
   analogWrite(PIN_MOTOR_LEFT_SPEED,leftS);
   delay(EACH_FT);
}
void carForwardRight(int leftS,int rightS){
  Serial.print("docarForwardRight_leftS");
  Serial.println(leftS);
   digitalWrite(PIN_MOTOR_RIGHT_DIRECTION,LOW);
   digitalWrite(PIN_MOTOR_LEFT_DIRECTION,LOW);
   analogWrite(PIN_MOTOR_RIGHT_SPEED,rightS);
   analogWrite(PIN_MOTOR_LEFT_SPEED,leftS);
   delay(EACH_FT);
}
