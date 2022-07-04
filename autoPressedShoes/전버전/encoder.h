int encoderPos = 0;
int motor_current_degree=0;

#define MOTOR_ENCODER_A 4// 모터 엔코더 A 핀번호
#define MOTOR_ENCODER_B 2// 모터 엔코더 B 핀번호

#define MOTOR_ENCODER_POS_PER_ROUND 2500.0 //(펄스) 최대 속도

//엔코더 함수 ///////////////////////////////////////////////////////////////////////////////////////////
void doEncoderA(){
  if(digitalRead(MOTOR_ENCODER_A)==digitalRead(MOTOR_ENCODER_B)) // 같으면
    encoderPos++; // 정회전
  else // 다르면
    encoderPos--; // 역회전
  //Serial.print("A   ");
  //Serial.println(encoderPos);
  motor_current_degree=(float)encoderPos/MOTOR_ENCODER_POS_PER_ROUND * 360;
}
void doEncoderB(){
  if(digitalRead(MOTOR_ENCODER_A)==digitalRead(MOTOR_ENCODER_B)) // 같으면
    encoderPos--; // 역회전
  else // 다르면
    encoderPos++; // 정회전
  //Serial.print("B   ");
  //Serial.println(encoderPos);
  motor_current_degree=(float)encoderPos/MOTOR_ENCODER_POS_PER_ROUND * 360;
}
//엔코더 함수 ///////////////////////////////////////////////////////////////////////////////////////////
