#include <Wire.h>
#include "pinHeader.h"
#include "encoder.h"

void DataFetch_ISEN_P10K(void);
void RawToDecimal_ISEN_P10k(void);
void Calculate_ISEN_P10k(void);

unsigned long press_decimal,temp_decimal;
float pressure,temperature;
unsigned char dat[4];

void motorWork(){
  if(target_pressure-current_pressure!=0 && target_pressure!=0){
    if(target_pressure-current_pressure>0){
      digitalWrite(MOTOR_DIR,HIGH);
      Serial.print(" motor HIGH ");
      Serial.println(" m1");
    }else{
      digitalWrite(MOTOR_DIR,LOW);   
      Serial.print(" motor HIGH ");
      Serial.println(" m0");
    }
    Serial.print("pressure : ");
    Serial.print(target_pressure);
    Serial.print(" ");
    Serial.print(current_pressure);
    Serial.print(" ");
    Serial.print(motor_current_degree);
    Serial.println(" ");  

    if(motor_current_degree < 10 && target_pressure-current_pressure<=0){
      float motorspeed=0;
      Serial.print("motorspeed : ");
      Serial.print(motorspeed);
      Serial.println(" ");

      analogWrite(MOTOR_PWM,motorspeed);
      return;
    }   
    float motorspeed =  mP * abs(target_pressure - current_pressure) + mI * pressure_integ + mD * pressure_diff ;
      /*
    if(motor_current_degree>60 && (target_pressure - current_pressure)>0){
      float motorspeed_gain= 2 * (motor_current_degree-60);
      motorspeed= motorspeed+motorspeed_gain;
    }*/
    Serial.print("motorspeed : ");
    Serial.print(motorspeed);
    Serial.println(" ");
    analogWrite(MOTOR_PWM,motorspeed);
  }
}
void setup() {
  // put your setup code here, to run once:
  Wire.begin(); // I2C 초기화(Master Mode)
  Serial.begin(115200); // UART 초기화 9600bps

  pinMode(13,OUTPUT);
  pinMode(MOTOR_DIR,OUTPUT);
  pinMode(MOTOR_PWM,OUTPUT);
  pinMode(MOTOR_ENCODER_A , INPUT_PULLUP);
  pinMode(MOTOR_ENCODER_B , INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_A), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_B), doEncoderB, CHANGE);
  Serial.println("startup the autoPressing");
  Serial.println("checking lowPressure");
  delay(1000);
  lowPressure=0;
  for (int i=0;i<10;i++){
    DataFetch_ISEN_P10K();
    RawToDecimal_ISEN_P10k();
    Calculate_ISEN_P10k();
    lowPressure+=press_decimal;
    delay(100);
  }
  lowPressure=lowPressure/10;
  Serial.println("checking highPressure");
  digitalWrite(MOTOR_DIR,HIGH);
  float motorspeed = 200; 
  analogWrite(MOTOR_PWM,motorspeed);
  int tmp=0;
  while(1){
    unsigned long mfirst = millis();
    DataFetch_ISEN_P10K();
    RawToDecimal_ISEN_P10k();
    Calculate_ISEN_P10k();
    if(press_decimal==highPressure&&tmp>100) break;
    highPressure=press_decimal;
    unsigned long msecond= millis();
    unsigned long code_time = msecond-mfirst;
    tmp++;
    delay(LOOP_DELAY-code_time);
  }
  Serial.println("checking complete");
  tmp=0;
  while(1){
    unsigned long mfirst = millis();
    DataFetch_ISEN_P10K();
    RawToDecimal_ISEN_P10k();
    Calculate_ISEN_P10k();
    current_pressure=press_decimal;
    motorWork();
    target_pressure=lowPressure;
    if(press_decimal==lowPressure&&tmp>20) break;
    unsigned long msecond= millis();
    unsigned long code_time = msecond-mfirst;
    tmp++;
    delay(LOOP_DELAY-code_time);
  }
  
  Serial.println("start autopressing");
}
void checkingPressureSensor(int val){
  if(val>=SENSOR_CRITICAL_VALUE){
    if(preMillis!=0){
      unsigned long millidelay = millis()-preMillis;
      if(millidelay>10000){
        Serial.println("걸음걸이에 오류가 있습니다.");
      }else{
        distance_walking=millis()-preMillis;
        Serial.print("걸음걸이 간격 시간은 ");
        Serial.println(distance_walking);
        walking_percentage=distance_walking;
      }
    }
    sensorDelay=20;
    preMillis=millis();
  }
}

//루프 함수 ///////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  unsigned long mfirst = millis();
  if(distance_walking!=(-1)){
    float percentage = (float)walking_percentage/(float)distance_walking*100.0;
    Serial.print("현재 걸음걸이 상태는 ");
    Serial.print(100.0-percentage);
    Serial.println("%%입니다.");
    if(first_pressure==(-1)&&(100.0-percentage)==0){
      first_pressure=press_decimal;
    }
    walking_percentage-=LOOP_DELAY;
    if(walking_percentage<=-5000){

      Serial.println("걸음걸이가 멈췄습니다.");
      Serial.println("초기상태로 돌아갑니다.");

      distance_walking=(-1);
      walking_percentage=(-1);
      preMillis=0;
    }
    if((100.0-percentage)>80){
      //조여주기
      target_pressure=highPressure;

      Serial.println("target_pressure=3150;");

    }else if((100.0-percentage)>=20){
      //풀어주기
      target_pressure=lowPressure;
      Serial.println("target_pressure=2970;");

    }
  }
  int val = 1023- analogRead(A0);
  Serial.print(" Sensor : ");
  Serial.print(val);
  if(sensorDelay<=0){
    checkingPressureSensor(val);
  }else{
    sensorDelay--;
    if(sensorDelay<0) sensorDelay=0;
  }
  motorWork();
  DataFetch_ISEN_P10K();
  RawToDecimal_ISEN_P10k();
  Calculate_ISEN_P10k();
  
  current_pressure=press_decimal;
  if(previous_pressure!=0)
    pressure_diff = (float)(previous_pressure-current_pressure)/(float)LOOP_DELAY;
  if(pressure_Integral)
    pressure_integ += (float)(target_pressure-current_pressure)*(float)LOOP_DELAY;
  previous_pressure=current_pressure;
  unsigned long msecond= millis();
  unsigned long code_time = msecond-mfirst;
  delay(LOOP_DELAY-code_time);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// 압력센서 모듈에서 압력 및 온도 Raw Data를 Fetch
void DataFetch_ISEN_P10K(void){
  int i=0;
  /****************************************************************************
  * 압력센서는 특별한 데이타 요청이 없을 때 소모전력을 최소화 하기 위해서 Sleep 상태로 들어감.
  * 센서를 Wake up 하기 위해서는 두 가지 명령이 존재. 
  * * 1. Read MR -> slave address 0x28 + I2C read command bit를 적용하여 Wake up
  * --> 압력 및 온도 데이터를 Read할때 마다 update 됨.
  * 2. Write MR -> slave address 0x28 + I2C write command bit를 적용햐여 Wake up
  * --> 압력 값만 Update 되고 온도 값은 이전 값을 유지함.
  * 압력센서 Application 에 따라 Wake up 명령을 적절해 사용할 필요 있음.
  ****************************************************************************/
  Wire.write(P10K_i2c_address+0x01); // I2C Read_MR
  //Wire.write(P10K_i2c_address); // I2C Write_MR
  Wire.endTransmission();
  delay(10);
  Wire.requestFrom(0x28,4);
  while (Wire.available()){
    if(i>=4) i=0;
    dat[i] = Wire.read();
    i++;
  }
}
/******************************************************************
// -.I2C라인을 통해 읽어들인 압력센서의 Raw 값을 decimal 값으로 Bit Arrange
// -.pressure : 2^14(16384),temperature : 2^11(2048)
*******************************************************************/
void RawToDecimal_ISEN_P10k(void){
  press_decimal= (((dat[0] & 0x3F)<<8)|dat[1]);
  temp_decimal=((dat[2]<<3)|(dat[3]>>5));
}
/*******************************************************************
// Decimal 값을 10kPa 범위의 압력값으로 환산
// - 압력[Pa]=10000*(decimal값-0Pa일때 Offset)(16383.0-Offset), Offset=3000
// - 온도[C]=(200*decimal값/2047)-50, 온도범위: -50~+150[C]
********************************************************************/
void Calculate_ISEN_P10k(void){
  pressure = (float)((press_decimal-3000.0)/(16383.0-3000))*10000;
  if(pressure<0) pressure=0.0; // 0Pa에서 음수 값이면 ->0
  if(pressure>10000.0) pressure=10000.0; // 10kPa 이상 값이면 -> 10,000 Truncation
  temperature=200.0*(float)(temp_decimal/2047.0)-50.0;
}
