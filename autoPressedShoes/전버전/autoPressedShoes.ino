#include <Wire.h>
#include "pinHeader.h"
#include "encoder.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//차압센서 함수
void DataFetch_ISEN_P10K(void);
void RawToDecimal_ISEN_P10k(void);
void Calculate_ISEN_P10k(void);

//차압센서 변수
unsigned long press_decimal, temp_decimal;
float pressure, temperature;
unsigned char dat[4];

//BNO55 센서
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void readStep() {
    sensors_event_t angVelocityData, linearAccelData;
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    int y = angVelocityData.gyro.y;

    float diff = (y - oldgyroY) / LOOP_DELAY;

    if (diff > 0.06 && swingCheck <= 0) {
        IsSwing = !IsSwing;
        swingCheck = 200;
        //paze 변경
        if (!IsSwing) {
            //조여주기
            target_pressure = highPressure;
            Serial.println("target_pressure is highPressure");
        }
        else {
            //풀어주기
            target_pressure = lowPressure;
            Serial.println("target_pressure is lowPressure");
        }
    }
    oldgyroY = y;
}

//루프마다 모터 함수
void motorWork() {
    if (target_pressure - current_pressure != 0 && target_pressure != 0) {

        //모터의 방향 설정
        if (target_pressure - current_pressure > 0) {
            digitalWrite(MOTOR_DIR, HIGH);
            Serial.print(" motor HIGH ");
            Serial.println(" m1");
        }
        else {
            digitalWrite(MOTOR_DIR, LOW);
            Serial.print(" motor HIGH ");
            Serial.println(" m0");
        }

        //모터가 반대방향으로 변경하는 것 방지
        if (motor_current_degree < 10 && target_pressure - current_pressure <= 0) {
            float motorspeed = 0;
            analogWrite(MOTOR_PWM, motorspeed);
            return;
        }

        //모터 PID코드
        float motorspeed = pControl + iControl + dControl;

        analogWrite(MOTOR_PWM, motorspeed);

        //디버그 모드 프린트
        Serial.print("pressure : ");
        Serial.print(target_pressure);
        Serial.print(" ");
        Serial.print(current_pressure);
        Serial.print(" ");
        Serial.print(motor_current_degree);
        Serial.println(" ");
        Serial.print("motorspeed : ");
        Serial.print(motorspeed);
        Serial.println(" ");
    }
}

//셋업 함수
void setup() {
    Wire.begin(); // I2C 초기화(Master Mode)
    Serial.begin(115200); // UART 초기화 9600bps

    pinMode(13, OUTPUT);
    pinMode(MOTOR_DIR, OUTPUT);
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(MOTOR_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_A), doEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_B), doEncoderB, CHANGE);
    Serial.println("startup the autoPressing");

    if (!bno.begin()) {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }

    //최저압 측정
    Serial.println("checking lowPressure");
    delay(1000);
    lowPressure = 0;
    for (int i = 0; i < 10; i++) {
        DataFetch_ISEN_P10K();
        RawToDecimal_ISEN_P10k();
        Calculate_ISEN_P10k();
        lowPressure += press_decimal;
        delay(100);
    }
    lowPressure = lowPressure / 10;
    Serial.print("lowPressure ");
    Serial.println(lowPressure);

    //최고압 측정
    Serial.println("checking highPressure");
    digitalWrite(MOTOR_DIR, HIGH);
    float motorspeed = 200;
    analogWrite(MOTOR_PWM, motorspeed);
    int tmp = 0;
    while (1) {
        unsigned long mfirst = millis();
        DataFetch_ISEN_P10K();
        RawToDecimal_ISEN_P10k();
        Calculate_ISEN_P10k();
        if (press_decimal == highPressure && tmp > 100) break;
        highPressure = press_decimal;
        unsigned long msecond = millis();
        unsigned long code_time = msecond - mfirst;
        tmp++;
        delay(LOOP_DELAY - code_time);
    }
    Serial.print("highPressure ");
    Serial.println(highPressure);

    //다시 최저압으로 변경
    Serial.println("checking complete");
    tmp = 0;
    target_pressure = lowPressure;
    while (1) {
        unsigned long mfirst = millis();
        DataFetch_ISEN_P10K();
        RawToDecimal_ISEN_P10k();
        Calculate_ISEN_P10k();
        current_pressure = press_decimal;
        motorWork();
        if (abs(target_pressure - current_pressure) < 5 && tmp > 20) break;
        unsigned long msecond = millis();
        unsigned long code_time = msecond - mfirst;
        tmp++;
        delay(LOOP_DELAY - code_time);
    }

    Serial.println("start autopressing");
}

//압력센서 값을 받아 걸음걸이 분석 함수
void checkingPressureSensor(int val) {

    if (val >= SENSOR_CRITICAL_VALUE) {//압력센서가 일정한 값 이상일때 작동

        if (preMillis != 0) {
            unsigned long millidelay = millis() - preMillis;
            if (millidelay > 10000) {
                Serial.println("걸음걸이에 오류가 있습니다.");
            }
            else {
                distance_walking = millis() - preMillis;
                Serial.print("걸음걸이 간격 시간은 ");
                Serial.println(distance_walking);
                walking_percentage = distance_walking;
            }
        }
        sensorDelay = 20;
        preMillis = millis();
    }
}

//루프 함수 ///////////////////////////////////////////////////////////////////////////////////////////
void loop() {
    unsigned long mfirst = millis();
    /*
      sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
      bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
      bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
      bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
      bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

      printEvent(&orientationData);
      printEvent(&angVelocityData);
      printEvent(&linearAccelData);
      printEvent(&magnetometerData);
      printEvent(&accelerometerData);
      printEvent(&gravityData);

      */
    /*if(distance_walking!=(-1)){
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
        /*if((100.0-percentage)>80){
          //조여주기
          target_pressure=highPressure;
          Serial.println("target_pressure is highPressure");
        }else if((100.0-percentage)>=20){
          //풀어주기
          target_pressure=lowPressure;
          Serial.println("target_pressure is lowPressure");
        }
    }*/

    /*//현재 압력값 받아오기
    int val = 1023- analogRead(A0);
    Serial.print(" Sensor : ");
    Serial.println(val);
    if(sensorDelay<=0){
      checkingPressureSensor(val);
    }else{
      sensorDelay--;
      if(sensorDelay<0) sensorDelay=0;
    }*/
    readStep();

    //모터 함수
    motorWork();

    //현재 압력값 받아오기
    DataFetch_ISEN_P10K();
    RawToDecimal_ISEN_P10k();
    Calculate_ISEN_P10k();
    current_pressure = press_decimal;

    //PID 값 알아내기 위한 코드
    errorGap = target_pressure - current_pressure - realError;
    realError = target_pressure - current_pressure;
    accError = accError + realError;

    pControl = mP * realError;
    iControl = mI * (accError * LOOP_DELAY);  // 전에 에러의 합과 지금 에러를 더해서 딜레이를 곱해서 더함
    dControl = mD * (errorGap / LOOP_DELAY);  // 전에 error와 지금 error의 차이를 딜레이로 나눔

    /*
      sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
      bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
      bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
      bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
      bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
    */

    unsigned long msecond = millis();
    unsigned long code_time = msecond - mfirst;
    delay(LOOP_DELAY - code_time);
}
//--------------------------------------------------------------------------------------------------------------------
void printEvent(sensors_event_t* event) {
    double x = -1000000, y = -1000000, z = -1000000; //dumb values, easy to spot problem
    if (event->type == SENSOR_TYPE_ACCELEROMETER) {
        Serial.print("Accl:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else if (event->type == SENSOR_TYPE_ORIENTATION) {
        Serial.print("Orient:");
        x = event->orientation.x;
        y = event->orientation.y;
        z = event->orientation.z;
    }
    else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
        Serial.print("Mag:");
        x = event->magnetic.x;
        y = event->magnetic.y;
        z = event->magnetic.z;
    }
    else if (event->type == SENSOR_TYPE_GYROSCOPE) {
        Serial.print("Gyro:");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    }
    else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
        Serial.print("Rot:");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    }
    else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
        Serial.print("Linear:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else if (event->type == SENSOR_TYPE_GRAVITY) {
        Serial.print("Gravity:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else {
        Serial.print("Unk:");
    }

    Serial.print("\tx= ");
    Serial.print(x);
    Serial.print(" |\ty= ");
    Serial.print(y);
    Serial.print(" |\tz= ");
    Serial.println(z);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// 압력센서 모듈에서 압력 및 온도 Raw Data를 Fetch
void DataFetch_ISEN_P10K(void) {
    int i = 0;
    /****************************************************************************
    * 압력센서는 특별한 데이타 요청이 없을 때 소모전력을 최소화 하기 위해서 Sleep 상태로 들어감.
    * 센서를 Wake up 하기 위해서는 두 가지 명령이 존재.
    * * 1. Read MR -> slave address 0x28 + I2C read command bit를 적용하여 Wake up
    * --> 압력 및 온도 데이터를 Read할때 마다 update 됨.
    * 2. Write MR -> slave address 0x28 + I2C write command bit를 적용햐여 Wake up
    * --> 압력 값만 Update 되고 온도 값은 이전 값을 유지함.
    * 압력센서 Application 에 따라 Wake up 명령을 적절해 사용할 필요 있음.
    ****************************************************************************/
    Wire.write(P10K_i2c_address + 0x01); // I2C Read_MR
    //Wire.write(P10K_i2c_address); // I2C Write_MR
    Wire.endTransmission();
    delay(10);
    Wire.requestFrom(0x28, 4);
    while (Wire.available()) {
        if (i >= 4) i = 0;
        dat[i] = Wire.read();
        i++;
    }
}
/******************************************************************
// -.I2C라인을 통해 읽어들인 압력센서의 Raw 값을 decimal 값으로 Bit Arrange
// -.pressure : 2^14(16384),temperature : 2^11(2048)
*******************************************************************/
void RawToDecimal_ISEN_P10k(void) {
    press_decimal = (((dat[0] & 0x3F) << 8) | dat[1]);
    temp_decimal = ((dat[2] << 3) | (dat[3] >> 5));
}
/*******************************************************************
// Decimal 값을 10kPa 범위의 압력값으로 환산
// - 압력[Pa]=10000*(decimal값-0Pa일때 Offset)(16383.0-Offset), Offset=3000
// - 온도[C]=(200*decimal값/2047)-50, 온도범위: -50~+150[C]
********************************************************************/
void Calculate_ISEN_P10k(void) {
    pressure = (float)((press_decimal - 3000.0) / (16383.0 - 3000)) * 10000;
    if (pressure < 0) pressure = 0.0; // 0Pa에서 음수 값이면 ->0
    if (pressure > 10000.0) pressure = 10000.0; // 10kPa 이상 값이면 -> 10,000 Truncation
    temperature = 200.0 * (float)(temp_decimal / 2047.0) - 50.0;
}
