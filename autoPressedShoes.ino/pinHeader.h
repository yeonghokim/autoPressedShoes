
#define P10K_i2c_address 0x28
//핀번호 리스트
#define MOTOR_DIR 11// 모터 DIR 핀번호
#define MOTOR_PWM 10// 모터 PWM 핀번호

//특별한 값 리스트
#define LOOP_DELAY 50

#define MOTOR_RPM 85
#define MOTRO_RAD_PER_MILLISEC (MOTOR_RPM * PI / 30000.0)

#define STEP_DELAY 2000
#define STEP_STANDING_READY 0.20
#define STEP_SWING_READY 0.90

#define SENSOR_CRITICAL_VALUE 5

void DataFetch_ISEN_P10K(void);
void RawToDecimal_ISEN_P10k(void);
void Calculate_ISEN_P10k(void);


unsigned long press_decimal,temp_decimal;
float pressure,temperature;
unsigned char dat[4];

int mP=5;
int mI=0;
int mD=0;

float pressure_diff;
float pressure_integ=0;
boolean pressure_Integral=false;

int target_pressure;
int current_pressure;
int previous_pressure=0;

int motor_target_degree=0;

boolean motor_on=true;
int count = 0;
boolean IsSwing=true;

int preValue=-1;
unsigned long preMillis=0;
boolean PressureCheck = false;
int sensorDelay=0;

int distance_walking = -1;
int walking_percentage=-1;

int first_pressure = -1;
int maximal_pressure_interval = 1000;
