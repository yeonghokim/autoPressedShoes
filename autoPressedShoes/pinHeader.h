
#define P10K_i2c_address 0x28
//핀번호 리스트
#define MOTOR_DIR 11// 모터 DIR 핀번호
#define MOTOR_PWM 10// 모터 PWM 핀번호

//특별한 값 리스트
#define LOOP_DELAY 10

#define MOTOR_RPM 85
#define MOTRO_RAD_PER_MILLISEC (MOTOR_RPM * PI / 30000.0)

#define STEP_DELAY 2000
#define STEP_STANDING_READY 0.20
#define STEP_SWING_READY 0.90

#define SENSOR_CRITICAL_VALUE 5

#define mP 5
#define mI 0
#define mD 0

float pControl=0;
float iControl=0;
float dControl=0;
float errorGap=0;
float realError=0;
float accError=0;

int target_pressure;
int current_pressure;
int previous_pressure=0;

int motor_target_degree=0;

boolean motor_on=true;
int count = 0;
boolean IsSwing=true;
int swingCheck=0;
float oldgyroY=0;

int preValue=-1;
unsigned long preMillis=0;
boolean PressureCheck = false;
int sensorDelay=0;

int distance_walking = -1;
int walking_percentage=-1;

int first_pressure = -1;
int maximal_pressure_interval = 1000;

int lowPressure=2900;
int highPressure=3200;
