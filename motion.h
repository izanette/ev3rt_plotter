#ifndef __MOTION_H__
#define __MOTION_H__

/**
 * Motion
 */

//#define USE_NEW_PID
#define MOTION_TIME_MULTIPLIER 1.25

#define X_MAX_POS (1080*5)
#define Y_MAX_POS (1500*5)
#define Z_MAX_POS 120

#define INV_MOTOR_X 1

#define BACKLASH_X  -20
#define BACKLASH_Y  -20

#define SERROR_SIZE 16
#define LOG_MAX_SIZE 1000

// range that the motor power is linear
// does it change with the voltage of the batteries?
#define MOTOR_MIN_POWER 2
#define MOTOR_MAX_POWER 65

typedef struct {
    int32_t p0;
    int32_t dp;
    float   vf;
    float   vmax;
    float   amax;
    float   amin;
    float   t1;
    float   t2;
    float   t3;
} MotionArgs;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Ks;
    int sampleTime;
} PidParams;

typedef struct {
    float time;
    float integral;
    float errors[SERROR_SIZE];
    int err_counter;
} PidRun;

typedef struct {
    int32_t pos;
    float   tgt_pos;
    float   time;
    int32_t power;
    float   P;
    float   I;
    float   D;
    float   S;
} PidLogData;

typedef struct {
    int counter;
    PidLogData data[LOG_MAX_SIZE];
} PidLog;

int32_t posX();
int32_t posY();
int32_t posZ();
int32_t motorPos(motor_port_t motor);
void motorX(int power);
void motorY(int power);
void motorZ(int power);
void motorPower(motor_port_t motor, int power);
void rotateX(int degrees, int power);
void rotateY(int degrees, int power);
void rotateZ(int degrees, int power);
void rotateAbsX(int pos, int power);
void rotateAbsY(int pos, int power);
void rotateAbsZ(int pos, int power);
void penUp();
void penDown();
void move(int32_t x, int32_t y, int32_t power);

void homing(intptr_t unused);
void control(intptr_t unused);

extern PidParams pidParams;
extern PidLog pidLog;
extern float useSCurve;
extern float amax;
extern float amin;
extern float speedmax;
extern float pidTestSize;
//extern float multiplier;
extern float backlashX;
extern float backlashY;
extern const motor_port_t  x_motor;
extern const motor_port_t  y_motor;
extern const motor_port_t  z_motor;
extern const sensor_port_t yend_sensor;
extern const sensor_port_t xend_sensor;

float get_time(SYSTIM start);

void testRawPid(intptr_t unused);

#endif // __MOTION_H__