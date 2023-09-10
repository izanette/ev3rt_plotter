/*
 * motion.c
 *
 *  Created on: 08/Sep/2018
 *      Author: izanette
 */

#include "ev3api.h"
#include "app.h"
#include "PID_v1.h"
#include "utils.h"
#include "cli_menu.h"
#include "motion.h"
#include <stdlib.h>
#include <math.h>

#define EPSILON 1e-10
/*
input:
    vmax
    amax
    amin
    p0
    pf

dp = pf - p0
pa = vmax*vmax/2/amax + vmax*vmax/2/amin
pa < dp:
    vf = vmax
pa >= dp:
    vf = SQRT(8*dp*amax*amin*(amax+amin))/2/(amax+amin)

tvmax = vf / amax             #
tvmin = vf / amin             #
dpvmax = amax*tvmax*tvmax/2   
dpvmin = amin*tvmin*tvmin/2   
sumdp = dpvmax + dpvmin       
tvf = (dp - sumdp) / vf       #

t1 = tvmax
t2 = tvmax + tvf
t3 = tvmax + tvf + tvmin

p(t) =
    t < tvmax:
        p0+amax*t*t/2
    t < tvmax+tvf:
        p0+amax*tvmax*tvmax/2+(t-tvmax)*vf
    t <= tvmax+tvf+tvmin:
        p0+amax*tvmax*tvmax/2+(t-tvmax)*vf-amin*POWER(t-tvf-tvmax,2)/2
    t > tvmax+tvf+tvmin:
        p0+dp

p(t) =
    t < t1:
        p0+amax*t*t/2
    t < t2:
        p0+amax*t1*t1/2+(t-t1)*vf
    t <= t3:
        p0+amax*t1*t1/2+(t-t1)*vf-amin*POWER(t-t2,2)/2
    t > t3:
        p0+dp
*/

#ifndef USE_NEW_PID

PidParams pidParams = {
    .Kp = 4.00,
    .Ki = 0.10,
    .Kd = 8.00,
    .Ks = 0.10,
    .sampleTime = 10
};

#else

PidParams pidParams = {
    .Kp = 1.0,
    .Ki = 2.56,
    .Kd = 0.08,
    .Ks = 0.02,
    .sampleTime = 10
};

#endif // USE_NEW_PID


PidLog pidLog;

float amax = 600.0;
float amin = 600.0;
float speedmax = 600.0;
float useSCurve = 1.0;

float pidTestSize = 1000.0;
float backlashX = BACKLASH_X;
float backlashY = BACKLASH_Y;


int32_t dbg_x0;
int32_t dbg_y0;
int32_t dbg_x;
int32_t dbg_y;
MotionArgs dbg_argx;
MotionArgs dbg_argy;


/**
 * X-Axis motor:        Port D
 * Y-Axis motor:        Port A
 * Z-Axis motor:        Port C
 * Y end stop sensor:   Port 1
 */

const motor_port_t  x_motor      = EV3_PORT_D;
const motor_port_t  y_motor      = EV3_PORT_A;
const motor_port_t  z_motor      = EV3_PORT_C;
const sensor_port_t yend_sensor  = EV3_PORT_1;
const sensor_port_t xend_sensor  = EV3_PORT_4;

void copyMotionArgs(MotionArgs* toArgs, MotionArgs* fromArgs)
{
    toArgs->p0   = fromArgs->p0;
    toArgs->dp   = fromArgs->dp;
    toArgs->vf   = fromArgs->vf;
    toArgs->vmax = fromArgs->vmax;
    toArgs->amax = fromArgs->amax;
    toArgs->amin = fromArgs->amin;
    toArgs->t1   = fromArgs->t1;
    toArgs->t2   = fromArgs->t2;
    toArgs->t3   = fromArgs->t3;
}

float position(MotionArgs* args, float t)
{
    if (t < args->t1)
    {
        return args->p0 + args->amax * t * t / 2;
    }
    
    if (t < args->t2)
    {
        return
            args->p0
          + args->amax * args->t1 * args->t1 / 2 
          + (t - args->t1) * args->vf;
    }
    
    if (t < args->t3)
    {
        return
            args->p0
          + args->amax * args->t1 * args->t1 / 2 
          + (t - args->t1) * args->vf 
          - args->amin * (t - args->t2) * (t - args->t2) / 2;
    }
    
    return args->p0 + args->dp;
}

class MotionArgsPosition : public Position
{
public:
    MotionArgsPosition(MotionArgs* pMotionArgs)
    : motionArgs(pMotionArgs)
    {}
    
    virtual float compute(unsigned long time)
    {
        return position(motionArgs, time * 0.001);
    }

private:
    MotionArgs* motionArgs;
};

int32_t posX()
{
    return INV_MOTOR_X * ev3_motor_get_counts(x_motor);
}

int32_t posY()
{
    return Y_MAX_POS + ev3_motor_get_counts(y_motor);
}

int32_t posZ()
{
    return Z_MAX_POS + ev3_motor_get_counts(z_motor);
}

int32_t motorPos(motor_port_t motor)
{
    if (motor == x_motor)
        return posX();
    else if (motor == y_motor)
        return posY();
    else if (motor == z_motor)
        return posZ();
    
    return 0;
}

void motorX(int power)
{
    if (power)
        ev3_motor_set_power(x_motor, INV_MOTOR_X * power);
    else
        ev3_motor_stop(x_motor, false);
}

void motorY(int power)
{
    if (power)
        ev3_motor_set_power(y_motor, power);
    else
        ev3_motor_stop(y_motor, false);
}

void motorZ(int power)
{
    if (power)
        ev3_motor_set_power(z_motor, -power);
    else
        ev3_motor_stop(z_motor, false);
}

void motorPower(motor_port_t motor, int power)
{
    if (motor == x_motor)
        return motorX(power);
    else if (motor == y_motor)
        return motorY(power);
    else if (motor == z_motor)
        return motorZ(power);
}

void turnOffAllMotors()
{
    motorX(0);
    motorY(0);
    motorZ(0);
}

void rotateX(int degrees, int power)
{
    ev3_motor_rotate(x_motor, -INV_MOTOR_X * degrees, abs(power), true);
}

void rotateY(int degrees, int power)
{
    ev3_motor_rotate(y_motor, -degrees, abs(power), true);
}

void rotateZ(int degrees, int power)
{
    ev3_motor_rotate(z_motor, -degrees, abs(power), true);
}

void rotateAbsX(int pos, int power)
{
    int32_t delta = posX() - pos;
    rotateX(delta, power);
}

void rotateAbsY(int pos, int power)
{
    int32_t delta = posY() - pos;
    rotateY(delta, power);
}

void rotateAbsZ(int pos, int power)
{
    int32_t delta = posZ() - pos;
    rotateZ(delta, power);
}

void penUp()
{
    rotateAbsZ(Z_MAX_POS, 10);
}

void penDown()
{
    rotateAbsZ(0, 10);
}

void motor(motor_port_t m, int power)
{
    if (power)
        ev3_motor_set_power(m, power);
    else
        ev3_motor_stop(m, false);
}

float getTimeMillis(SYSTIM start)
{
    //const float TIME_TO_SECONDS = 1000.0;
    SYSTIM now;
    get_tim(&now);
    return (now - start); // / TIME_TO_SECONDS;
}

void findHome(motor_port_t m, int power, int hold_time, int inv)
{
    SYSTIM start;
    get_tim(&start);
    
    motor(m, inv * power);
    tslp_tsk(hold_time);
    
    int32_t last_pos = ev3_motor_get_counts(m);
    float time_last_pos = getTimeMillis(start);
    
    while(true)
    {
        int32_t pos = ev3_motor_get_counts(m);
        
        if (pos != last_pos)
        {
            time_last_pos = getTimeMillis(start);
            last_pos = pos;
        }
        else
        {
            if (getTimeMillis(start) - time_last_pos > hold_time)
            {
                motor(m, 0);
                ev3_motor_reset_counts(m);
                return;
            }
        }
    }
}

void homing(intptr_t unused)
{
    char lcdstr[100];
    
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    draw_title("Homing", 0, 0, MENU_FONT);
    
    print(3, "Homing Z");
    findHome(z_motor, 7, 80, 1);
    print(3, "Homing Z - Done");
    
    print(4, "Homing X");
    //findHome(x_motor, 5, 30, -1);
    motor(x_motor, -60);
    while(!ev3_touch_sensor_is_pressed(xend_sensor))
    {
        sprintf(lcdstr, "XPos %ld", posX());
        print(6, lcdstr);
    }
    motor(x_motor, 0);
    ev3_motor_reset_counts(x_motor);
    print(4, "Homing X - Done");
    
    print(5, "Homing Y");
    motor(y_motor, 60);
    while(!ev3_touch_sensor_is_pressed(yend_sensor))
    {
        sprintf(lcdstr, "YPos %ld", posY());
        print(6, lcdstr);
    }
    motor(y_motor, 0);
    ev3_motor_reset_counts(y_motor);
    
    //rotateAbsY(100, 15);
    print(5, "Homing Y - Done");
    move(0, 0, speedmax);
    
    //ev3_speaker_play_tone(NOTE_C4, 100);
    //tslp_tsk(2000);
}

void calculateMotionArgsSCurve(MotionArgs* args, int32_t p0, int32_t pf, float vmax, float amax, float amin)
{
    args->vmax = vmax;
    
    int32_t dp = pf - p0;
    if (dp < 0)
    {
        vmax = -vmax;
        amax = -amax;
        amin = -amin;
    }
    
    float pa = vmax*vmax*.5*(1/amax+1/amin);
    float vf = vmax;
    if (   (dp > 0 && pa > dp)
        || (dp < 0 && pa < dp))
    {
        vf = sqrt(8 * dp * amax * amin * (amax + amin)) / 2 / (amax + amin);
    }
    
    float tvmax = vf / amax;
    float tvmin = vf / amin;
    float dpvmax = amax * tvmax * tvmax / 2;
    float dpvmin = amin * tvmin * tvmin / 2;
    float sumdp = dpvmax + dpvmin;
    float tvf = (dp - sumdp) / vf;
    
    if (dp == 0)
    {
        amax = 0;
        amin = 0;
        vf = 0;
    }
    
    args->p0 = p0;
    args->dp = dp;
    args->vf = vf;
    args->amax = amax;
    args->amin = amin;
    args->t1 = tvmax;
    args->t2 = tvmax+tvf;
    args->t3 = tvmax+tvf+tvmin;
}

void calculateMotionArgsLinear(MotionArgs* args, int32_t p0, int32_t pf, float vmax, float amax, float amin)
{
    args->vmax = vmax;
    
    int32_t dp = pf - p0;
    if (dp < 0)
    {
        vmax = -vmax;
    }
    
    args->p0 = p0;
    args->dp = dp;
    args->vf = vmax;
    args->amax = amax;
    args->amin = amin;
    args->t1 = 0.0;
    args->t2 = dp / vmax;
    args->t3 = args->t2;
}

void calculateMotionArgs(MotionArgs* args, int32_t p0, int32_t pf, float vmax, float amax, float amin)
{
    if (useSCurve > 0.01)
        calculateMotionArgsSCurve(args, p0, pf, vmax, amax, amin);
    else
        calculateMotionArgsLinear(args, p0, pf, vmax, amax, amin);
}

void syncMotionArgs(MotionArgs* args1, MotionArgs* args2)
{
    // assumes acceleration is the same for both args
    if (args1->t3 == args2->t3)
        return;
    if (fabs(args1->dp) < EPSILON || fabs(args2->dp) < EPSILON)
        return;
    
    if (args1->t3 < args2->t3)
    {
        MotionArgs* t = args2;
        args2 = args1;
        args1 = t;
    }
    
    float scale = fabs((float)args2->dp / args1->dp);
    
    if (scale == 0.0)
        return;
    
    calculateMotionArgs(args2, args2->p0, args2->p0 + args2->dp, fabs(args2->vmax * scale), fabs(args2->amax * scale), fabs(args2->amin * scale));
}

int32_t int_power1(float power)
{
    if (power >  100.0) return  100;
    if (power < -100.0) return -100;
    
    return (int32_t) power;
}

float float_power(float power)
{
    float apower = fabs(power);
    if (apower > 100.0) apower = 100.0;
    
    apower = apower / 100.0 * (MOTOR_MAX_POWER - MOTOR_MIN_POWER);
    
    if (apower < 1.0) return 0.0;
    
    if (power < 0) return - apower - MOTOR_MIN_POWER;
    
    return apower + MOTOR_MIN_POWER;
}

int32_t int_power(float power)
{
    return (int32_t) float_power(power);
}

// http://en.wikipedia.org/wiki/Simple_linear_regression
float errorDev(float* errors, int count)
{
    static float Sx = 0.0;
    static float D = 0.0;
    
    if (Sx == 0.0)
    {
        float Sxx = 0;
        for(int i = 1; i < SERROR_SIZE; i++)
        {
            Sx += i;
            Sxx += i * i;
        }
        D = SERROR_SIZE * Sxx - Sx * Sx;
    }
    
    if (count < SERROR_SIZE-1) return 0.0;
    
    float Sy = 0.0;
    float Sxy = 0.0;
    for(int i = 0; i < SERROR_SIZE; i++)
    {
        float error = errors[(count+i+1) % SERROR_SIZE];
        
        Sy += error;
        Sxy += i * error;
    }
    
    return (SERROR_SIZE * Sxy - Sx * Sy) / D;
}

float fround(float f)
{
    return floor(f + 0.5);
}

float get_time(SYSTIM start)
{
    const float TIME_TO_SECONDS = 1000.0;
    SYSTIM now;
    get_tim(&now);
    return (now - start) / TIME_TO_SECONDS;
}

void initPidRun(PidRun* run)
{
    run->time = 0.0;
    run->integral = 0.0;
    run->err_counter = 0;
    for(int i = 0; i < SERROR_SIZE; i++) run->errors[i] = 0.0;
}

float absminf(float a, float b)
{
    if (a > b) return b;
    if (a < -b) return -b;
    return a;
}

bool motor_pid1(MotionArgs* args, PidParams* pid, PidRun* run, motor_port_t motor, float time, PidLog* log)
{
    const float iFactor = 0.95;
    
    int32_t pos = motorPos(motor);
    float dtime = time - run->time;
    float tgt_pos_raw = position(args, time);
    float tgt_pos_future = position(args, time + dtime);
    
    float tgt_pos = fround(tgt_pos_raw);
    float error = tgt_pos - pos;
    run->integral = float_power(run->integral * iFactor + pid->Ki * error);
    run->errors[run->err_counter % SERROR_SIZE] = -error;
    float derivative = errorDev(run->errors, run->err_counter++);
    if (dtime < EPSILON) dtime = EPSILON; // not sure if this ever happens
    
    float speed = (tgt_pos_future - tgt_pos_raw) / dtime;
    
    float P = pid->Kp * error;
    float I =           run->integral;
    float D = pid->Kd * derivative;
    float S = pid->Ks * speed;
    float power = P + I + D + S;
    int32_t ipower = int_power(power);
    motorPower(motor, ipower);
    
    //if (power >= 199.0 || power <= -199.0)
    //{
    //    turnOffAllMotors();
    //    char buf[100];
    //    clearScreen();
    //    if (motor == x_motor)
    //        sprintf(buf, "MOTOR X");
    //    else
    //        sprintf(buf, "MOTOR Y");
    //    print(0, buf);
    //    sprintf(buf, "X %ld %ld",   dbg_x0, dbg_x   ); print(2, buf);
    //    sprintf(buf, "Y %ld %ld",   dbg_y0, dbg_y   ); print(3, buf);
    //    sprintf(buf, "P %.1f %.1f", P, error        ); print(4, buf);
    //    sprintf(buf, "I %.1f %.1f", I, run->integral); print(5, buf);
    //    sprintf(buf, "D %.1f %.1f", D, derivative   ); print(6, buf);
    //    sprintf(buf, "S %.1f %.1f", S, speed        ); print(7, buf);
    //    ev3_speaker_play_tone(NOTE_C4, 50);
    //    waitEnterButtonPressed();
    //    waitNoButtonPressed();
    //    
    //    clearScreen();
    //    sprintf(buf, "X Motion Args");                                     print(0, buf);
    //    sprintf(buf, "p0 %ld dp %ld",       dbg_argx.p0, dbg_argx.dp);     print(1, buf);
    //    sprintf(buf, "vf   %.2f",           dbg_argx.vf);                  print(2, buf);
    //    sprintf(buf, "vmax %.1f",           dbg_argx.vmax);                print(3, buf);
    //    sprintf(buf, "a %.1f %.1f",         dbg_argx.amax, dbg_argx.amin); print(4, buf);
    //    sprintf(buf, "t1 %.3f",             dbg_argx.t1);                  print(5, buf);
    //    sprintf(buf, "t2 %.3f",             dbg_argx.t2);                  print(6, buf);
    //    sprintf(buf, "t3 %.3f",             dbg_argx.t3);                  print(7, buf);
    //    ev3_speaker_play_tone(NOTE_C4, 50);
    //    waitEnterButtonPressed();
    //    waitNoButtonPressed();
    //    
    //    clearScreen();
    //    sprintf(buf, "Y Motion Args");                                     print(0, buf);
    //    sprintf(buf, "p0 %ld dp %ld",       dbg_argy.p0, dbg_argy.dp);     print(1, buf);
    //    sprintf(buf, "vf   %.2f",           dbg_argy.vf);                  print(2, buf);
    //    sprintf(buf, "vmax %.1f",           dbg_argy.vmax);                print(3, buf);
    //    sprintf(buf, "a %.1f %.1f",         dbg_argy.amax, dbg_argy.amin); print(4, buf);
    //    sprintf(buf, "t1 %.3f",             dbg_argy.t1);                  print(5, buf);
    //    sprintf(buf, "t2 %.3f",             dbg_argy.t2);                  print(6, buf);
    //    sprintf(buf, "t3 %.3f",             dbg_argy.t3);                  print(7, buf);
    //    ev3_speaker_play_tone(NOTE_C4, 50);
    //    waitEnterButtonPressed();
    //}
    
    float endTime = args->t3 * MOTION_TIME_MULTIPLIER;
    if (log)
    {
        float logInterval = endTime / LOG_MAX_SIZE;
        int c = log->counter;
        float lastLog = (c > 0) ? log->data[c-1].time : -logInterval;
        
        if (c < LOG_MAX_SIZE && time - lastLog >= logInterval)
        {
            log->data[c].pos = pos;
            log->data[c].tgt_pos = tgt_pos;
            log->data[c].time = time;
            log->data[c].power = ipower;
            log->data[c].P = P;
            log->data[c].I = I;
            log->data[c].D = D;
            log->data[c].S = S;
            log->counter   = c + 1;
        }
    }
    
    //if (time < args->t3)
    //    return false;
    
    if (time > endTime)
        return true;
    
    return fabs(args->p0 + args->dp - pos) < 1;
}

bool motor_pid(MotionArgs* args, PID2* pid, motor_port_t motor, float time, PidLog* log)
{
    int32_t pos         = motorPos(motor);
    float   tgt_pos_raw = position(args, time);
    float   tgt_pos     = fround(tgt_pos_raw);
    float   power       = pid->Compute2(pos, tgt_pos);
    int32_t ipower      = int_power(power);
    float   endTime     = args->t3 * MOTION_TIME_MULTIPLIER;
    motorPower(motor, ipower);
    
    if (log)
    {
        float logInterval = endTime / LOG_MAX_SIZE;
        int c = log->counter;
        float lastLog = (c > 0) ? log->data[c-1].time : -logInterval;
        
        if (c < LOG_MAX_SIZE && time - lastLog >= logInterval)
        {
            log->data[c].pos     = pos;
            log->data[c].tgt_pos = tgt_pos;
            log->data[c].time    = time;
            log->data[c].power   = ipower;
            log->data[c].P       = pid->GetP();
            log->data[c].I       = pid->GetI();
            log->data[c].D       = pid->GetD();
            log->data[c].S       = pid->GetS();
            log->counter         = c + 1;
        }
    }
    
    //if (time < args->t3)
    //    return false;
    
    if (time > endTime)
        return true;
    
    return fabs(args->p0 + args->dp - pos) < 1;
}

void printMotionArgs(MotionArgs* args1, MotionArgs* args2)
{
    char buf[50];
    print(0, "1:vf amax amin/t");
    sprintf(buf, "%.1f %.1f %.1f", args1->vf, args1->amax, args1->amin);
    print(1, buf);
    sprintf(buf, "%.2f %.2f %.2f", args1->t1, args1->t2, args1->t3);
    print(2, buf);
    sprintf(buf, "%ld", args1->dp);
    print(3, buf);

    print(4, "2:vf amax amin/t");
    sprintf(buf, "%.1f %.1f %.1f", args2->vf, args2->amax, args2->amin);
    print(5, buf);
    sprintf(buf, "%.2f %.2f %.2f", args2->t1, args2->t2, args2->t3);
    print(6, buf);
    sprintf(buf, "%ld", args2->dp);
    print(7, buf);
}

// uses internal PID instead of the library's PID
void raw_move1(int32_t x, int32_t y, int32_t power)
{
    char buf[100];
    MotionArgs xArgs;
    MotionArgs yArgs;
    PidRun xRun; xRun.err_counter = 0;
    PidRun yRun; yRun.err_counter = 0;
    
    pidLog.counter = 0;
    
    int32_t x0 = motorPos(x_motor);
    int32_t y0 = motorPos(y_motor);
    calculateMotionArgs(&xArgs, x0, x, power, amax, amin);
    calculateMotionArgs(&yArgs, y0, y, power, amax, amin);
    syncMotionArgs(&xArgs, &yArgs);
    
    //printMotionArgs(&xArgs, &yArgs);
    //waitButtonPressed();
    
    // save this for debug
    dbg_x  = x;
    dbg_y  = y;
    dbg_x0 = x0;
    dbg_y0 = y0;
    copyMotionArgs(&dbg_argx, &xArgs);
    copyMotionArgs(&dbg_argy, &yArgs);
    
    initPidRun(&xRun);
    initPidRun(&yRun);
    
    SYSTIM start;
    get_tim(&start);
    
    bool xFinished = false;
    bool yFinished = false;
    while(!xFinished || !yFinished)
    {
        sprintf(buf, "(%5ld, %5ld)", posX(), posY());
        print(1, buf);
        float time = get_time(start);
        if (!xFinished)
        {
            xFinished = motor_pid1(&xArgs, &pidParams, &xRun, x_motor, time, &pidLog);
            if (xFinished) motorX(0);
        }
        if (!yFinished)
        {
            yFinished = motor_pid1(&yArgs, &pidParams, &yRun, y_motor, time, NULL);
            if (yFinished) motorY(0);
        }
        
        tslp_tsk(pidParams.sampleTime);
    }
}

// uses a library PID
void raw_move(int32_t x, int32_t y, int32_t power)
{
    char buf[100];
    MotionArgs xArgs;
    MotionArgs yArgs;
    //PidRun xRun;
    //PidRun yRun;
    
    MotionArgsPosition motionArgsPositionX(&xArgs);
    MotionArgsPosition motionArgsPositionY(&yArgs);
    
    float xInput=0, xOutput=0, xSetpoint=0;
    float yInput=0, yOutput=0, ySetpoint=0;
    //PID  xPid(&xInput, &xOutput, &xSetpoint, pidParams.Kp, pidParams.Ki, pidParams.Kd, P_ON_E, DIRECT);
    PID2  xPid(&xInput, &xOutput, &xSetpoint, pidParams.Kp, pidParams.Ki, pidParams.Kd, pidParams.Ks, P_ON_E, DIRECT, &motionArgsPositionX);
    xPid.SetMode(AUTOMATIC);
    xPid.SetSampleTime(pidParams.sampleTime);
    xPid.SetOutputLimits(-MOTOR_MAX_POWER, MOTOR_MAX_POWER);
    
    //PID  yPid(&yInput, &yOutput, &ySetpoint, pidParams.Kp, pidParams.Ki, pidParams.Kd, P_ON_E, DIRECT);
    PID2  yPid(&yInput, &yOutput, &ySetpoint, pidParams.Kp, pidParams.Ki, pidParams.Kd, pidParams.Ks, P_ON_E, DIRECT, &motionArgsPositionY);
    yPid.SetMode(AUTOMATIC);
    yPid.SetSampleTime(pidParams.sampleTime);
    yPid.SetOutputLimits(-MOTOR_MAX_POWER, MOTOR_MAX_POWER);
    
    pidLog.counter = 0;
    
    int32_t x0 = motorPos(x_motor);
    int32_t y0 = motorPos(y_motor);
    calculateMotionArgs(&xArgs, x0, x, power, amax, amin);
    calculateMotionArgs(&yArgs, y0, y, power, amax, amin);
    syncMotionArgs(&xArgs, &yArgs);
    
    //printMotionArgs(&xArgs, &yArgs);
    //waitButtonPressed();
    
    //initPidRun(&xRun);
    //initPidRun(&yRun);
    
    PidLog* log = &pidLog;
    log->counter = 0;
    
    SYSTIM start;
    get_tim(&start);
    
    // PID2 only
    xPid.Start();
    yPid.Start();
    
    sprintf(buf, "(%5ld, %5ld)", x, y);
    print(1, buf);
    
    bool xFinished = false;
    bool yFinished = false;
    while(!xFinished || !yFinished)
    {
        //sprintf(buf, "(%5ld, %5ld)", posX(), posY());
        //print(2, buf);
        float time = get_time(start);
        if (!xFinished)
        {
            xFinished = motor_pid(&xArgs, &xPid, x_motor, time, log);
            if (xFinished)
            {
                //print(2, "X finished");
                motorX(0);
            }
        }
        if (!yFinished)
        {
            yFinished = motor_pid(&yArgs, &yPid, y_motor, time, NULL);
            if (yFinished)
            {
                //print(3, "Y finished");
                motorY(0);
            }
        }
        
        //tslp_tsk(1);
    }
}

void move(int32_t x, int32_t y, int32_t power)
{
    static bool lastXmovRight = false;
    static bool lastYmovDown = false;
    
    if (x < 0) x = 0;
    if (x > X_MAX_POS) x = X_MAX_POS;
    if (y < 0) y = 0;
    if (y > Y_MAX_POS) y = Y_MAX_POS;
    
    int32_t bs_fix_x = 0;
    int32_t bs_fix_y = 0;
    int32_t x0 = posX();
    int32_t y0 = posY();
    
    if (x0 > x)
    {
        bs_fix_x = backlashX;
        if (lastXmovRight && y != y0)
        {
            #ifdef USE_NEW_PID
            raw_move(x0+bs_fix_x, y0, power);
            #else
            raw_move1(x0+bs_fix_x, y0, power);
            #endif
        }
        lastXmovRight = false;
    }
    else if (x0 < x)
        lastXmovRight = true;
    
    if (y0 > y)
    {
        bs_fix_y = backlashY;
        if (lastYmovDown && x != x0)
        {
            #ifdef USE_NEW_PID
            raw_move(x0, y0+bs_fix_y, power);
            #else
            raw_move1(x0, y0+bs_fix_y, power);
            #endif
        }
        lastYmovDown = false;
    }
    else if (y0 < y)
        lastYmovDown = true;
    
    #ifdef USE_NEW_PID
    raw_move(x+bs_fix_x, y+bs_fix_y, power);
    #else
    raw_move1(x+bs_fix_x, y+bs_fix_y, power);
    #endif
}

void testRawPid(intptr_t unused)
{
    float input=posY(), output=0, setpoint=0;
    float kp=pidParams.Kp, ki=pidParams.Ki, kd=pidParams.Kd, ks=pidParams.Ks;
    MotionArgs motionArgs;
    MotionArgsPosition motionArgsPosition(&motionArgs);
    PID2 myPID(&input, &output, &setpoint, kp, ki, kd, ks, P_ON_E, DIRECT, &motionArgsPosition);
    
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(pidParams.sampleTime);
    myPID.SetOutputLimits(-100, 100);
    
    PidLog* log = &pidLog;
    log->counter = 0;
    
    calculateMotionArgs(&motionArgs, posY(), posY() + pidTestSize, speedmax, amax, amin);
    float end = motionArgs.t3 * MOTION_TIME_MULTIPLIER;
    float logInterval = end / LOG_MAX_SIZE;
    SYSTIM start;
    get_tim(&start);
    myPID.Start();
    
    float lastLog = -1000;
    while(get_time(start) < end)
    {
        input = posY();
        float time = get_time(start);
        //setpoint = position(&motionArgs, time);
        
        if (!myPID.Compute())
            continue;
        
        motorPower(y_motor, output);
        
        if (log->counter < LOG_MAX_SIZE && time - lastLog >= logInterval)
        {
            int c = log->counter;
            log->data[c].pos     = input;
            log->data[c].tgt_pos = setpoint;
            log->data[c].time    = time;
            log->data[c].power   = output;
            log->data[c].P       = myPID.GetP();
            log->data[c].I       = myPID.GetI();
            log->data[c].D       = myPID.GetD();
            log->data[c].S       = myPID.GetS();
            log->counter         = c + 1;
            
            lastLog = time;
        }
    }
    
    motorPower(y_motor, 0);
}

void control(intptr_t unused)
{
    char buf[100];
    
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    draw_title("Control", 0, 0, MENU_FONT);
    
    int xspeed = 0;
    int prevXSpeed = 0;
    int yspeed = 0;
    int prevYSpeed = 0;
    bool waitButtonRelease = false;
    
    while (1)
    {
        prevXSpeed = xspeed;
        prevYSpeed = yspeed;
        if (waitButtonRelease)
        {
            if (!hasAnyButtonPressed())
            {
                waitButtonRelease = false;
            }
        }
        else if (ev3_button_is_pressed(DOWN_BUTTON))
        {
            //if (yspeed <= -5 || yspeed >= 5)
                yspeed = MINVAL(100, yspeed + 5);
            //else
            //    yspeed++;
            waitButtonRelease = true;
        }
        else if (ev3_button_is_pressed(UP_BUTTON))
        {
            //if (yspeed <= -5 || yspeed >= 5)
                yspeed = MAXVAL(-100, yspeed - 5);
            //else
            //    yspeed--;
            waitButtonRelease = true;
        }
        else if (ev3_button_is_pressed(LEFT_BUTTON))
        {
            //if (xspeed <= -5 || xspeed >= 5)
                xspeed = MAXVAL(-100, xspeed - 5);
            //else
            //    xspeed--;
            waitButtonRelease = true;
        }
        else if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            //if (xspeed <= -5 || xspeed >= 5)
                xspeed = MINVAL(100, xspeed + 5);
            //else
            //    xspeed++;
            waitButtonRelease = true;
        }
        else if (ev3_button_is_pressed(ENTER_BUTTON))
        {
            if (xspeed != 0 || yspeed != 0)
            {
                xspeed = 0;
                yspeed = 0;
            }
            else if (posZ() > 20)
            {
                penDown();
            }
            else
            {
                penUp();
            }
            waitButtonRelease = true;
        }
        else if (ev3_button_is_pressed(BACK_BUTTON))
        {
            return;
        }
        
        if (yspeed > 0 && ev3_touch_sensor_is_pressed(yend_sensor))
        {
            yspeed = 0;
            ev3_motor_reset_counts(y_motor);
        }
        
        if (  (xspeed < 0 && posX() < 200 - xspeed)
           || (xspeed > 0 && posX() > X_MAX_POS - 200 - xspeed))
            xspeed = 0;
        
        if (  (yspeed < 0 && posY() < 200 - yspeed)
           || (yspeed > 0 && posY() > Y_MAX_POS - 200 - yspeed))
            yspeed = 0;
        
        if (xspeed != prevXSpeed)
        {
            motorX(xspeed);
        }
        
        if (yspeed != prevYSpeed)
        {
            motorY(yspeed);
        }
        
        sprintf(buf, "Vx %5d", xspeed);
        print(2, buf);
        sprintf(buf, "Vy %5d", yspeed);
        print(3, buf);
        
        sprintf(buf, " X %5ld", posX());
        print(4, buf);
        sprintf(buf, " Y %5ld", posY());
        print(5, buf);
        
        sprintf(buf, " Z %5ld", posZ());
        print(6, buf);
        
        // wait 10 mili-seconds
        tslp_tsk(10);
    }
}
