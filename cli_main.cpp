/*
 * cli_main.c
 *
 *  Created on: 08/Sep/2018
 *      Author: izanette
 */

#include "ev3api.h"
#include "app.h"
#include "cli_menu.h"
#include "gcodeparser.h"
#include "token.h"
#include "utils.h"
#include "selectfile.h"
#include "cli_menu.h"
#include "letters.h"
#include "motion.h"
#include <math.h>
#include <list>

#define NUM_PARAMS       15
#define TEST_PARAM       (NUM_PARAMS-2)
#define LOG_PARAM        (NUM_PARAMS-1)
#define NUM_LOG_DISPLAYS 6
#define NUM_TESTS        4

#define PI             3.141592653589793
#define GCODE_SCALE_X  (5.65*5.0)
#define GCODE_SCALE_Y  (5.65*5.0)

#define MAX_LINE_LEN  2
#define MIN_ANGLE     (3 * PI / 180)
#define EPSILON       0.00001
#define NON_VALUE     -999999.0

float logDisplay = 0.0;
float multiplier = 2.0;
float useMultiplier = 1.0;
float testType = 0.0;
float lastX = NON_VALUE;
float lastY = NON_VALUE;

const char* paramNames[] = {
    "Kp",
    "Ki",
    "Kd",
    "Ks",
    "use multi",
    "multi",
    "sampleTime",
    "accel",
    "SCurve",
    "speedmax",
    "testsize",
    "backlashX",
    "backlashY",
    "test",
    "log"
};

const char* testNames[] = {
    "back/fwd",
    "68",
    "circle",
    "star 7",
};

float paramInc[] = {
    0.10,
    0.02,
    0.02,
    0.02,
    1.00,
    0.10,
    1.00,
    10.0,
    10.0,
    25.0,
    100.0,
    1.00,
    1.00,
    1.00,
    1.00
};

static void drawCircle(intptr_t unused);
static void drawStar7(intptr_t unused);

void printCharAbs(int32_t x0, int32_t y0, char c, float scale)
{
    int i = 0;
    LetterPoint* lp = NULL;
    
    if (c >= '0' && c <= '9')
        lp = numbers[c - '0'];
    else if (c >= 'a' && c <= 'z')
        lp = letters[c - 'a'];
    else if (c >= 'A' && c <= 'Z')
        lp = letters[c - 'A'];
    else
        return;
    
    while(1)
    {
        int x = lp[i].x;
        int y = lp[i].y;
        
        if (x == -1)
        {
            if (y < 0)
            {
                penDown();
            }
            else if (y > 0)
            {
                penUp();
                if (y > 1) break;
            }
            i++;
            continue;
        }
        
        move(x0 + x * scale, y0 + y * scale, speedmax);
        
        i++;
    }
}

void printChars(const char* nums, float scale)
{
    int32_t x0 = posX();
    int32_t y0 = posY() + 4 * scale;
    
    int i = 0;
    while(nums[i] != 0)
    {
        char c = nums[i];
        printCharAbs(x0 + i * 4 * scale, y0, c, scale);
        i++;
    }
}

const char* getTestName(int test)
{
    return testNames[test];
}

const char* getParamName(int param)
{
    return paramNames[param];
}

float getParamValue(int param)
{
    switch(param)
    {
        case  0: return pidParams.Kp;
        case  1: return pidParams.Ki;
        case  2: return pidParams.Kd;
        case  3: return pidParams.Ks;
        case  4: return useMultiplier;
        case  5: return multiplier;
        case  6: return pidParams.sampleTime;
        case  7: return amax;
        case  8: return useSCurve;
        case  9: return speedmax;
        case 10: return pidTestSize;
        case 11: return backlashX;
        case 12: return backlashY;
        case 13: return testType;
        case 14: return logDisplay;
    }
    
    return 0.0;
}

float updateParameter(float& param, float multi, float inc)
{
    if (useMultiplier > 0.0)
        param *=  multi;
    else
        param += inc;
    
    return param;
}

float updateParam(int param, int u)
{
    float inc = paramInc[param];
    if (u < 0) inc = -inc;
    float multi = (u < 0) ? 1 / multiplier : multiplier;
    
    switch(param)
    {
        case  0: return updateParameter(pidParams.Kp, multi, inc); //return pidParams.Kp;
        case  1: return updateParameter(pidParams.Ki, multi, inc); //return pidParams.Ki;
        case  2: return updateParameter(pidParams.Kd, multi, inc); //return pidParams.Kd;
        case  3: return updateParameter(pidParams.Ks, multi, inc); //return pidParams.Ks;
        case  4: useMultiplier = 1.0 - useMultiplier; return useMultiplier;
        case  5: multiplier = multiplier + inc; if (multiplier < 1.0) multiplier = 1.0; return multiplier;
        case  6: pidParams.sampleTime = pidParams.sampleTime + inc; return pidParams.sampleTime;
        case  7: amax         = amax + inc; amin = amin + inc; return amax;
        case  8: useSCurve    = (useSCurve > 0.01) ? 0.0 : 1.0; return useSCurve;
        case  9: speedmax     = speedmax     + inc; return speedmax;
        case 10: return updateParameter(pidTestSize, multi, inc); //pidTestSize  = pidTestSize  + inc; return pidTestSize;
        case 11: backlashX += inc; return backlashX;
        case 12: backlashY += inc; return backlashY;
        case 13: testType     = ((int)(testType + inc + NUM_TESTS)) % NUM_TESTS; return testType;
        case 14: logDisplay   = ((int)(logDisplay + inc + NUM_LOG_DISPLAYS)) % NUM_LOG_DISPLAYS; return logDisplay;
    }
    
    return 0.0;
}

void drawPoint(int32_t x, int32_t y)
{
    ev3_lcd_draw_line(x, y, x, y);
}

void logChartPos()
{
    if (pidLog.counter == 0)
    {
        show_message_box("Log Chart Pos", "No data to display!");
        return;
    }
    
    float minv =  1000000;
    float maxv = -1000000;
    for(int i = 0; i < pidLog.counter; i++)
    {
        if (minv > pidLog.data[i].pos)
        {
            minv = pidLog.data[i].pos;
        }
        if (minv > pidLog.data[i].tgt_pos)
        {
            minv = pidLog.data[i].tgt_pos;
        }
        if (maxv < pidLog.data[i].pos)
        {
            maxv = pidLog.data[i].pos;
        }
        if (maxv < pidLog.data[i].tgt_pos)
        {
            maxv = pidLog.data[i].tgt_pos;
        }
    }
    
    if (maxv - minv == 0)
    {
        show_message_box("Log Chart Pos", "Data is inconsistent!");
        return;
        //float d = EV3_LCD_HEIGHT / 2;
        //maxv += d;
        //minv -= d;
    }
    
    float hscale = ((float) EV3_LCD_WIDTH) / pidLog.counter;
    float vscale = ((float) EV3_LCD_HEIGHT - MENU_FONT_HEIGHT) / (maxv - minv);
    
    for(int i = 0; i < pidLog.counter; i++)
    {
        int x  = (int) (hscale * i);
        int y  = (int) ((pidLog.data[i].pos     - minv) * vscale);
        int yt = (int) ((pidLog.data[i].tgt_pos - minv) * vscale);
        
        drawPoint(x, EV3_LCD_HEIGHT - MENU_FONT_HEIGHT - y);
        drawPoint(x, EV3_LCD_HEIGHT - MENU_FONT_HEIGHT - yt);
    }
    
    char buf[50];
    sprintf(buf, "Pos %.2f %.2f", minv, maxv);
    print(7, buf);
}

void logChartError()
{
    if (pidLog.counter == 0)
    {
        show_message_box("Log Chart Error", "No data to display!");
        return;
    }
    
    float minv = 1000000;
    float maxv = -1000000;
    for(int i = 0; i < pidLog.counter; i++)
    {
        float error = fabs(pidLog.data[i].tgt_pos - pidLog.data[i].pos);
        if (minv > error)
        {
            minv = error;
        }
        if (maxv < error)
        {
            maxv = error;
        }
        //if (minv > pidLog.data[i].tgt_pos)
        //{
        //    minv = pidLog.data[i].tgt_pos;
        //}
        //if (maxv < pidLog.data[i].tgt_pos)
        //{
        //    maxv = pidLog.data[i].tgt_pos;
        //}
    }
    
    if (maxv - minv == 0)
    {
        show_message_box("Log Chart error", "Data is inconsistent!");
        return;
    }
    
    float hscale = ((float) EV3_LCD_WIDTH) / pidLog.counter;
    float vscale = (EV3_LCD_HEIGHT /*- MENU_FONT_HEIGHT*/) / (maxv - minv);
    float sumError = 0;
    
    for(int i = 0; i < pidLog.counter; i++)
    {
        int x = (int) (hscale * i);
        float error = (pidLog.data[i].tgt_pos - pidLog.data[i].pos);
        sumError += fabs(error);
        int y = (int) ((error) * vscale) + (EV3_LCD_HEIGHT /*- MENU_FONT_HEIGHT*/) / 2;
        int yt = (int) ((pidLog.data[i].tgt_pos - minv) * vscale);
        
        drawPoint(x, EV3_LCD_HEIGHT /*- MENU_FONT_HEIGHT*/ - y);
        drawPoint(x, EV3_LCD_HEIGHT /*- MENU_FONT_HEIGHT*/ - yt);
        drawPoint(x, (EV3_LCD_HEIGHT /*- MENU_FONT_HEIGHT*/) / 2);
    }
    
    char buf[50];
    sprintf(buf, "Sum %.2f", sumError / 1000.0);
    print(6, buf);
    sprintf(buf, "Err %.2f %.2f", minv, maxv);
    print(7, buf);
}

float getPidParamValue(int param, PidLogData* data)
{
    switch(param)
    {
        case 0: return data->P;
        case 1: return data->I;
        case 2: return data->D;
        case 3: return data->S;
        case 4: return fabs(data->tgt_pos - data->pos);
    }
    
    return 0.0;
}

const char* getPidChartName(int param)
{
    switch(param)
    {
        case 0: return "P";
        case 1: return "I";
        case 2: return "D";
        case 3: return "S";
        case 4: return "Err";
    }
    
    return "?";
}

void logChartPID(int param)
{
    if (pidLog.counter == 0)
    {
        show_message_box("Log Chart PID", "No data to display!");
        return;
    }
    
    float minv  =  100000000;
    float maxv  = -100000000;
    float tminv =  100000000;
    float tmaxv = -100000000;
    for(int i = 0; i < pidLog.counter; i++)
    {
        float val = getPidParamValue(param, &pidLog.data[i]);
        if (minv > val)
        {
            minv = val;
        }
        if (maxv < val)
        {
            maxv = val;
        }
        if (tminv > pidLog.data[i].tgt_pos)
        {
            tminv = pidLog.data[i].tgt_pos;
        }
        if (tmaxv < pidLog.data[i].tgt_pos)
        {
            tmaxv = pidLog.data[i].tgt_pos;
        }
    }
    
    if (0.0 < minv)
    {
        minv = 0.0;
    }
    
    if (maxv < 0.0)
    {
        maxv = 0.0;
    }
    
    if (maxv - minv == 0)
    {
        //show_message_box("Log Chart PID", "Data is inconsistent!");
        //return;
        float d = (EV3_LCD_HEIGHT - MENU_FONT_HEIGHT) / 2;
        maxv += d;
        minv -= d;
    }
    
    if (tmaxv - tminv == 0)
    {
        float d = (EV3_LCD_HEIGHT - MENU_FONT_HEIGHT) / 2;
        tmaxv += d;
        tminv -= d;
    }
    
    float hscale  = ((float) EV3_LCD_WIDTH) / pidLog.counter;
    float vscale  = (EV3_LCD_HEIGHT - MENU_FONT_HEIGHT) / (maxv - minv) * .95;
    float tvscale = (EV3_LCD_HEIGHT - MENU_FONT_HEIGHT) / (tmaxv - tminv);
    
    int zero = (int) (((0.0 - minv) * vscale) /*+ (EV3_LCD_HEIGHT - MENU_FONT_HEIGHT) / 2*/);
    float sumVal = 0.0;
    for(int i = 0; i < pidLog.counter; i++)
    {
        int x = (int) (hscale * i);
        float val = getPidParamValue(param, &pidLog.data[i]);
        sumVal += val;
        int y = (int) (((val - minv) * vscale) /*+ (EV3_LCD_HEIGHT - MENU_FONT_HEIGHT) / 2*/);
        int yt = (int) ((pidLog.data[i].tgt_pos - tminv) * tvscale);
        
        drawPoint(x, EV3_LCD_HEIGHT - MENU_FONT_HEIGHT - y);
        drawPoint(x, EV3_LCD_HEIGHT - MENU_FONT_HEIGHT - yt);
        drawPoint(x, EV3_LCD_HEIGHT - MENU_FONT_HEIGHT - zero);
    }
    
    char buf[50];
    const char* chartName = getPidChartName(param);
    if (param == 4)
    {
        sprintf(buf, "%s %.2f %.2f", chartName, sumVal / 1000.0, maxv);
    }
    else
    {
        sprintf(buf, "%s %.2f %.2f", chartName, minv, maxv);
    }
    print(7, buf);
}

void logChart(int chart)
{
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    
    switch(chart)
    {
        case 0: logChartPos(); return;
        //case 1: logChartError(); return;
        case 1: logChartPID(4); return;
        case 2: logChartPID(0); return;
        case 3: logChartPID(1); return;
        case 4: logChartPID(2); return;
        case 5: logChartPID(3); return;
    }
}

void findTestPosition(float sizeX, float sizeY)
{
    if (posX() > X_MAX_POS - sizeX)
    {
        if (posY() > Y_MAX_POS - sizeX)
            move(100, 100, speedmax);
        else
            move(100, posY() + sizeY, speedmax);
    }
}

void doTestBackFwd()
{
    rotateAbsX(500, 30);
    rotateAbsY(500, 30);
    tslp_tsk(1000);
    
    move(500 + (int)pidTestSize, 500 + (int)(pidTestSize/10.0), speedmax);
    tslp_tsk(1000);
    move(500, 500, speedmax);
}

void doTest68()
{
    printChars("68", 50);
    findTestPosition(250, -100);
}

void doTestCircle()
{
    drawCircle(0);
    findTestPosition(pidTestSize, pidTestSize);
}

void doTestStar7()
{
    drawStar7(0);
    findTestPosition(pidTestSize, pidTestSize);
}

void doTest(int test)
{
    switch(test)
    {
    case 0: doTestBackFwd(); return;
    case 1: doTest68();      return;
    case 2: doTestCircle();  return;
    case 3: doTestStar7();   return;
    default:
        return;
    }
}

static void configPid(intptr_t unused) {
    char buf[50];
    
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    draw_title("Config PID", 0, 0, MENU_FONT);
    
    bool waitButtonRelease = false;
    int param = 0;
    int prevParam = 1;
    float value = 0.0;
    float prevValue = 0.0;
    while (1)
    {
        if (waitButtonRelease)
        {
            if (!hasAnyButtonPressed())
            {
                waitButtonRelease = false;
            }
        }
        else if (ev3_button_is_pressed(DOWN_BUTTON))
        {
            value = updateParam(param, -1);
            waitButtonRelease = true;
        }
        else if (ev3_button_is_pressed(UP_BUTTON))
        {
            value = updateParam(param, +1);
            waitButtonRelease = true;
        }
        else if (ev3_button_is_pressed(LEFT_BUTTON))
        {
            param = (param + NUM_PARAMS - 1) % NUM_PARAMS;
            waitButtonRelease = true;
        }
        else if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            param = (param + 1) % NUM_PARAMS;
            waitButtonRelease = true;
        }
        else if (ev3_button_is_pressed(ENTER_BUTTON))
        {
            doTest(testType);
            
            waitButtonRelease = true;
        }
        else if (ev3_button_is_pressed(BACK_BUTTON))
        {
            return;
        }
        
        if (param != prevParam || value != prevValue)
        {
            value = getParamValue(param);
            
            if (param == LOG_PARAM)
            {
                logChart(value);
            }
            else
            {
                ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
                draw_title("Config PID", 0, 0, MENU_FONT);
            
                const char* name = getParamName(param);
                if (param == TEST_PARAM)
                {
                    const char* testName = getTestName((int)value);
                    sprintf(buf, "%s %s", name, testName);
                }
                else
                    sprintf(buf, "%s %.4f", name, value);
                print(7, buf);
            }
            
            prevParam = param;
            prevValue = value;
        }
        
        tslp_tsk(pidParams.sampleTime);
    }
}

static void testBacklashX(intptr_t unused) {
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    draw_title("Backlash X", 0, 0, MENU_FONT);
    
    int x = pidTestSize;
    
    // horizontal backlash
    penUp();
    for(int i = -2; i <= 2; i++)
    {
        move(x +  500, 500 + i * 125, speedmax);
        move(x + 1000, 500 + i * 125, speedmax);
        
        penDown();
        move(x + 1100, 500 + i * 125, speedmax);
        move(x + 1100, 550 + i * 125, speedmax);
        //move(x + 1100, 450 + i * 125, speedmax);
        penUp();
        
        move(x + 1500, 500 + i * 125, speedmax);
        move(x + 1200, 500 + i * 125, speedmax);
        
        penDown();
        move(x + 1100 - i * 5, 500 + i * 125, speedmax);
        //move(x + 1100 - i * 5, 550 + i * 125, speedmax);
        move(x + 1100 - i * 5, 450 + i * 125, speedmax);
        penUp();
    }
}

static void testBacklashY(intptr_t unused) {
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    draw_title("Backlash Y", 0, 0, MENU_FONT);
    
    int y = pidTestSize;
    
    // vertical backlash
    penUp();
    for(int i = -2; i <= 2; i++)
    {
        move(500 + i * 125, y +  500, speedmax);
        move(500 + i * 125, y + 1000, speedmax);
        
        penDown();
        move(500 + i * 125, y + 1100, speedmax);
        move(550 + i * 125, y + 1100, speedmax);
        //move(450 + i * 125, y + 1100, speedmax);
        penUp();
        
        move(500 + i * 125, y + 1500, speedmax);
        move(500 + i * 125, y + 1200, speedmax);
        
        penDown();
        move(500 + i * 125, y + 1100 - i * 5, speedmax);
        //move(550 + i * 125, y + 1100 - i * 5, speedmax);
        move(450 + i * 125, y + 1100 - i * 5, speedmax);
        penUp();
    }
}

static void testPid(intptr_t unused) {
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    draw_title("Test PID", 0, 0, MENU_FONT);
    
    //move(100, 100, 10);
    rotateAbsX(500, 30);
    rotateAbsY(500, 30);
    //ev3_speaker_play_tone(NOTE_C4, 100);
    tslp_tsk(1000);
    
    //rotateAbsX(200, 10);
    //rotateAbsY(200, 10);
    //rotateAbsX(100, 10);
    //rotateAbsY(100, 10);
    //ev3_speaker_play_tone(NOTE_C4, 100);
    //tslp_tsk(2000);
    //
    int s = (int)pidTestSize;
    move(500+s, 500, speedmax);
    //ev3_speaker_play_tone(NOTE_C4, 100);
    tslp_tsk(1000);
    
    move(500+s, 500+2*s, speedmax);
    //ev3_speaker_play_tone(NOTE_C4, 100);
    tslp_tsk(1000);
    
    //move(100, 100+s, speedmax);
    //ev3_speaker_play_tone(NOTE_C4, 100);
    //tslp_tsk(1000);
    
    move(500, 500, speedmax);
    //ev3_speaker_play_tone(NOTE_C4, 100);
}

void testMotorRunX(int speed, int limit)
{
    char buf[100];
    clearScreen();
    sprintf(buf, "Speed %d", speed);
    print(1, buf);
    
    rotateAbsX(500, 30);
    tslp_tsk(400);
    
    int32_t posS = posX();
    motorX(speed);
    
    SYSTIM start;
    get_tim(&start);
    
    while(posX() < limit)
        tslp_tsk(10);
    
    int32_t posE = posX();
    float total = get_time(start);
    
    motorX(0);
    
    sprintf(buf, "Deg/s = %.2f", (posE - posS) / total);
    print(2, buf);
    
    waitButtonPressed();
}

void testMotorRunY(int speed, int limit)
{
    char buf[100];
    clearScreen();
    sprintf(buf, "Speed %d", speed);
    print(1, buf);
    
    rotateAbsY(500, 30);
    tslp_tsk(400);
    
    int32_t posS = posY();
    motorY(speed);
    
    SYSTIM start;
    get_tim(&start);
    
    while(posY() < limit)
        tslp_tsk(10);
    
    int32_t posE = posY();
    float total = get_time(start);
    
    motorY(0);
    
    sprintf(buf, "Deg/s = %.2f", (posE - posS) / total);
    print(2, buf);
    
    waitButtonPressed();
}

void testMotorRun(int speed, int limit)
{
    testMotorRunX(speed, limit);
}

static void testMotor(intptr_t unused) {
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    draw_title("Test Motor", 0, 0, MENU_FONT);
    
    for(int s = 1; s <= 4; s++)
    {
        testMotorRun(s, 800);
    }
    
    for(int s = 6; s <= 8; s+=2)
    {
        testMotorRun(s, 1200);
    }
    
    for(int s = 10; s <= 100; s+=10)
    {
        testMotorRun(s, 1750);
    }
}

static void dummy(intptr_t unused)
{
    show_message_box("Dummy Handler", "I do nothing!");
}

static void executeGCodeMoveZ(Command* cmd)
{
    // check if we need to move the pen up or down
    int idx = cmd->getParameterTypeIndex(ParameterZT);
    if (idx != -1)
    {
        float value = cmd->getParameter(idx)->getValue();
        if (value < 0) penDown();
        else penUp();
    }
}

// float equal
static bool fequal(float f1, float f2)
{
    float d = f2-f1;
    return (d >= -EPSILON && d <= EPSILON);
}

// float greater than or equal
static bool fge(float f1, float f2)
{
    return ((f1-f2) >= -EPSILON);
}

static float pos_angle(float rad)
{
    if (fge(rad, 2*PI))
    {
        if (rad < 4*PI)
            return (rad - 2*PI);
        else
            return rad - floor(rad / 2*PI) * 2*PI;
    }
    
    if (rad < -EPSILON)
    {
        if  (rad >= -2*PI)
            return (rad + 2*PI);
        else
            return rad - ceil(rad / 2*PI) * 2*PI;
    }   
    return rad;
}

//static float deg(float rad)
//{
//    return rad*180/PI;
//}

static void g2g3tog1(std::list<std::pair<float, float>>& points, float x0, float y0, float x1, float y1, float i, float j, bool clockwise)
{
    float cx = x0 + i;
    float cy = y0 + j;
    float r  = sqrt((x0 - cx) * (x0 - cx) + (y0 - cy) * (y0 - cy));
    float a0 = pos_angle(atan2((y0 - cy), (x0 - cx)));
    float a1 = pos_angle(atan2((y1 - cy), (x1 - cx)));
    float da = (clockwise) ? pos_angle(a0 - a1) : pos_angle(a1 - a0);
    
    float arclen = r * da;
    
    int numseg = (int) std::min(ceil(arclen / MAX_LINE_LEN), ceil(da / MIN_ANGLE));
    
    float a_inc = (clockwise) ? -da / numseg : da / numseg;
    
    float a = a0;
    while(!fequal(a, a1) && points.size() < 50)
    {
        a = pos_angle(a + a_inc);
        float x = cos(a) * r + cx;
        float y = sin(a) * r + cy;
        points.push_back(std::pair<float, float>(x, y));
    }
    
    //if (points.size() == 50)
    //{
    //    char buf[50];
    //    clearScreen();
    //    sprintf(buf, "G2 / G3");
    //    print(0, buf);
    //    sprintf(buf, "r %.3f", r);                     print(3, buf);
    //    sprintf(buf, "a %.3f %.3f", deg(a0), deg(a1)); print(4, buf);
    //    sprintf(buf, "da %.3f", deg(da));              print(5, buf);
    //    sprintf(buf, "numseg %d", numseg);             print(6, buf);
    //    sprintf(buf, "a_inc %.3f", deg(a_inc));        print(7, buf);
    //    ev3_speaker_play_tone(NOTE_C4, 50);
    //    waitEnterButtonPressed();
    //    waitNoButtonPressed();
    //    
    //    clearScreen();
    //    
    //    float a = a0;
    //    int l = 0;
    //    while(!fequal(a, a1) && l < 8)
    //    {
    //        a = pos_angle(a + a_inc);
    //        sprintf(buf, "%d - %.3f", l, deg(a));
    //        print(l, buf);
    //        l++;
    //    }
    //    
    //    waitEnterButtonPressed();
    //    waitNoButtonPressed();
    //}
}

static void executeGCodeMoveG2G3(Command* cmd, bool clockwise)
{
    executeGCodeMoveZ(cmd);
    
    int idxI = cmd->getParameterTypeIndex(ParameterIT);
    int idxJ = cmd->getParameterTypeIndex(ParameterJT);
    
    if (idxI == -1 || idxJ == -1)
        return;
    
    int idxX = cmd->getParameterTypeIndex(ParameterXT);
    int idxY = cmd->getParameterTypeIndex(ParameterYT);
    
    float x = (idxX != -1) ? cmd->getParameter(idxX)->getValue() : lastX;
    float y = (idxY != -1) ? cmd->getParameter(idxY)->getValue() : lastY;
    
    if (x == NON_VALUE || y == NON_VALUE)
        return;
    
    float i = cmd->getParameter(idxI)->getValue();
    float j = cmd->getParameter(idxJ)->getValue();
    
    std::list<std::pair<float, float>> points;
    g2g3tog1(points, lastX, lastY, x, y, i, j, clockwise);
    
    for (auto it = points.begin(); it != points.end(); it++)
    {
        float x0 = it->first;
        float y0 = it->second;
        move(x0 * GCODE_SCALE_X, Y_MAX_POS - y0 * GCODE_SCALE_Y, speedmax);
    }
    lastX = x;
    lastY = y;
}

static void executeGCodeMoveG0G1(Command* cmd)
{
    executeGCodeMoveZ(cmd);
    
    int idxX = cmd->getParameterTypeIndex(ParameterXT);
    int idxY = cmd->getParameterTypeIndex(ParameterYT);
    float x = (idxX != -1) ? cmd->getParameter(idxX)->getValue() : lastX;
    float y = (idxY != -1) ? cmd->getParameter(idxY)->getValue() : lastY;
    
    if (x == NON_VALUE || y == NON_VALUE)
        return;
    
    move(x * GCODE_SCALE_X, Y_MAX_POS - y * GCODE_SCALE_Y, speedmax);
    lastX = x;
    lastY = y;
}

static void executeGCode(Command* cmd)
{
    switch(cmd->getCommandType())
    {
    case CommandG00T:
    case CommandG01T:
        executeGCodeMoveG0G1(cmd);
        break;
    case CommandG02T:
        executeGCodeMoveG2G3(cmd, true);
        break;
    case CommandG03T:
        executeGCodeMoveG2G3(cmd, false);
        break;
    default:
        break;
    }
}

static void drawFromFile(intptr_t unused)
{
    char buf[50];
    char path[100];
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    
    const char *inputPath = "/gcode";
    
    SelectFile sel(inputPath);
    fileinfo_t* fi = sel.select();
    
    if (fi == NULL) return;
    
    clearScreen();
    draw_title("File", 0, 0, MENU_FONT);
    sprintf(path, "%s/%s", inputPath, fi->name);
    GCodeParser parser(path);

    Command* command = parser.getNextCommand();
    while (command)
    {
        char s[50];
        sprintf(buf, "%s", command->toString(s));
        print(1, buf);
        for (int i = 0; i < command->getParameterNumber(); i++)
        {
            Parameter* parameter = command->getParameter(i);
            sprintf(buf, " %s", parameter->toString(s));
            print(i+2, buf);
        }
        
        executeGCode(command);
        
        delete command;
        command = parser.getNextCommand();
        
        //waitButtonPressed();
        
        ev3_lcd_fill_rect(0, 14, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    }
    
    penUp();
    move(0, 0, speedmax);
    print(2, "Finished");
}

static void drawPlott3r(intptr_t unused)
{
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    draw_title("Plott3r", 0, 0, MENU_FONT);
    
    printChars("PLOTT3R v03", 60);
}

static void drawExpo(intptr_t unused)
{
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    draw_title("Expo LEGO", 0, 0, MENU_FONT);
    
    printChars("EXPO LUG 2018", 60);
}

static void drawNumbers(intptr_t unused)
{
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    draw_title("Numbers", 0, 0, MENU_FONT);
    
    printChars("0123456789", 50);
}

static void drawLetters1(intptr_t unused)
{
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    draw_title("Letters 1", 0, 0, MENU_FONT);
    
    printChars("abcdefghijklm", 50);
}

static void drawLetters2(intptr_t unused)
{
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    draw_title("Letters 2", 0, 0, MENU_FONT);
    
    printChars("nopqrstuvwxyz", 50);
}

static void drawPoly(int num_vertex, float turn)
{
    char buf[20];
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    sprintf(buf, "Poly %d", num_vertex);
    draw_title(buf, 0, 0, MENU_FONT);
    
    float radius = pidTestSize / 2.0;
    float cx = posX() + radius;
    float cy = posY() + radius;
    
    penUp();
    
    float x = cx + radius * cos(turn);
    float y = cy + radius * sin(turn);
    move(x, y, speedmax);
    
    penDown();
    int j = 0;
    for(int i = 0; i < num_vertex; i++)
    {
        j = (j + 1) % num_vertex;
        x = cx + radius * cos(j * 2 * PI / num_vertex + turn);
        y = cy + radius * sin(j * 2 * PI / num_vertex + turn);
        move(x, y, speedmax);
    }
    
    penUp();
    
    move(cx + radius, cy - radius, speedmax);
}

static void drawCircle(intptr_t unused)
{
    const float MAX_CIRCLE_SEQ = 100.0;
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    draw_title("Circle", 0, 0, MENU_FONT);
    
    float radius = pidTestSize / 2.0;
    float circ = 2.0 * PI * radius;
    int num_vertex = (int) ceil(circ / MAX_CIRCLE_SEQ);
    
    if (num_vertex < 10) num_vertex = 10;
    
    drawPoly(num_vertex, 0.0);
}

static void drawPoly3 (intptr_t unused) { drawPoly( 3, PI/2); }
static void drawPoly4 (intptr_t unused) { drawPoly( 4, PI/2); }
static void drawPoly5 (intptr_t unused) { drawPoly( 5, PI/2); }
static void drawPoly6 (intptr_t unused) { drawPoly( 6, PI/2); }
static void drawPoly7 (intptr_t unused) { drawPoly( 7, PI/2); }
static void drawPoly8 (intptr_t unused) { drawPoly( 8, PI/2); }
//static void drawPoly10(intptr_t unused) { drawPoly(10, PI/2); }
static void drawPoly12(intptr_t unused) { drawPoly(12, PI/2); }

static void drawPolygonMenu(intptr_t unused)
{
    static const CliMenuEntry entry_tab[] = {
        { .key = '1', .title = "Triangle",  .handler = drawPoly3  },
        { .key = '1', .title = "Quad"    ,  .handler = drawPoly4  },
        { .key = '1', .title = "Pentagon",  .handler = drawPoly5  },
        { .key = '1', .title = "Hexagon",   .handler = drawPoly6  },
        { .key = '1', .title = "Heptagon",  .handler = drawPoly7  },
        { .key = '1', .title = "Octagon",   .handler = drawPoly8  },
        //{ .key = '1', .title = "Decagon",   .handler = drawPoly10 },
        { .key = '1', .title = "Dodecagon", .handler = drawPoly12 },
        { .key = 'Q', .title = "Cancel",    .handler = dummy, .exinf = -1 }
    };

    static const CliMenu climenu = {
        .title     = "Star",
        .entry_tab = entry_tab,
        .entry_num = sizeof(entry_tab) / sizeof(CliMenuEntry),
    };

    const CliMenuEntry* cme = NULL;
    while(cme == NULL) {
        show_cli_menu(&climenu, 0, MENU_FONT_HEIGHT * 0, MENU_FONT);
        cme = select_menu_entry(&climenu, 0, MENU_FONT_HEIGHT * 1, MENU_FONT);
    }

    if(cme->exinf == -1) return;

    assert(cme->handler != NULL);
    cme->handler(cme->exinf);
}

static void drawStarOdd(int num_vertex, float turn)
{
    char buf[20];
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    sprintf(buf, "Star %d", num_vertex);
    draw_title(buf, 0, 0, MENU_FONT);
    
    float radius = pidTestSize / 2.0;
    float cx = posX() + radius;
    float cy = posY() + radius;
    
    penUp();
    
    float x = cx + radius * cos(turn);
    float y = cy + radius * sin(turn);
    move(x, y, speedmax);
    
    penDown();
    int jump = num_vertex / 2;
    int j = 0;
    for(int i = 0; i < num_vertex; i++)
    {
        j = (j + jump) % num_vertex;
        x = cx + radius * cos(j * 2 * PI / num_vertex + turn);
        y = cy + radius * sin(j * 2 * PI / num_vertex + turn);
        move(x, y, speedmax);
    }
    
    penUp();
}

static void drawStarTurn(int num_vertex, float turn)
{
    float x = posX();
    float y = posY();
    if (num_vertex % 2 == 1)
    {
        drawStarOdd(num_vertex, turn);
    }
    else if (num_vertex == 4)
    {
        drawPoly(num_vertex, turn);
    }
    else
    {
        drawStarTurn(num_vertex/2, turn);
        move(x, y, speedmax);
        drawStarTurn(num_vertex/2, turn + 2*PI/num_vertex);
    }
}

static void drawStar(int num_vertex)
{
    float x = posX();
    float y = posY();
    drawStarTurn(num_vertex, PI/2);
    
    move(x + pidTestSize, y, speedmax);
}

static void drawStar5 (intptr_t unused) { drawStar( 5); }
static void drawStar7 (intptr_t unused) { drawStar( 7); }
static void drawStar9 (intptr_t unused) { drawStar( 9); }
static void drawStar11(intptr_t unused) { drawStar(11); }
static void drawStar13(intptr_t unused) { drawStar(13); }

static void drawStar6 (intptr_t unused) { drawStar( 6); }
static void drawStar8 (intptr_t unused) { drawStar( 8); }
static void drawStar10(intptr_t unused) { drawStar(10); }
static void drawStar12(intptr_t unused) { drawStar(12); }
static void drawStar14(intptr_t unused) { drawStar(14); }

static void drawPointyStarsMenu(intptr_t unused)
{
    static const CliMenuEntry entry_tab[] = {
        { .key = '1', .title = "Star 10", .handler = drawStar10 },
        { .key = '1', .title = "Star 11", .handler = drawStar11 },
        { .key = '1', .title = "Star 12", .handler = drawStar12 },
        { .key = '1', .title = "Star 13", .handler = drawStar13 },
        { .key = '1', .title = "Star 14", .handler = drawStar14 },
        { .key = 'Q', .title = "Cancel",  .handler = dummy, .exinf = -1 },
    };

    static const CliMenu climenu = {
        .title     = "Star 2",
        .entry_tab = entry_tab,
        .entry_num = sizeof(entry_tab) / sizeof(CliMenuEntry),
    };

    const CliMenuEntry* cme = NULL;
    while(cme == NULL) {
        show_cli_menu(&climenu, 0, MENU_FONT_HEIGHT * 0, MENU_FONT);
        cme = select_menu_entry(&climenu, 0, MENU_FONT_HEIGHT * 1, MENU_FONT);
    }

    if(cme->exinf == -1) return;

    assert(cme->handler != NULL);
    cme->handler(cme->exinf);
}

static void drawStarMenu(intptr_t unused)
{
    static const CliMenuEntry entry_tab[] = {
        { .key = '1', .title = "Star 5",  .handler = drawStar5 },
        { .key = '1', .title = "Star 6",  .handler = drawStar6 },
        { .key = '1', .title = "Star 7",  .handler = drawStar7 },
        { .key = '1', .title = "Star 8",  .handler = drawStar8 },
        { .key = '1', .title = "Star 9",  .handler = drawStar9  },
        { .key = '1', .title = "more",    .handler = drawPointyStarsMenu },
        { .key = 'Q', .title = "Cancel",  .handler = dummy, .exinf = -1 },
    };

    static const CliMenu climenu = {
        .title     = "Star",
        .entry_tab = entry_tab,
        .entry_num = sizeof(entry_tab) / sizeof(CliMenuEntry),
    };

    const CliMenuEntry* cme = NULL;
    while(cme == NULL) {
        show_cli_menu(&climenu, 0, MENU_FONT_HEIGHT * 0, MENU_FONT);
        cme = select_menu_entry(&climenu, 0, MENU_FONT_HEIGHT * 1, MENU_FONT);
    }

    if(cme->exinf == -1) return;

    assert(cme->handler != NULL);
    cme->handler(cme->exinf);
}

static void shapes(intptr_t unused) {
    static const CliMenuEntry entry_tab[] = {
        { .key = '1', .title = "Star",      .handler = drawStarMenu },
        { .key = '2', .title = "Polygon",   .handler = drawPolygonMenu },
        { .key = '3', .title = "Circle",    .handler = drawCircle },
        { .key = 'Q', .title = "Cancel",    .handler = dummy, .exinf = -1 }
    };

    static const CliMenu climenu = {
        .title     = "Shapes",
        .entry_tab = entry_tab,
        .entry_num = sizeof(entry_tab) / sizeof(CliMenuEntry),
    };

    const CliMenuEntry* cme = NULL;
    while(cme == NULL) {
        show_cli_menu(&climenu, 0, MENU_FONT_HEIGHT * 0, MENU_FONT);
        cme = select_menu_entry(&climenu, 0, MENU_FONT_HEIGHT * 1, MENU_FONT);
    }

    if(cme->exinf == -1) return;

    assert(cme->handler != NULL);
    cme->handler(cme->exinf);
}

static void drawText(intptr_t unused) {
    static const CliMenuEntry entry_tab[] = {
        { .key = '1', .title = "Expo",      .handler = drawExpo },
        { .key = '2', .title = "Plott3r",   .handler = drawPlott3r },
        { .key = '3', .title = "Numbers",   .handler = drawNumbers },
        { .key = '4', .title = "Letters 1", .handler = drawLetters1 },
        { .key = '5', .title = "Letters 2", .handler = drawLetters2 },
        { .key = 'Q', .title = "Cancel",    .handler = dummy, .exinf = -1 }
    };

    static const CliMenu climenu = {
        .title     = "Shapes",
        .entry_tab = entry_tab,
        .entry_num = sizeof(entry_tab) / sizeof(CliMenuEntry),
    };

    const CliMenuEntry* cme = NULL;
    while(cme == NULL) {
        show_cli_menu(&climenu, 0, MENU_FONT_HEIGHT * 0, MENU_FONT);
        cme = select_menu_entry(&climenu, 0, MENU_FONT_HEIGHT * 1, MENU_FONT);
    }

    if(cme->exinf == -1) return;

    assert(cme->handler != NULL);
    cme->handler(cme->exinf);
}

static void tests(intptr_t unused) {
    static const CliMenuEntry entry_tab[] = {
        { .key = '1', .title = "Config PID", .handler = configPid },
        { .key = '2', .title = "Test PID",   .handler = testPid },
        { .key = '3', .title = "Backlash X", .handler = testBacklashX },
        { .key = '4', .title = "Backlash Y", .handler = testBacklashY },
        { .key = '5', .title = "Test Motor", .handler = testMotor },
        { .key = 'Q', .title = "Cancel",     .handler = dummy, .exinf = -1 }
    };

    static const CliMenu climenu = {
        .title     = "Shapes",
        .entry_tab = entry_tab,
        .entry_num = sizeof(entry_tab) / sizeof(CliMenuEntry),
    };

    const CliMenuEntry* cme = NULL;
    while(cme == NULL) {
        show_cli_menu(&climenu, 0, MENU_FONT_HEIGHT * 0, MENU_FONT);
        cme = select_menu_entry(&climenu, 0, MENU_FONT_HEIGHT * 1, MENU_FONT);
    }

    if(cme->exinf == -1) return;

    assert(cme->handler != NULL);
    cme->handler(cme->exinf);
}

static const CliMenuEntry entry_tab[] = {
    { .key = '1', .title = "Control",    .handler = control },
    { .key = '2', .title = "Homing",     .handler = homing },
    { .key = '3', .title = "Shapes",     .handler = shapes },
    { .key = '4', .title = "Text",       .handler = drawText },
    { .key = '5', .title = "Print File", .handler = drawFromFile },
    { .key = '6', .title = "Tests",      .handler = tests },
};

const CliMenu climenu_main = {
    .title     = "PLOTT3R v03",
    .entry_tab = entry_tab,
    .entry_num = sizeof(entry_tab) / sizeof(CliMenuEntry),
};

