/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/
#include "ev3api.h"
#include <PID_v1.h>

int32_t millis()
{
    SYSTIM now;
    get_tim(&now);
    return now;
}

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(float* Input, float* Output, float* Setpoint,
         float Kp, float Ki, float Kd, int POn, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    PID::SetOutputLimits(0, 255);               //default output limit corresponds to
                                                //the arduino pwm limits

    SampleTime = 100;                           //default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd, POn);

    lastTime = millis()-SampleTime;
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PID::PID(float* Input, float* Output, float* Setpoint,
         float Kp, float Ki, float Kd, int ControllerDirection)
    :PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection)
{

}

float PID::Compute2(float input, float setpoint)
{
    *myInput = input;
    *mySetpoint = setpoint;
    
    Compute();
    
    return *myOutput;
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange < SampleTime)
       return false;
   
   /*Compute all the working error variables*/
   float input = *myInput;
   float error = *mySetpoint - input;
   float dInput = (input - lastInput);
   outputSum+= (ki * error);

   /*Add Proportional on Measurement, if P_ON_M is specified*/
   if(!pOnE) outputSum-= kp * dInput;

   if(outputSum > outMax) outputSum= outMax;
   else if(outputSum < outMin) outputSum= outMin;

   I = outputSum;
   
   /*Add Proportional on Error, if P_ON_E is specified*/
   float output;
   P = kp * error;
   if(pOnE) output = P;
   else output = 0;

   /*Compute Rest of PID Output*/
   D = kd * dInput;
   output += outputSum - D;

   if(output > outMax) output = outMax;
   else if(output < outMin) output = outMin;
   *myOutput = output;

   /*Remember some variables for next time*/
   lastInput = input;
   lastTime = now;
   return true;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(float Kp, float Ki, float Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   float SampleTimeInSec = ((float)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

  if(controllerDirection == REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID::SetTunings(float Kp, float Ki, float Kd){
    SetTunings(Kp, Ki, Kd, pOn); 
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = (float)NewSampleTime
                      / (float)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(float Min, float Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
      if(*myOutput > outMax) *myOutput = outMax;
      else if(*myOutput < outMin) *myOutput = outMin;

      if(outputSum > outMax) outputSum= outMax;
      else if(outputSum < outMin) outputSum= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *  does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
   outputSum = *myOutput;
   lastInput = *myInput;
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float PID::GetKp(){ return  dispKp; }
float PID::GetKi(){ return  dispKi;}
float PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}





PID2::PID2(float* Input, float* Output, float* Setpoint,
           float Kp, float Ki, float Kd, float Ks, int POn,
           int ControllerDirection, Position* pPosition)
: PID(Input, Output, Setpoint, Kp, Ki, Kd, POn, ControllerDirection)
, position(pPosition)
, pidStart(0)
, ks(Ks)
, err_counter(0)
{
}

void PID2::Start()
{
    pidStart = millis();
}

// http://en.wikipedia.org/wiki/Simple_linear_regression
float PID2::errorDev(float* errors, int count)
{
    static float Sx = 0.0;
    static float Dv = 0.0;
    
    if (Sx == 0.0)
    {
        float Sxx = 0;
        for(int i = 1; i < PID_ERROR_SIZE; i++)
        {
            Sx += i;
            Sxx += i * i;
        }
        Dv = PID_ERROR_SIZE * Sxx - Sx * Sx;
    }
    
    if (count < PID_ERROR_SIZE) return 0.0;
    
    float Sy = 0.0;
    float Sxy = 0.0;
    for(int i = 0; i < PID_ERROR_SIZE; i++)
    {
        float error = errors[(count+i) % PID_ERROR_SIZE];
        
        Sy += error;
        Sxy += i * error;
    }
    
    return (PID_ERROR_SIZE * Sxy - Sx * Sy) / Dv;
}

bool PID2::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange < SampleTime)
       return false;
   
   *mySetpoint    = position->compute(now - pidStart);
   float futureSetpoint = position->compute(now - pidStart + SampleTime);
   
   /*Compute all the working error variables*/
   float input = *myInput;
   float error = *mySetpoint - input;
   float dInput = (input - lastInput);
   outputSum+= (ki * error);

   I = outputSum;
   
   /*Add Proportional on Measurement, if P_ON_M is specified*/
   if(!pOnE) outputSum-= kp * dInput;

   /* this is a forecast of the next movement */
   float speed = (futureSetpoint - *mySetpoint) / SampleTime;
   S = ks * speed;
   outputSum += S;

   if(outputSum > outMax) outputSum= outMax;
   else if(outputSum < outMin) outputSum= outMin;
   
   /*Add Proportional on Error, if P_ON_E is specified*/
   float output;
   P = kp * error;
   if(pOnE) output = P;
   else output = 0;

   /*Compute Rest of PID Output*/
   //D = kd * dInput;
   errors[err_counter % PID_ERROR_SIZE] = error;
   D = kd * errorDev(errors, err_counter++);

   output += outputSum - D;
   
   if(output > outMax) output = outMax;
   else if(output < outMin) output = outMin;
   *myOutput = output;

   /*Remember some variables for next time*/
   lastInput = input;
   lastTime = now;
   return true;
}