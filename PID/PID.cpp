/**********************************************************************************************
* Arduino PID Library - Version 1.0.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
* Modified by Lau Ka Chun, Mechanical and Automation Engineering, The Chinese University of Hong Kong
* Last modification: 18 May 2013
* This Library is licensed under a GPLv3 License
**********************************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <PID.h>

PID::PID(double Kp, double Ki, double Kd, double MinOut, double MaxOut, long Period)
{
	inAuto = false;
	iTerm = 0;

	PID::SetOutputLimits(MinOut, MaxOut);

	sampleTime = Period;

	PID::SetTunings(Kp, Ki, Kd);

	lastTime = millis() - sampleTime;				
}

/* Compute() **********************************************************************
*     This, as they say, is where the magic happens.  this function should be called
*   every time "void loop()" executes.  the function will decide for itself whether a new
*   pid Output needs to be computed.  returns true when the output is computed,
*   false when nothing has been done.
**********************************************************************************/ 
bool PID::Compute()
{
	if(!inAuto)
		return false;
	unsigned long now = millis();
	unsigned long timeChange = (now - lastTime);
	if(timeChange >= sampleTime)
	{
		/*Compute all the working error variables*/
		double input = myInput;
		double error = mySetpoint - input;

		iTerm += (ki * error);
		if(iTerm > outMax)
			iTerm = outMax;
		else if(iTerm < outMin)
			iTerm = outMin;

		double dInput = (input - lastInput);

		pTerm = kp * error;
		dTerm = kd * (-dInput);

		/*Compute PID Output*/
		double output = pTerm + iTerm + dTerm;

		if(output > outMax)
			output = outMax;
		else if(output < outMin)
			output = outMin;

		myOutput = output;

		/*Remember some variables for next time*/
		lastInput = input;
		lastTime = now;
		return true;
	}
	else
		return false;
}


/* SetTunings(...)*************************************************************
* This function allows the controller's dynamic performance to be adjusted. 
* it's called automatically from the constructor, but tunings can also
* be adjusted on the fly during normal operation
******************************************************************************/ 
void PID::SetTunings(double Kp, double Ki, double Kd)
{
	if (Kp < 0 || Ki < 0 || Kd < 0)
		return;

	double SampleTimeInSec = ((double)sampleTime) / 1000;  
	kp = Kp;
	ki = Ki * SampleTimeInSec;
	kd = Kd / SampleTimeInSec;
}

/* SetSampleTime(...) *********************************************************
* sets the period, in Milliseconds, at which the calculation is performed	
******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
	if (NewSampleTime > 0)
	{
		double ratio  = (double)NewSampleTime / (double)sampleTime;
		ki *= ratio;
		kd /= ratio;
		sampleTime = (unsigned long)NewSampleTime;
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
void PID::SetOutputLimits(double Min, double Max)
{
	if(Min >= Max)
		return;

	outMin = Min;
	outMax = Max;

	if(inAuto)
	{
		if(myOutput > outMax)
			myOutput = outMax;
		else if(myOutput < outMin)
			myOutput = outMin;

		if(iTerm > outMax)
			iTerm= outMax;
		else if(iTerm < outMin)
			iTerm= outMin;
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
	if(newAuto == !inAuto)
	{
		/*we just went from manual to auto*/
		PID::Initialize();
	}
	inAuto = newAuto;
}

void PID::SetSetpoint(double Setpoint)
{
	mySetpoint = Setpoint;
}

void PID::SetInput(double Input)
{
	myInput = Input;
}

double PID::GetOutput()
{
	return myOutput;
}

/* Initialize()****************************************************************
*	does all the things that need to happen to ensure a bumpless transfer
*  from manual to automatic mode.
******************************************************************************/ 
void PID::Initialize()
{
	iTerm = myOutput;
	lastInput = myInput;
	if(iTerm > outMax)
		iTerm = outMax;
	else if(iTerm < outMin)
		iTerm = outMin;
}

/* Status Funcions*************************************************************
* Just because you set the Kp=-1 doesn't mean it actually happened.  these
* functions query the internal state of the PID.  they're here for display 
* purposes.  this are the functions the PID Front-end uses for example
******************************************************************************/
double PID::GetKp(){ return  kp; }
double PID::GetKi(){ return  ki; }
double PID::GetKd(){ return  kd; }
double PID::GetPTerm(){ return pTerm; }
double PID::GetITerm(){ return iTerm; }
double PID::GetDTerm(){ return dTerm; }
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL; }