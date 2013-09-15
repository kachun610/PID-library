/**********************************************************************************************
* Arduino PID Library - Version 1.0.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
* Modified by Lau Ka Chun, Mechanical and Automation Engineering, The Chinese University of Hong Kong
* Last modification: 18 May 2013
* This Library is licensed under a GPLv3 License
**********************************************************************************************/

#ifndef PID_h
#define PID_h
#define LIBRARY_VERSION	1.0.1

class PID
{
public:
	//Constants used in some of the functions below
	#define AUTOMATIC	1
	#define MANUAL		0

	// Constructor. Initial tuning parameters, output limits and sampling time
	PID(double Kp, double Ki, double Kd, double MinOut, double MaxOut, long Period);   

	// Performs the PID calculation.  it should be called every time loop() cycles. ON/OFF and
	// calculation frequency can be set using SetMode SetSampleTime respectively
	bool Compute();
	
	// While most users will set the tunings once in the constructor, this function gives the user the option
	// of changing tunings during runtime for Adaptive control
	void SetTunings(double Kp, double Ki, double Kd);

	// Sets the frequency, in Milliseconds, with which the PID calculation is performed.
	void SetSampleTime(int NewSampleTime);
	
	// Clamps the output to a specific range.
	void SetOutputLimits(double Min, double Max); 

	// Sets PID to either Manual (0) or Auto (non-0)
	void SetMode(int Mode);

	// Setup setpoint	
	void SetSetpoint(double Setpoint);
	
	// Setup input
	void SetInput(double);
	
	// Get output	
	double GetOutput();
	
	//Display functions ****************************************************************
	double GetKp();
	double GetKi();
	double GetKd();
	double GetPTerm();
	double GetITerm();
	double GetDTerm();
	int GetMode();

private:
	void Initialize();

	double kp;                  // * (P)roportional Tuning Parameter
	double ki;                  // * (I)ntegral Tuning Parameter
	double kd;                  // * (D)erivative Tuning Parameter

	double myInput;				// Input, Output, and Setpoint variables
	double myOutput;
	double mySetpoint;

	unsigned long lastTime;
	double lastInput;

	unsigned long sampleTime;
	double outMin, outMax;
	bool inAuto;
	double pTerm;
	double iTerm;
	double dTerm;
};
#endif