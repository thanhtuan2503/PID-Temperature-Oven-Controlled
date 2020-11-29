/*
 * PIDver1.cpp
 *
 * Created: 8/3/2020 5:01:00 PM
 *  Author: thanh
 */ 
#include "PIDver1.h"
PID::PID(double Kp, double Ki, double Kd, double SampleTime, double HighLimit, double LowLimit, bool Direct)
{
	 _kp = Kp;
	 _ki = Ki;
	 _kd = Kd;
	 _SampleTime = SampleTime;
	 _outMax = HighLimit;
	 _outMin = LowLimit;
}

double PID::Calculate(double SettingValue, double PresentValue){
	Error = SettingValue - PresentValue;
	P_part = _kp*(Error - pre_error);
	I_part = 0.5*_ki*_SampleTime*(Error + pre_error);
	D_part = _kd/(_SampleTime*_SampleTime)*( Error - 2*pre_error + pre_pre_error);
	PidOut = PidLastOut + P_part + I_part + D_part ;
	pre_pre_error = pre_error;
	pre_error = Error;
	PidLastOut = PidOut;
	if (PidOut > _outMax)
	{
	  PidOut = _outMax;
	}
	else if (PidOut < _outMin)
	{
	  PidOut = _outMin;
	}
	return PidOut;
}
