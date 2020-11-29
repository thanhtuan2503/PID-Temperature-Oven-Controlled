/*
 * PIDver1.h
 *
 * Created: 8/3/2020 5:00:31 PM
 *  Author: thanh
 */ 


#ifndef PIDVER1_H_
#define PIDVER1_H_
class PID{
	public:
		PID(double Kp, double Ki, double Kd, double SampleTime, double HLimit, double LowLimit, bool Direct);
		double Calculate(double SettingValue, double PresentValue);
	private: 
	   double _kp;
	   double _ki;
	   double _kd;
	   double _SampleTime;
	   double _outMax, _outMin;
	   double PidLastOut;
	   double Error, pre_error, pre_pre_error, P_part, I_part, D_part, PidOut;
   };
#endif /* PIDVER1_H_ */
