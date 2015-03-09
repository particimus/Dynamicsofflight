#include "stdafx.h"

class controller{
private:
	double Kp, Kd, Ki, Kr;
	double error, up, ui, ud, ur, error_previous;
	double ui_max , uc_max, uc_min, uc, rate_error, trim;

public:
	// Variables passed to the Constructor:  Proportional Gain (Kp), Integral Gain (Ki),      
	// Derivative Gain (Kd), Rate Gain (Kr), Integral Windeup limit (ui_max),       
	// Output Limiter Max (uc_max), Output Limiter Min (uc_min), Output Trim value (trim).
	
	controller(double, double, double, double, double, double, double, double);
	~controller();

	// Variables passed to the pidr function:  Input, Command, Measured,
	// Rate_Command, Rate_Measured.

	double pidr(double, double, double, double, double);

};