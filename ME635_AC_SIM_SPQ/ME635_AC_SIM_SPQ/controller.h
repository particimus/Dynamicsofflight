#include "stdafx.h"

class controller{
private:
double Kp,Kd, Ki, Kr;
double error, up, ui, ud, ur, error_previous;
double ui_max , uc_max, uc_min, uc, rate_error, trim;

public:
controller(double, double, double, double, double, double, double, double);
~controller();
double pidr(double, double, double, double, double);

};