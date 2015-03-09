// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently

#pragma once
#define gr 9.80665  // gravity (m/s^2)
#define roe 1.2682  // air density (kg/m^3)
#define pi 3.1416
#define d2r (pi / 180.)
#define N 20  // 12 force and moment states and 8 actuator states
#define h 0.01 // numerical integration step-size (s)

#include <stdio.h>
#include <tchar.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include "aircraft.h"
//#include "controller.h"

using namespace std;

// TODO: reference additional headers your program requires here
