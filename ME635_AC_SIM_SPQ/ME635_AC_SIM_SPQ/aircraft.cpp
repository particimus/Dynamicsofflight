// aircraft.cpp  class function definition file   
#include"stdafx.h"  // #define h 0.01      is included in stdafx.h
#include <math.h>

aircraft::aircraft()   // Constructor.
{
	mass = 13.5; //  Initialize all parameters

	//Assign constant coefficients
	Jx = 0.8244;
	Jy = 1.135;
	Jz = 1.759;
	Jxz = 0.1204;
	rho = 1.2682;
	S = 0.55; b = 2.8956; c = 0.18994; Sprop = 0.2027; Kmotor = 80.;
	CL0 = 0.28; CD0 = 0.03; Cm0 = -0.02338; CLa = 3.45; CDa = 0.30; 
	Cma = -0.38; CLq = 0.0; CDq = 0.0; Cmq = -3.6; CLde = -0.36;
	CDde = 0.0; Cmde = -0.5; Cprop = 1.0; CY0 = 0.0; Cl0 = 0.0; Cn0 = 0.0; CYb = -0.98; Clb = -0.12;
	Cnb = 0.25; CYp = 0.0; Clp = -0.26; Cnp = 0.022;
	CYr = 0.0; Clr = 0.14; Cnr = -0.35; CYda = 0.0; Clda = 0.08; Cnda = 0.06; CYdr = -0.17; Cldr = 0.105; Cndr = -0.032;

	//Compute Gamma functions
	G[0] = Jx*Jz - Jxz*Jxz;
	G[1] = (Jxz*(Jx - Jy + Jz)) / G[0];
	G[2] = (Jz*(Jz - Jy) + Jxz*Jxz) / G[0];
	G[3] = Jz / G[0];
	G[4] = Jxz / G[0];
	G[5] = (Jz - Jx) / G[0];
	G[6] = Jxz / Jy;
	G[7] = ((Jx - Jy)*Jx + Jxz*Jxz) / G[0];
	G[8] = Jx / G[0];

	//Call initial state
	init_state();
}

aircraft::~aircraft() {}        // Destructor

double aircraft::rk(double t)
{
	int i;
	for (i = 0; i < N; i++) y[i] = x[i];
	function(0);
	for (i = 0; i < N; i++) y[i] = x[i] + 0.5*h*K[0][i];
	t += 0.5*h;
	function(1);
	for (i = 0; i < N; i++) y[i] = x[i] + 0.5*h*K[1][i];
	function(2);
	for (i = 0; i < N; i++) y[i] = x[i] + h*K[2][i];
	t += 0.5*h;
	function(3);
	for (i = 0; i < N; i++)
	{
		xdot[i] = (K[0][i] + 2.*K[1][i] + 2.*K[2][i] + K[3][i]) / 6.;
		x[i] += h*xdot[i];
	} // other computation as needed 
	return t;
}

void aircraft::function(int i)
{
	// states are 
	// enum{Pn, Pe, Pd, u, v, w, phi, theta, psi, p, q, r}; 
	// Add equations of motion here 


	//Compute trig functions
	Sphi = sin(y[phi]);
	Cphi = cos(y[phi]);
	Sth = sin(y[theta]);
	Cth = cos(y[theta]);
	Tth = tan(y[theta]);
	Spsi = sin(y[psi]);
	Cpsi = cos(y[psi]);

	//Position and forces
	force();
	K[i][Pn] = y[u] * Cth*Cpsi + y[v] * (Sphi*Sth*Cpsi - Spsi*Cphi) + y[w] * (Cpsi*Cphi*Sth + Sphi*Spsi);
	K[i][Pe] = y[u] * Spsi*Cth + y[v] * (Cphi*Cpsi + Sth*Spsi*Sphi) + y[w] * (Spsi*Sth*Cphi - Sphi*Cpsi);
	K[i][Pd] = -1 * y[u] * Sth + y[v] * Sphi*Cth + y[w] * Cphi*Cth;
	K[i][u] = Fx / mass + (y[r] * y[v] - y[q] * y[w]);
	K[i][v] = Fy / mass + (y[p] * y[w] - y[r] * y[u]);
	K[i][w] = Fz / mass + (y[q] * y[u] - y[p] * y[v]);

	//Angles and moments
	moment();
	K[i][phi] = y[p]  + Sphi*Tth*y[q] + Tth*Cphi*y[r];
	K[i][theta] = Cphi*y[q] - Sphi*y[r];
	K[i][psi] = (Cphi*y[r] + Sphi*y[q]) / Cth;
	K[i][p] = G[1] * y[p] * y[q] - G[2] * y[q] * y[r] + G[3] * Mx + G[4] * Mz;
	K[i][q] = G[5] * y[p] * y[r] - G[6] * ((y[p] * y[p]) - (y[r] * y[r])) + My / Jy;
	K[i][r] = G[7] * y[p] * y[q] - G[1] * y[q] * y[r] + G[4] * Mx + G[8] * Mz;

	//Actuators
	K[i][as1] = Wna*(uc[aa] - y[as1]);
	K[i][as2] = Wna*(y[as1] - y[as2]);
	K[i][es1] = Wne*(uc[ee] - y[es1]);
	K[i][es2] = Wne*(y[es1] - y[es2]);
	K[i][rs1] = Wnr*(uc[rr] - y[rs1]);
	K[i][rs2] = Wnr*(y[rs1] - y[rs2]);
	K[i][ts1] = Wnt*(uc[tt] - y[ts1]);
	K[i][ts2] = Wnt*(y[ts1] - y[ts2]);
}

void aircraft::force(void)
{

	//Compute C functions

	Va = sqrt(((y[u] * y[u]) + (y[v] * y[v]) + (y[w] * y[w])));
	AP = atan(y[w] / y[u]);
	Beta = asin(y[v] / Va);
	CAP = cos(AP);
	SAP = sin(AP);

	//assign actuator values
	if (y[as2] >= amax) d[aa] = amax;
	else if (y[as2] < -amax) d[aa] = -amax;
	else d[aa] = y[as2];

	if (y[es2] >= emax) d[ee] = emax;
	else if (y[as2] < -emax) d[aa] = -emax;
	else d[ee] = y[es2];

	if (y[as2] >= rmax) d[rr] = rmax;
	else if (y[rs2] < -amax) d[rr] = -rmax;
	else d[rr] = y[rs2];

	if (y[as2] >= tmax) d[tt] = tmax;
	else if (y[as2] < -tmax) d[tt] = -tmax;
	else d[tt] = y[ts2];

	Flift = 0.5*rho*Va*Va*S*(CL0 + CLa*AP + (CLq*c*y[q]) / (2 * Va) + CLde*d[ee]);
	Fdrag = 0.5*rho*Va*Va*S*(CD0 + CDa*AP + (CDq*c*y[q]) / (2 * Va) + CDde*d[ee]);
	Fprop = 0.5*rho*Sprop*Cprop*((Kmotor*d[tt])*(Kmotor*d[tt]) - Va*Va);

	//Compute force equations
	Fx = -mass*gr*Sth + SAP*Fdrag - CAP*Flift + Fprop;
	Fy = mass*gr*Cth*Sphi + 0.5*rho*(Va*Va)*S*(CY0 + CYb*Beta + CYp*(b / (2 * Va))*y[p] + CYr*(b / (2 * Va))*y[r] + CYda*d[aa] + CYdr*d[rr]);
	Fz = mass*gr*Cth*Cphi - SAP*Fdrag - CAP*Flift;  // gr is defined in stdafx.h
}

void aircraft::moment(void)
{
	// Add Computation of Moments
	Cx = -CDa*cos(AP) + CLa*sin(AP);
	Cxq = -CDq*cos(AP) + CLq*sin(AP);
	Cxde = -CDde*cos(AP) + CLde*sin(AP);
	Cz = -CDa*sin(AP) - CLa*cos(AP);
	Czq = -CDq*sin(AP) - CLq*cos(AP);
	Czde = -CDde*sin(AP) - CLde*cos(AP);

	Mx = 0.5*rho*(Va*Va)*S*(b*(Cl0 + Clb*Beta + Clp*(b / (2 * Va))*y[p] + Clr*(b / (2 * Va))*y[r] + Clda*d[aa] + Cldr*d[rr]));
	My = 0.5*rho*(Va*Va)*S*(c*(Cm0 + Cma*AP + Cmq*(c / (2 * Va))*y[q] + Cmde*d[ee]));
	Mz = 0.5*rho*(Va*Va)*S*(b*(Cn0 + Cnb*Beta + Cnp*(b / (2 * Va))*y[p] + Cnr*(b / (2 * Va))*y[r] + Cnda*d[aa] + Cndr*d[rr]));
}

void aircraft::init_state(void)
{
	int i; // Computations as needed
	for (i = 0; i < N; i++) x[i] = 0.0;
	for (i = 0; i < 4; i++) d[i] = 0.0;
	// Other initializations
	d[aa]=-2.3864E-12; x[as1]=-2.3864E-12; x[as2]=-2.3864E-12;
	d[ee] = -0.025077; x[es1] = -0.025077; x[es2] = -0.025077;
	d[rr]=0; x[rs1]=0; x[rs2]=0;
	d[tt] = 0.57478; x[ts1] = 0.57478; x[ts2] = 0.57478;
	uc[aa]=-2.3864E-12; uc[ee]=-0.025077; uc[rr]=0; uc[tt]=0.57478;
	AP = -0.0285;
	Wna = 30; Wne=30; Wnr=30; Wnt=2;
	amax=0.8; emax=0.8; rmax=0.8; tmax=1.2;
	Va = 44.7;
	x[phi]=0.05;
	return;
}

double aircraft::control(double t)
{
	double u_La, u_Gt, u_Ro, u_Lo, u_Ve, u_Pi, u_En;

	nav_temp(t);

	u_La = La.pidr(Lac, 0.0, 0.0, 0.0, 0.0);
	uc[aa] = u_Ro;
	uc[ee] = u_Pi - 0.025077;
	uc[tt] = u_En + 0.5747775;
}

double aircraft::get_state(int i)
{
	return x[i];
}