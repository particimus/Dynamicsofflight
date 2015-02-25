// aircraft.h  header file for class aircraft 

#include "stdafx.h"
//int N = 12;   //is in stdafx.h

class aircraft {
private:
	/*
	Pn = Position North inertial
	Pe = Position East inertial
	Pd = Position Down inertial
	u = Velocity Forward
	v = Velocity Side
	w = Velocity Down
	phi = Roll angle
	theta = Pitch angle
	psi = Yaw Angle
	p = Roll Rate
	q = Pitch Rate
	r = Yaw Rate
	x[N] =
	xdot[N] =
	y[N] =
	K[4][N] = Runge-Kutta
	Sphi = Sin(phi)
	Cphi = Cos(phi)
	Sth = Sin(theta)
	Cth = Cos(theta)
	Tth = Tan(theta)
	Spsi = Sin(psi)
	Cpsi = Cos(psi)
	Fx = Force x direction
	Fy = Force y direction
	Fz = Force z direction
	Mx = Moment x direction
	My = Moment y direction
	AP = Alpha
	Beta = Beta angle
	The Gs are functions of moments and products of inertia
	*/
	enum{ Pn, Pe, Pd, u, v, w, phi, theta, psi, p, q, r, as1, as2, es1, es2, rs1, rs2, ts1, ts2 };
	enum{ aa, ee, rr, tt };
	/*enum{ w_ns, w_es, w_ds, u_wg, v_wg, w_wg };*/
	double x[N], xdot[N], y[N], K[4][N], G[9], d[4], wn[6], uc[4];
	double Sphi, Cphi, Sth, Cth, Tth, Spsi, Cpsi;
	double Fx, Fy, Fz, Mx, My, Mz, Flift, Fdrag, Fprop;
	double mass, rho;
	double Jx, Jy, Jz, Jxz;
	double S, b, c, Sprop, Kmotor, CL0, CD0, Cm0, CLa, CDa, Cma, CLq, CDq, Cmq, CLde;
	double CDde, Cmde, Cprop, CY0, Cl0, Cn0, CYb, Clb, Cnb, CYp, Clp, Cnp;
	double CYr, Clr, Cnr, CYda, Clda, Cnda, CYdr, Cldr, Cndr;
	double Cx, Cxq, Cxde, Cz, Czq, Czde;
	double Va, AP, Beta, CAP, SAP;
	double amax, amin, emax, emin, rmax, rmin, tmax, tmin;
	double Wna, Wne, Wnr, Wnt;
	//  other parameters as needed
public:
	aircraft();   // constructor
	~aircraft();   // destructor 
	double rk(double);
	void  function(int);
	void control(double t);
	void  force(void);
	void  moment(void);
	void  init_state(void);
	double  get_state(int);
	// other functions as needed
};