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
	double Lap, Lai, La_ui_max, La_uc_max, La_uc_min, La_trim; 
	double Gtp, Gti, Gtd, Gt_ui_max, Gt_uc_max, Gt_uc_min, Gt_trim;
	double Rop, Roi, Ror, Ro_ui_max, Ro_uc_max, Ro_uc_min, Ro_trim;
	double Lop, Lod, Lo_ui_max, Lo_uc_max, Lo_uc_min, Lo_trim;
	double Vep, Vei, Ved, Ve_ui_max, Ve_uc_max, Ve_uc_min, Ve_trim;
	double Pip, Pii, Pir, Pi_ui_max, Pi_uc_max, Pi_uc_min, Pi_trim;
	double Enp, Eni, End, En_ui_max, En_uc_max, En_uc_min, En_trim;
	double Sphi, Cphi, Sth, Cth, Tth, Spsi, Cpsi;
	double Fx, Fy, Fz, Mx, My, Mz, Flift, Fdrag, Fprop;
	double mass, rho;
	double Jx, Jy, Jz, Jxz;
	double S, b, c, Sprop, Kmotor, CL0, CD0, Cm0, CLa, CDa, Cma, CLq, CDq, Cmq, CLde;
	double CDde, Cmde, Cprop, CY0, Cl0, Cn0, CYb, Clb, Cnb, CYp, Clp, Cnp;
	double CYr, Clr, Cnr, CYda, Clda, Cnda, CYdr, Cldr, Cndr;
	double Cx, Cxq, Cxde, Cz, Czq, Czde;
	double Va, V_a, AP, Beta, CAP, SAP;
	double amax, amin, emax, emin, rmax, rmin, tmax, tmin;
	double Wna, Wne, Wnr, Wnt;
	double Vc, Pnc, Pec, Altc, Rollc, Pitchc, roll_ratec, pitch_ratec, Lac, Loc, Gtc, Energyc;
	double Alt, GT;
	double Energy;
	//  other parameters as needed
public:
	aircraft();   // constructor
	~aircraft();   // destructor 
	double rk(double);
	void  function(int);
	void  force(void);
	void  moment(void);
	void  init_state(void);
	void nav_temp(double t);
	void control(double t);
	double  get_state(int);
	// other functions as needed
};