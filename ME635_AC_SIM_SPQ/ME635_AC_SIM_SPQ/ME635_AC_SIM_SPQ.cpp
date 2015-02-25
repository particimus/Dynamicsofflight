// ME635_AC_SIM_SPQ.cpp : Defines the entry point for the console application.

#include "stdafx.h"
ofstream fout("acdata.txt");

int main()
{
	aircraft  ac;
	double  t, tend = 100.0;
	int i;
	for (t = 0.; t < tend;)
	{
		t = ac.rk(t);
		cout << t << "\t" << h << endl;
		fout << t << "\t";
		for (i = 0; i < N; i++)
		{
			fout << ac.get_state(i) << "\t";  //  output to screen and file
		}
		fout << endl;
	}
	return 0;
}