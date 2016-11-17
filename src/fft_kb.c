#include <math.h>
#include <complex.h>
typedef double complex cpx;

static const double dos_pi = 4 * acos(0);

void fft (cpx x[], cpx y[], int dx, int N, int dir)
{
	if (N>1)
	{
		fft(x,y,2*dx,N/2,dir);
		fft(x+dx,y+N/2,2*dx,N/2,dir);
		int i = 0;
		for(; i<N/2; i++)
		{
			cpx par=y[i];
			cpx impar=y[i+N/2];
			cpx twiddle=exp((cpx)(0+I*dir*dos_pi*i/N));
			y[i]=par + twiddle*impar;
			y[i+N]=par - twiddle*impar;
		}

	}
	else
	{
		y[0]=x[0];
	}
}

