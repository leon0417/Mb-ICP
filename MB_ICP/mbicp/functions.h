#ifndef FUNCTION_H
#define FUNCTION_H

#include "utility.h"

//-----------------------------------------
void car2pol(PRPOINT in, PTpfp out)
{
	out->r=(float)sqrt(in->x*in->x+in->y*in->y);
	out->t=(float)atan2(in->y,in->x);
}
//-----------------------------------------
void pol2car(PTpfp in, PRPOINT out)
{
	out->x=in->r*(float)cos(in->t);
	out->y=in->r*(float)sin(in->t);
}
//-----------------------------------------
void transfor_directa_p(float x, float y,const ArPose &sistema, PRPOINT sol)
{

	float SinT,CosT;
	double theta=sistema.getThRad();

	SinT=(float)sin(theta);
	CosT=(float)cos(theta);

	sol->x=x*CosT-y*SinT+sistema.getX();
	sol->y=x*SinT+y*CosT+sistema.getY();

}
//-----------------------------------------
void composicion_sis( ArPose &sis1,ArPose &sis2,ArPose &sisOut)
{
	PRPOINT sol=new RPOINT;
	transfor_directa_p(sis2.getX(), sis2.getY(), sis1, sol);
	sisOut.setX(sol->x);
	sisOut.setY(sol->y);
	sisOut.setThRad(NormalizarPI(sis1.getThRad()+sis2.getThRad()));
	delete sol;
}
//-----------------------------------------
void transfor_inversa_p(float x,float y,const ArPose &sistema, PRPOINT sol)
{

	float a13, a23;
	float SinT,CosT;

	double theta=sistema.getThRad();
	double X=sistema.getX();
	double Y=sistema.getY();
	//
	SinT=(float)sin(theta);
	CosT=(float)cos(theta);
	//
	a13=-Y*SinT-X*CosT;
	a23=-Y*CosT+X*SinT;

	sol->x=x*CosT+y*SinT+a13;
	sol->y=-x*SinT+y*CosT+a23;
}  
//-----------------------------------------
void inversion_sis(const ArPose &sisIn, ArPose &sisOut)
{
	float c,s;
	double theta=sisIn.getThRad();
	double x=sisIn.getX();
	double y=sisIn.getY();

	c=(float)cos(theta);
	s=(float)sin(theta);

	sisOut.setX(-c*x-s*y);
	sisOut.setY(s*x-c*y);
	sisOut.setThRad(NormalizarPI(-theta));
}
//////////////////////////////////////////////////////////////////////////
void swapItem(TAsoc *a, TAsoc *b)
{
	TAsoc c;
	c=*a;
	*a=*b;
	*b=c;
}
//-----------------------------------------
void perc_down(TAsoc a[], int i, int n) 
{
	int child; TAsoc tmp;
	for (tmp=a[i]; i*2 <= n; i=child) {
		child = i*2;
		if ((child != n) && (a[child+1].dist > a[child].dist))
			child++;
		if (tmp.dist < a[child].dist)
			a[i] = a[child];
		else
			break;
	}
	a[i] = tmp;
}
//-----------------------------------------
void heapsort(TAsoc a[], int n) 
{
	int i, j;
	j = n;
	for (i=n/2; i>0; i--)  /* BuildHeap */
		perc_down(a,i,j);
	i = 1;
	for (j=n; j>=2; j--) {
		swapItem(&a[i],&a[j]);   /* DeleteMax */
		perc_down(a,i,j-1);
	}
}

#endif