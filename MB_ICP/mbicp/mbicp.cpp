//////////////////////////////////////////////////////////////////////////
// by Nike for mbICP laser matching [2/29/2008 Administrator]
//////////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "mbicp.h"
#include "functions.h"
//-----------------------------------------------------------------------
float * mbicp::refdqx=NULL;
float * mbicp::refdqx2=NULL;
float * mbicp::refdqy=NULL;
float * mbicp::refdqy2=NULL;
float * mbicp::distref=NULL;
float * mbicp::refdqxdqy=NULL;
//----------------------------
float mbicp::error_k1;
int mbicp::numConverged;
//-----------------------------------------------------------------------
void mbicp::Init_MbICP_ScanMatching(int laser_points_num,float max_laser_range,float Bw, float Br,
							        float L, int laserStep,
									float MaxDistInter,
									float filter,
									int ProjectionFilter,
									float AsocError,
									int MaxIter, float error_ratio,
									float error_x, float error_y, float error_t, int IterSmoothConv)
{

	MAXLASERPOINTS = laser_points_num;
	MAXLASERRANGE = max_laser_range;
	//---------------------------------------------------
	params.Bw = Bw;
	params.Br = Br*Br;
	params.error_th=error_ratio;
	params.MaxIter=MaxIter;
	params.LMET=L;
	params.laserStep=laserStep;
	params.MaxDistInter=MaxDistInter;
	params.filter=filter;
	params.ProjectionFilter=ProjectionFilter;
	params.AsocError=AsocError;
	params.errx_out=error_x;
	params.erry_out=error_y;
	params.errt_out=error_t;
	params.IterSmoothConv=IterSmoothConv;
	//---------------------------------------------------
	if (!ptosNew.laserC&&!ptosNew.laserP)
	{
		ptosNew.laserC = new RPOINT[MAXLASERPOINTS];
		ptosNew.laserP = new Tpfp [MAXLASERPOINTS];
	}
	if (!ptosRef.laserC&&!ptosRef.laserP)
	{
		ptosRef.laserC = new RPOINT[MAXLASERPOINTS];
		ptosRef.laserP = new Tpfp [MAXLASERPOINTS];
	}
	if (!ptosNoView.laserC&&!ptosNoView.laserP)
	{
		ptosNoView.laserC = new RPOINT[MAXLASERPOINTS];
		ptosNoView.laserP = new Tpfp [MAXLASERPOINTS];
	}
	//---------------------------------------------------
	if (!refdqx){
		refdqx = new float[MAXLASERPOINTS];
	}
	if (!refdqx2)
	{
		refdqx2   = new float[MAXLASERPOINTS];
	}
	if (!refdqy)
	{
		refdqy    = new float[MAXLASERPOINTS];
	}
	if (!refdqy2)
	{
		refdqy2   = new float[MAXLASERPOINTS];
	}
	if (!distref)
	{
		distref   = new float[MAXLASERPOINTS];
	}
	if (!refdqxdqy)
	{
		refdqxdqy = new float[MAXLASERPOINTS];
	}

}
//-----------------------------------------------------------------------
// Function to do the least-squares but optimized for the metric
//-----------------------------------------------------------------------
int mbicp::computeMatrixLMSOpt(PTAsoc cp_ass, int cnt, ArPose &estimacion) 
{

	int i;
	float LMETRICA2;
	//---------------------------------------------------
	float *X1    = new float[MAXLASERPOINTS];
	float *Y1    = new float[MAXLASERPOINTS];
	float *X2    = new float[MAXLASERPOINTS];
	float *Y2    = new float[MAXLASERPOINTS];
	float *X2Y2  = new float[MAXLASERPOINTS];
	float *X1X2  = new float[MAXLASERPOINTS];
	float *X1Y2  = new float[MAXLASERPOINTS];
	float *Y1X2  = new float[MAXLASERPOINTS];
	float *Y1Y2  = new float[MAXLASERPOINTS];
	float *X1Y1  = new float[MAXLASERPOINTS];
	float *K     = new float[MAXLASERPOINTS];
	float *DS    = new float[MAXLASERPOINTS];
	float *DsD   = new float[MAXLASERPOINTS];
	float *X2DsD = new float[MAXLASERPOINTS]; 
	float *Y2DsD = new float[MAXLASERPOINTS];
	float *Bs    = new float[MAXLASERPOINTS]; 
	float *BsD   = new float[MAXLASERPOINTS];
	//---------------------------------------------------
	float A1, A2, A3, B1, B2, B3, C1, C2, C3, D1, D2, D3;
	//---------------------------------------------------
	vector vecSol(3);
	//---------------------------------------------------
	A1=0;A2=0;A3=0;B1=0;B2=0;B3=0;
	C1=0;C2=0;C3=0;D1=0;D2=0;D3=0;
	//---------------------------------------------------
	LMETRICA2=params.LMET*params.LMET;
	//---------------------------------------------------
	for (i=0; i<cnt; i++){
		X1[i]=cp_ass[i].nx*cp_ass[i].nx;//cix^2
		Y1[i]=cp_ass[i].ny*cp_ass[i].ny;//ciy^2
		X2[i]=cp_ass[i].rx*cp_ass[i].rx;//pix^2
		Y2[i]=cp_ass[i].ry*cp_ass[i].ry;//piy^2
		X2Y2[i]=cp_ass[i].rx*cp_ass[i].ry;//pix*piy
		X1Y1[i]=cp_ass[i].nx*cp_ass[i].ny;//cix*ciy
		X1X2[i]=cp_ass[i].nx*cp_ass[i].rx;//cix*pix
		X1Y2[i]=cp_ass[i].nx*cp_ass[i].ry;//cix*piy
		Y1X2[i]=cp_ass[i].ny*cp_ass[i].rx;//ciy*pix
		Y1Y2[i]=cp_ass[i].ny*cp_ass[i].ry;//ciy*piy

		K[i]   =X2[i]+Y2[i] + LMETRICA2;
		DS[i]  =Y1Y2[i] + X1X2[i];
		DsD[i]=DS[i]/K[i];
		X2DsD[i]=cp_ass[i].rx*DsD[i];
		Y2DsD[i]=cp_ass[i].ry*DsD[i];

		Bs[i]=X1Y2[i]-Y1X2[i];
		BsD[i]=Bs[i]/K[i];

		A1=A1 + (1-Y2[i]/K[i]);
		B1=B1 + X2Y2[i]/K[i];
		C1=C1 + (-cp_ass[i].ny + Y2DsD[i]);
		D1=D1 + (cp_ass[i].nx - cp_ass[i].rx -cp_ass[i].ry*BsD[i]);

		A2=B1;
		B2=B2 + (1-X2[i]/K[i]);
		C2=C2 + (cp_ass[i].nx-X2DsD[i]);
		D2=D2 + (cp_ass[i].ny -cp_ass[i].ry +cp_ass[i].rx*BsD[i]);

		A3=C1;
		B3=C2;
		C3=C3 + (X1[i] + Y1[i] - DS[i]*DS[i]/K[i]);
		D3=D3 + (Bs[i]*(-1+DsD[i]));
		// modified by nike [3/21/2008]
		C += X2[i] + Y2[i]-2*(X1X2[i]+Y1Y2[i])+ X1[i]+Y1[i]
	        -Y2[i]*(X1[i]-2*X1X2[i]+X2[i])/K[i]
		    -X2[i]*(Y1[i]-2*Y1Y2[i]+Y2[i])/K[i]
		    +2*Y1Y2[i]*(X1Y1[i]+X2Y2[i]-Y1X2[i]-X1Y2[i])/K[i];
	}
	//--------------------------------------------------------
    //calculate matrix A;
	matA(1,1)=A1;	matA(1,2)=B1;	matA(1,3)=C1;
	matA(2,1)=A2;	matA(2,2)=B2;	matA(2,3)=C2;
	matA(3,1)=A3;	matA(3,2)=B3;	matA(3,3)=C3;
	//--------------------------------------------------------
	if (!invMatA.Invers(matA))
		return -1;
	//--------------------------------------------------------
	vecB[1]=D1; vecB[2]=D2; vecB[3]=D3;
	vecSol=invMatA*vecB;
	//---------------------------------------------------
	estimacion.setX(-vecSol[1]);
	estimacion.setY(-vecSol[2]);
	vecSol[3]=NormalizarPI(-vecSol[3]);
	estimacion.setThRad(vecSol[3]);
	//SamN=cnt;
	//--------------------------------------------------------
	delete [] X1;
	delete [] Y1;
	delete [] X2;
	delete [] Y2;
	delete [] X2Y2;
	delete [] X1X2;
	delete [] X1Y2;
	delete [] Y1X2;
	delete [] Y1Y2;
	delete [] K;
	delete [] DS;
	delete [] DsD;
	delete [] X2DsD; 
	delete [] Y2DsD;
	delete [] Bs; 
	delete [] BsD;
	delete [] X1Y1;
	//--------------------------------------------------------
	return 1;
}
//-----------------------------------------------------------------------
// Function that compute covariance matrix of association laser data
//-----------------------------------------------------------------------
matrix & mbicp::computeCov(int cnt){
	Emin=C-vecB*invMatA*vecB;
	double s2=Emin/(cnt-3-1);
	invMatA *=s2;
	return invMatA;
}
//-----------------------------------------------------------------------
// Function that initializes the SM parameters
//-----------------------------------------------------------------------
int mbicp::MbICPmatcher(PTpfp laserRef, PTpfp laserNew, const ArPose &sensorMotion,ArPose &solution,matrix &cov)
{

		int resEStep=1;
		int resMStep=1;
		int numIteration=0;

		// Preprocess both scans
		preProcessingLib(laserRef,laserNew,sensorMotion);
		// double sys0=GetTickCount();
		// double sys1;
		while (numIteration<params.MaxIter){

			// Compute the correspondences of the MbICP
			
			resEStep=EStep();
			
			//
			if (resEStep!=1)
				return -1;
			
			// Minimize and compute the solution
			resMStep=MStep(solution);
			

			if (resMStep==1)
			{
				//sys1=GetTickCount()-sys0;
				//sys0=GetTickCount();
				cov=computeCov(cntAssociationsTemp);
				//sys1=GetTickCount()-sys0;
				return 1;
			}
			else if (resMStep==-1)
				return -2;
			else
				numIteration++;
			//Sleep(0);
		}
	    
		return 2;

}
//-----------------------------------------------------------------------
// Function added by Javi for compatibility
//-----------------------------------------------------------------------
void mbicp::preProcessingLib(PTpfp laserK, PTpfp laserK1, const ArPose &initialMotion)
{

	int i,j;
	// ------------------------------------------------//
	motion2=initialMotion;
	// ------------------------------------------------//
	// Compute xy coordinates of the points in laserK1
	ptosNew.numPuntos=0;
	for (i=0; i<MAXLASERPOINTS; i++) {
		if ((laserK1[i].r <MAXLASERRANGE) && (laserK1[i].r>0.02)){
			ptosNew.laserP[ptosNew.numPuntos].r=laserK1[i].r;
			ptosNew.laserP[ptosNew.numPuntos].t=laserK1[i].t;
			ptosNew.laserC[ptosNew.numPuntos].x=(float)(laserK1[i].r * cos(laserK1[i].t));
			ptosNew.laserC[ptosNew.numPuntos].y=(float)(laserK1[i].r * sin(laserK1[i].t));
			ptosNew.numPuntos++;
		}
	}
	// ------------------------------------------------//
	// Choose one point out of params.laserStep points
	j=0;
	for (i=0; i<ptosNew.numPuntos; i+=params.laserStep) {
		ptosNew.laserC[j]=ptosNew.laserC[i];
		j++;
	}
	ptosNew.numPuntos=j;
	// ------------------------------------------------//
	// Compute xy coordinates of the points in laserK
	ptosRef.numPuntos=0;
	for (i=0; i<MAXLASERPOINTS; i++) {
		if ((laserK[i].r <MAXLASERRANGE) && (laserK[i].r>0.02)){
			ptosRef.laserP[ptosRef.numPuntos].r=laserK[i].r;
			ptosRef.laserP[ptosRef.numPuntos].t=laserK[i].t;
			ptosRef.laserC[ptosRef.numPuntos].x=(float)(laserK[i].r * cos(laserK[i].t));
			ptosRef.laserC[ptosRef.numPuntos].y=(float)(laserK[i].r * sin(laserK[i].t));
			ptosRef.numPuntos++;
		}
	}
	// ------------------------------------------------//
	// Choose one point out of params.laserStep points
	j=0;
	for (i=0; i<ptosRef.numPuntos; i+=params.laserStep) {
		ptosRef.laserC[j]=ptosRef.laserC[i];
		j++;
	}
	ptosRef.numPuntos=j;
	// ------------------------------------------------//
	// Preprocess reference points
	for (i=0;i<ptosRef.numPuntos-1;i++) {
		car2pol(&ptosRef.laserC[i],&ptosRef.laserP[i]);
		refdqx[i]=ptosRef.laserC[i].x - ptosRef.laserC[i+1].x;
		refdqy[i]=ptosRef.laserC[i].y - ptosRef.laserC[i+1].y;
		refdqx2[i]=refdqx[i]*refdqx[i];
		refdqy2[i]=refdqy[i]*refdqy[i];
		distref[i]=refdqx2[i] + refdqy2[i];
		refdqxdqy[i]=refdqx[i]*refdqy[i];
	}
	// ------------------------------------------------//
	car2pol(&ptosRef.laserC[ptosRef.numPuntos-1],&ptosRef.laserP[ptosRef.numPuntos-1]);
	// ------------------------------------------------//
	error_k1=BIG_INITIAL_ERROR;
	numConverged=0;
}
//-----------------------------------------------------------------------
// Function that does the minimization step of the MbICP
//-----------------------------------------------------------------------
int mbicp::EStep()
{
	int cnt;
	int i,J;
	// ------------------------------------------------//
	Tscan ptosNewRef;
	ptosNewRef.laserP = new Tpfp[MAXLASERPOINTS];
	ptosNewRef.laserC = new RPOINT[MAXLASERPOINTS];
	// ------------------------------------------------//
	int *indexPtosNewRef = new int[MAXLASERPOINTS];
	// ------------------------------------------------//
	int L,R,Io;
	float dist;
	float cp_ass_ptX,cp_ass_ptY,cp_ass_ptD;
	float tmp_cp_indD;
	// ------------------------------------------------//
	float q1x, q1y, q2x,q2y,p2x,p2y, dqx, dqy, dqpx, dqpy, qx, qy,dx,dy;
	float landaMin;
	float A,B,C,D;
	float LMET2;
	// ------------------------------------------------//
	LMET2=params.LMET*params.LMET;
	// ------------------------------------------------//
	// Transform the points according to the current pose estimation
	ptosNewRef.numPuntos=0;
	for (i=0; i<ptosNew.numPuntos; i++){
		transfor_directa_p ( ptosNew.laserC[i].x, ptosNew.laserC[i].y,
			motion2, &ptosNewRef.laserC[ptosNewRef.numPuntos]);
		car2pol(&ptosNewRef.laserC[ptosNewRef.numPuntos],&ptosNewRef.laserP[ptosNewRef.numPuntos]); 
		ptosNewRef.numPuntos++;
	}
	// ------------------------------------------------//
	/* Projection Filter */
	/* Eliminate the points that cannot be seen */
	/* Furthermore it orders the points with the angle */
	cnt = 1; /* Be careful with this filter (order) when the angles are big >90 */
	ptosNoView.numPuntos=0;
	if (params.ProjectionFilter==1){
		for (i=1;i<ptosNewRef.numPuntos;i++){
			if (ptosNewRef.laserP[i].t>=ptosNewRef.laserP[cnt-1].t){ 
				ptosNewRef.laserP[cnt]=ptosNewRef.laserP[i];
				ptosNewRef.laserC[cnt]=ptosNewRef.laserC[i];
				cnt++;
			}
			else{
				ptosNoView.laserP[ptosNoView.numPuntos]=ptosNewRef.laserP[i];
				ptosNoView.laserC[ptosNoView.numPuntos]=ptosNewRef.laserC[i];
				ptosNoView.numPuntos++;
			}
		}
		ptosNewRef.numPuntos=cnt;
	}

	// ------------------------------------------------//
	/* Build the index for the windows (this is the role of the Bw parameter */
	/* The correspondences are searched between windows in both scans */
	/* Like this you speed up the algorithm */
	L=0; R=0; /* index of the window for ptoRef */
	Io=0; /* index of the window for ptoNewRef */
	//
	if (ptosNewRef.laserP[Io].t<ptosRef.laserP[L].t) {
		if (ptosNewRef.laserP[Io].t + params.Bw < ptosRef.laserP[L].t){
			while (Io<ptosNewRef.numPuntos-1 && ptosNewRef.laserP[Io].t + params.Bw < ptosRef.laserP[L].t) {
				Io++;
			}
		}
		else{
			while (R<ptosRef.numPuntos-1 && ptosNewRef.laserP[Io].t + params.Bw > ptosRef.laserP[R+1].t)
				R++;
		}
	}
	else{
		while (L<ptosRef.numPuntos-1 && ptosNewRef.laserP[Io].t - params.Bw > ptosRef.laserP[L].t)
			L++;
		R=L;
		while (R<ptosRef.numPuntos-1 && ptosNewRef.laserP[Io].t + params.Bw > ptosRef.laserP[R+1].t)
			R++;
	}

	// ------------------------------------------------//
	/* Look for potential correspondences between the scans */
	/* Here is where we use the windows */
	cnt=0;
	for (i=Io;i<ptosNewRef.numPuntos;i++)
	{
		// ------------------------------------------------//
		// Keep the index of the original scan ordering
		cp_associations[cnt].index=indexPtosNewRef[i];
		// ------------------------------------------------//
		// Move the window
		while  (L < ptosRef.numPuntos-1 && ptosNewRef.laserP[i].t - params.Bw > ptosRef.laserP[L].t)
			L = L + 1;
		while (R <ptosRef.numPuntos-1 && ptosNewRef.laserP[i].t + params.Bw > ptosRef.laserP[R+1].t)
			R = R + 1;
		// ------------------------------------------------//
		cp_associations[cnt].L=L;
		cp_associations[cnt].R=R;
		// ------------------------------------------------//
		if (L==R)
		{
			// Just one possible correspondence
			// ------------------------------------------------//
			// precompute stuff to speed up
			qx=ptosRef.laserC[R].x; qy=ptosRef.laserC[R].y;
			p2x=ptosNewRef.laserC[i].x;	p2y=ptosNewRef.laserC[i].y;
			dx=p2x-qx; dy=p2y-qy;
			dist=dx*dx+dy*dy-(dx*qy-dy*qx)*(dx*qy-dy*qx)/(qx*qx+qy*qy+LMET2);
			// ------------------------------------------------//
			if (dist<params.Br)
			{
				cp_associations[cnt].nx=ptosNewRef.laserC[i].x;
				cp_associations[cnt].ny=ptosNewRef.laserC[i].y;
				cp_associations[cnt].rx=ptosRef.laserC[R].x;
				cp_associations[cnt].ry=ptosRef.laserC[R].y;
				cp_associations[cnt].dist=dist;
				cnt++;
			}
		}
		else if (L<R)
		{
			// More possible correspondences
			// ------------------------------------------------//
			cp_ass_ptX=0;
			cp_ass_ptY=0;
			cp_ass_ptD=100000;
			// ------------------------------------------------//
			/* Metric based Closest point rule */
			for (J=L+1;J<=R;J++)
			{
				// ------------------------------------------------//	
				// Precompute stuff to speed up
				q1x=ptosRef.laserC[J-1].x; q1y=ptosRef.laserC[J-1].y;
				q2x=ptosRef.laserC[J].x; q2y=ptosRef.laserC[J].y;
				p2x=ptosNewRef.laserC[i].x; p2y=ptosNewRef.laserC[i].y;
				// ------------------------------------------------//
				dqx=refdqx[J-1]; dqy=refdqy[J-1];
				dqpx=q1x-p2x;  dqpy=q1y-p2y;
				A=1/(p2x*p2x+p2y*p2y+LMET2);
				B=(1-A*p2y*p2y);
				C=(1-A*p2x*p2x);
				D=A*p2x*p2y;
				// ------------------------------------------------//
				landaMin=(D*(dqx*dqpy+dqy*dqpx)+B*dqx*dqpx+C*dqy*dqpy)/(B*refdqx2[J-1]+C*refdqy2[J-1]+2*D*refdqxdqy[J-1]);
				// ------------------------------------------------//
				if (landaMin<0){ // Out of the segment on one side
					qx=q1x; qy=q1y;}
				else if (landaMin>1){ // Out of the segment on the other side
					qx=q2x; qy=q2y;}
				else if (distref[J-1]<params.MaxDistInter) { // Within the segment and interpotation OK
					qx=(1-landaMin)*q1x+landaMin*q2x;
					qy=(1-landaMin)*q1y+landaMin*q2y;
				}
				else{ // Segment too big do not interpolate
					//if (landaMin<0.5){
					if (landaMin<0.5){
						qx=q1x; qy=q1y;}
					else{
						qx=q2x; qy=q2y;}
				}
				// ------------------------------------------------//
				// Precompute stuff to see if we save the association
				dx=p2x-qx;
				dy=p2y-qy;
				tmp_cp_indD=dx*dx+dy*dy-(dx*qy-dy*qx)*(dx*qy-dy*qx)/(qx*qx+qy*qy+LMET2);
				// ------------------------------------------------//
				// Check if the association is the best up to now
				if (tmp_cp_indD < cp_ass_ptD){
					cp_ass_ptX=qx;
					cp_ass_ptY=qy;
					cp_ass_ptD=tmp_cp_indD;
				}
			}

			// Association compatible in distance (Br parameter)
			if (cp_ass_ptD< params.Br){
				cp_associations[cnt].nx=ptosNewRef.laserC[i].x;
				cp_associations[cnt].ny=ptosNewRef.laserC[i].y;
				cp_associations[cnt].rx=cp_ass_ptX;
				cp_associations[cnt].ry=cp_ass_ptY;
				cp_associations[cnt].dist=cp_ass_ptD;

				cnt++;
			}
		}
		else { // This cannot happen but just in case ...
			cp_associations[cnt].nx=ptosNewRef.laserC[i].x;
			cp_associations[cnt].ny=ptosNewRef.laserC[i].y;
			cp_associations[cnt].rx=0;
			cp_associations[cnt].ry=0;
			cp_associations[cnt].dist=params.Br;
			cnt++;
		}
	}  // End for (i=Io;i<ptosNewRef.numPuntos;i++){
	// ------------------------------------------------//
	cntAssociationsT=cnt;
	// ------------------------------------------------//
	delete [] indexPtosNewRef;
	delete [] ptosNewRef.laserP;
	delete [] ptosNewRef.laserC;
	// ------------------------------------------------//
	// Check if the number of associations is ok
	if (cntAssociationsT<ptosNewRef.numPuntos*params.AsocError)
		return 0;
	// ------------------------------------------------//
	return 1;
}

//-----------------------------------------------------------------------
// Function that does the minimization step of the MbICP
//-----------------------------------------------------------------------
int mbicp::MStep(ArPose &solucion)
{

	ArPose estim_cp;
	int i,cnt,res;
	float error_ratio, error;
	float cosw, sinw, dtx, dty, tmp1, tmp2;
	PTAsoc cp_tmp=new TAsoc[MAXLASERPOINTS+1];
	// ------------------------------------------------//
	// Filtering of the spurious data
	// Used the trimmed versions that orders the point by distance between associations
	// ------------------------------------------------//
	if (params.filter<1)
	{
		// ------------------------------------------------//
		// Add Null element in array position 0	(this is because heapsort requirement)
		for (i=0;i<cntAssociationsT;i++)
		{
			cp_tmp[i+1]=cp_associations[i];
		}
		cp_tmp[0].dist=-1;
		// Sort array
		heapsort(cp_tmp, cntAssociationsT);
		// Filter out big distances
		cnt=((int)(cntAssociationsT*100*params.filter))/100;
		// Remove Null element
		for (i=0;i<cnt;i++){
			cp_associationsTemp[i]=cp_tmp[i+1];
		}
	}
	else{ // Just build the Temp array to minimize
		cnt=0;
		for (i=0; i<cntAssociationsT;i++){
			if (cp_associations[i].dist<params.Br){
				cp_associationsTemp[cnt]=cp_associations[i];
				cnt++;
			}
		}
	}
	// ------------------------------------------------//
	cntAssociationsTemp=cnt;
	// ------------------------------------------------//
	/* Do  minimization Minimize Metric-based distance */
	/* This function is optimized to speed up          */
	// ------------------------------------------------//
	res=computeMatrixLMSOpt(cp_associationsTemp,cnt,estim_cp);
	// ------------------------------------------------//
	if (res==-1)
		return -1;
	// ------------------------------------------------//
	double theta=estim_cp.getThRad();
	cosw=(float)cos(theta); 
	sinw=(float)sin(theta);
	// ------------------------------------------------//
	dtx=estim_cp.getX(); 
	dty=estim_cp.getY();
	// ------------------------------------------------//
	/* Compute the error of the associations           */
	// ------------------------------------------------//
	error=0;
	for (i = 0; i<cnt;i++)
	{
		tmp1=cp_associationsTemp[i].nx * cosw - cp_associationsTemp[i].ny * sinw + dtx - cp_associationsTemp[i].rx;
		tmp1*=tmp1;
		tmp2=cp_associationsTemp[i].nx * sinw + cp_associationsTemp[i].ny * cosw + dty - cp_associationsTemp[i].ry;
		tmp2*=tmp2;
		error = error+ tmp1+tmp2;
	}
	// ------------------------------------------------//
	error_ratio = error / error_k1;
	/* Check the exit criteria */
	/* Error ratio */
	if (fabs(1.0-error_ratio)<=params.error_th ||
		(fabs(estim_cp.getX())<params.errx_out && fabs(estim_cp.getY())<params.erry_out
		&& fabs(estim_cp.getThRad())<params.errt_out) ){
			numConverged++;
		}
	else
		numConverged=0;

	// ------------------------------------------------//
	/* Build the solution                              */
	composicion_sis(estim_cp, motion2, solucion);
	motion2=solucion;
	error_k1=error;
	// ------------------------------------------------//
	delete [] cp_tmp;
	/* Number of iterations doing convergence (smooth criterion of convergence) */
	if (numConverged>params.IterSmoothConv)
		return 1;
	else
		return 0;
}

