//////////////////////////////////////////////////////////////////////////
#ifndef MBICP_H
#define MBICP_H
// by Nike for metric-based ICP matching of laser data [2/29/2008 Administrator]
//////////////////////////////////////////////////////////////////////////
//#include "Aria.h"
#include "Param.h"
#include "Matrix.h"
#include "utility.h"
//////////////////////////////////////////////////////////////////////////
class mbicp
{
public:
	
	mbicp():cp_associations(NULL),cp_associationsTemp(NULL){
		//
		ptosRef.laserC=NULL;
		ptosRef.laserP=NULL;
		ptosRef.numPuntos=0;
		//
		ptosNew.laserC=NULL;
		ptosNew.laserP=NULL;
		ptosNew.numPuntos=0;
		//
		ptosNoView.laserC=NULL;
		ptosNoView.laserP=NULL;
		ptosNoView.numPuntos=0;
		//
		refdqx=NULL;
		refdqx2=NULL;
		refdqy=NULL;
		refdqy2=NULL;
		distref=NULL;
		refdqxdqy=NULL;
		//
		cp_associations=NULL;
		cntAssociationsT=0;
		cp_associationsTemp=NULL;
		cntAssociationsTemp=0;
	}
	mbicp(TSMparams &params,int laser_points_num,float max_laser_range=8000.0)
	:matA(3,3),vecB(3),C(0)
	{
		MAXLASERPOINTS = laser_points_num;
		MAXLASERRANGE = max_laser_range;
		//---------------------------------------------------
		this->params=params;
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
		if (!refdqx)
			refdqx = new float[MAXLASERPOINTS];
		if (!refdqx2)
			refdqx2   = new float[MAXLASERPOINTS];
		if (!refdqy)
			refdqy    = new float[MAXLASERPOINTS];
		if (!refdqy2)
			refdqy2   = new float[MAXLASERPOINTS];
		if (!distref)
			distref   = new float[MAXLASERPOINTS];
		if (!refdqxdqy)
			refdqxdqy = new float[MAXLASERPOINTS];
		//
		if (!cp_associations)
			cp_associations = new TAsoc[MAXLASERPOINTS];
		if (!cp_associationsTemp)
			cp_associationsTemp = new TAsoc[MAXLASERPOINTS];
		//-----------------------------------------
	}
	//-----------------------------------------
	~mbicp()
	{
		if (ptosRef.laserC)
			delete [] ptosRef.laserC;
		if (ptosRef.laserP)
			delete [] ptosRef.laserP;
		ptosRef.numPuntos=0;
		//---------------------------------------------------
		if (ptosNew.laserC)
			delete [] ptosNew.laserC;
		if (ptosNew.laserP)
			delete [] ptosNew.laserP;
		ptosNew.numPuntos=0;
		//---------------------------------------------------
		if (ptosNoView.laserC)
			delete [] ptosNoView.laserC;
		if (ptosNoView.laserP)
			delete [] ptosNoView.laserP;
		ptosNoView.numPuntos=0;
		//---------------------------------------------------
		if (refdqx)
			delete [] refdqx;
		if (refdqx2)
			delete [] refdqx2;
		if (refdqy)
			delete [] refdqy;
		if (refdqy2)
			delete [] refdqy2;
		if (distref)
			delete [] distref;
		if (refdqxdqy)
			delete [] refdqxdqy;
		//---------------------------------------------------
		if (cp_associations)
			delete [] cp_associations;
		cntAssociationsT=0;
		if (cp_associationsTemp)
			delete [] cp_associationsTemp;
		cntAssociationsTemp=0;

	}
    //-----------------------------------------
	void Init_MbICP_ScanMatching(
		int laser_points_num,
		float max_laser_range,
		float Bw,
		float Br,
		float L,
		int   laserStep,
		float MaxDistInter,
		float filter,
		int   ProjectionFilter,
		float AsocError,
		int   MaxIter,
		float errorRatio,
		float errx_out,
		float erry_out,
		float errt_out,
		int IterSmoothConv);
	//-----------------------------------------
	int MbICPmatcher(PTpfp laserRef, PTpfp laserNew,const ArPose &sensorMotion,ArPose &solution,matrix &cov = matrix(3,3));
	//-----------------------------------------
	void preProcessingLib(PTpfp laserRef, PTpfp laserNew,const ArPose &initialMotion);
	//-----------------------------------------
	// Function that does the association step of the MbICP
	int EStep();
	//-----------------------------------------
	// Function that does the minimization step of the MbICP
	int MStep(ArPose &solution);
	//-----------------------------------------
	int computeMatrixLMSOpt(PTAsoc cp_ass, int cnt, ArPose &estimacion);
	//-----------------------------------------
	matrix & computeCov(int cnt);
	//-----------------------------------------
	// Initial error to compute error ratio
	#define BIG_INITIAL_ERROR 1000000.0F
private:
	
	int MAXLASERPOINTS;
	float MAXLASERRANGE;
	//---------------------------------------------------
	// Static structure to initialize the SM parameters
	TSMparams params;
	//-----------------------------------------
	// Original laser points to be aligned
	Tscan ptosRef;
	Tscan ptosNew;
	//-----------------------------------------
	// At each step::
	// Those points removed by the projection filter (see Lu & Millios -- IDC)
	Tscan ptosNoView; // Only with ProjectionFilter=1;
	//-----------------------------------------
	// Structure of the associations before filtering
	PTAsoc cp_associations;
	int cntAssociationsT;
	//-----------------------------------------
	// Filtered Associations
	PTAsoc cp_associationsTemp;
	int cntAssociationsTemp;
	//-----------------------------------------
	// Some precomputations for each scan to speed up
	static float *refdqx;
	static float *refdqx2;
	static float *refdqy;
	static float *refdqy2;
	static float *distref;
	static float *refdqxdqy;
	//-----------------------------------------
	// value of errors
	static float error_k1;
	static int numConverged;
	//-----------------------------------------
	// Current motion estimation
	ArPose motion2;
	//-----------------------------------------
	matrix matA;
	matrix invMatA;
	//matrix Ecov;
	vector vecB;
	int    SamN;
	double Emin;
	double C;
};
#endif