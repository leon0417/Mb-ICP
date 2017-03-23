#ifndef PARAM_H
#define PARAM_H



#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	//---------------------------------------------------
	/* --- Thresold parameters */
	/* Bw: maximum angle diference between points of different scans */
	/* Points with greater Bw cannot be correspondent (eliminate spurius asoc.) */
	/* This is a speed up parameter */
	double Bw;
	//---------------------------------------------------
	/* Br: maximum distance difference between points of different scans */
	/* Points with greater Br cannot be correspondent (eliminate spurius asoc.) */
	double Br;

	//---------------------------------------------------
	/* --- Inner parameters */
	/* L: value of the metric */
	/* When L tends to infinity you are using the standart ICP */
	/* When L tends to 0 you use the metric (more importance to rotation */
	double LMET;
	//---------------------------------------------------
	/* laserStep: selects points of each scan with an step laserStep  */
	/* When laserStep=1 uses all the points of the scans */
	/* When laserStep=2 uses one each two ... */
	/* This is an speed up parameter */
	int laserStep;
	//---------------------------------------------------
	/* ProjectionFilter: */
	/* Eliminate the points that cannot be seen given the two scans (see Lu&Millios 97) */
	/* It works well for angles < 45 \circ*/
	/* 1 : activates the filter */
	/* 0 : desactivates the filter */
	int ProjectionFilter;
	//---------------------------------------------------
	/* MaxDistInter: maximum distance to interpolate between points in the ref scan */
	/* Consecutive points with less Euclidean distance than MaxDistInter are considered to be a segment */
	double MaxDistInter;
	//---------------------------------------------------
	/* filtrado: in [0,1] sets the % of asociations NOT considered spurious */
	double filter;
	//---------------------------------------------------
	/* AsocError: in [0,1] */
	/* One way to check if the algorithm diverges if to supervise if the number of associatios goes below a thresold */
	/* When the number of associations is below AsocError, the main function will return error in associations step */
	double AsocError;
	
	//---------------------------------------------------
	/* --- Exit parameters */
	/* MaxIter: sets the maximum number of iterations for the algorithm to exit */
	/* More iterations more chance you give the algorithm to be more accurate   */
	int MaxIter;
	//---------------------------------------------------
	/* error_th: in [0,1] sets the maximum error ratio between iterations to exit */
	/* In each iteration, the error is the residual of the minimization */
	/* When error_th tends to 1 more precise is the solution of the scan matching */
	double error_th;
	//---------------------------------------------------
	/* errx_out,erry_out, errt_out: minimum error of the asociations to exit */
	/* In each iteration, the error is the residual of the minimization in each component */
	/* The condition is (lower than errx_out && lower than erry_out && lower than errt_out */
	/* When error_XXX tend to 0 more precise is the solution of the scan matching */
	double errx_out,erry_out, errt_out;
	//---------------------------------------------------
	/* IterSmoothConv: number of consecutive iterations that satisfity the error criteria */
	/* (error_th) OR (errorx_out && errory_out && errt_out) */
	/* With this parameter >1 avoids random solutions */
	int IterSmoothConv;

} TSMparams ;
//----------------------------------------------

#ifdef __cplusplus
}
#endif

#endif