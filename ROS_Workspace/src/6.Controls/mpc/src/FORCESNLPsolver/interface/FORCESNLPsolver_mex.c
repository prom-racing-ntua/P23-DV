/*
FORCESNLPsolver : A fast customized optimization solver.

Copyright (C) 2013-2023 EMBOTECH AG [info@embotech.com]. All rights reserved.


This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/

#include "mex.h"
#include "math.h"
#include "../include/FORCESNLPsolver.h"
#include "../include/FORCESNLPsolver_memory.h"
#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif



/* copy functions */

void copyCArrayToM_double(double *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_double(double *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double) (*src++) ;
    }
}

void copyMValueToC_double(double * src, double * dest)
{
	*dest = (double) *src;
}

/* copy functions */

void copyCArrayToM_FORCESNLPsolver_int(FORCESNLPsolver_int *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_FORCESNLPsolver_int(double *src, FORCESNLPsolver_int *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (FORCESNLPsolver_int) (*src++) ;
    }
}

void copyMValueToC_FORCESNLPsolver_int(double * src, FORCESNLPsolver_int * dest)
{
	*dest = (FORCESNLPsolver_int) *src;
}

/* copy functions */

void copyCArrayToM_solver_int32_unsigned(solver_int32_unsigned *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_solver_int32_unsigned(double *src, solver_int32_unsigned *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (solver_int32_unsigned) (*src++) ;
    }
}

void copyMValueToC_solver_int32_unsigned(double * src, solver_int32_unsigned * dest)
{
	*dest = (solver_int32_unsigned) *src;
}

/* copy functions */

void copyCArrayToM_solver_int32_default(solver_int32_default *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_solver_int32_default(double *src, solver_int32_default *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (solver_int32_default) (*src++) ;
    }
}

void copyMValueToC_solver_int32_default(double * src, solver_int32_default * dest)
{
	*dest = (solver_int32_default) *src;
}

/* copy functions */

void copyCArrayToM_FORCESNLPsolver_float(FORCESNLPsolver_float *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_FORCESNLPsolver_float(double *src, FORCESNLPsolver_float *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (FORCESNLPsolver_float) (*src++) ;
    }
}

void copyMValueToC_FORCESNLPsolver_float(double * src, FORCESNLPsolver_float * dest)
{
	*dest = (FORCESNLPsolver_float) *src;
}



extern solver_int32_default (FORCESNLPsolver_float *x, FORCESNLPsolver_float *y, FORCESNLPsolver_float *l, FORCESNLPsolver_float *p, FORCESNLPsolver_float *f, FORCESNLPsolver_float *nabla_f, FORCESNLPsolver_float *c, FORCESNLPsolver_float *nabla_c, FORCESNLPsolver_float *h, FORCESNLPsolver_float *nabla_h, FORCESNLPsolver_float *hess, solver_int32_default stage, solver_int32_default iteration, solver_int32_default threadID);
FORCESNLPsolver_extfunc pt2function_FORCESNLPsolver = &;


/* Some memory for mex-function */
static FORCESNLPsolver_params params;
static FORCESNLPsolver_output output;
static FORCESNLPsolver_info info;
static FORCESNLPsolver_mem * mem;

/* THE mex-function */
void mexFunction( solver_int32_default nlhs, mxArray *plhs[], solver_int32_default nrhs, const mxArray *prhs[] )  
{
	/* file pointer for printing */
	FILE *fp = NULL;

	/* define variables */	
	mxArray *par;
	mxArray *outvar;
	const mxArray *PARAMS = prhs[0]; 
	double *pvalue;
	solver_int32_default i;
	solver_int32_default exitflag;
	const solver_int8_default *fname;
	const solver_int8_default *outputnames[40] = {"x01", "x02", "x03", "x04", "x05", "x06", "x07", "x08", "x09", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "x18", "x19", "x20", "x21", "x22", "x23", "x24", "x25", "x26", "x27", "x28", "x29", "x30", "x31", "x32", "x33", "x34", "x35", "x36", "x37", "x38", "x39", "x40"};
	const solver_int8_default *infofields[10] = { "it", "res_eq", "rsnorm", "pobj", "solvetime", "fevalstime", "QPtime", "QPit", "QPexitflag", "solver_id"};
	
	/* Check for proper number of arguments */
    if (nrhs != 1)
	{
		mexErrMsgTxt("This function requires exactly 1 input: PARAMS struct.\nType 'help FORCESNLPsolver_mex' for details.");
	}    
	if (nlhs > 3) 
	{
        mexErrMsgTxt("This function returns at most 3 outputs.\nType 'help FORCESNLPsolver_mex' for details.");
    }

	/* Check whether params is actually a structure */
	if( !mxIsStruct(PARAMS) ) 
	{
		mexErrMsgTxt("PARAMS must be a structure.");
	}
	 

    /* initialize memory */
    if (mem == NULL)
    {
        mem = FORCESNLPsolver_internal_mem(0);
    }

	/* copy parameters into the right location */
	par = mxGetField(PARAMS, 0, "xinit");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.xinit not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.xinit must be a double.");
    }
    if( mxGetM(par) != 9 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.xinit must be of size [9 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.xinit,9);

	}
	par = mxGetField(PARAMS, 0, "x0");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.x0 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.x0 must be a double.");
    }
    if( mxGetM(par) != 480 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.x0 must be of size [480 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.x0,480);

	}
	par = mxGetField(PARAMS, 0, "all_parameters");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.all_parameters not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.all_parameters must be a double.");
    }
    if( mxGetM(par) != 160 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.all_parameters must be of size [160 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.all_parameters,160);

	}
	par = mxGetField(PARAMS, 0, "reinitialize");
	if ( (par != NULL) && (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMValueToC_FORCESNLPsolver_int(mxGetPr(par), &params.reinitialize);

	}


	par = mxGetField(PARAMS, 0, "num_of_threads");
	if ( (par != NULL) && (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMValueToC_solver_int32_unsigned(mxGetPr(par), &params.num_of_threads);

	}




	#if SET_PRINTLEVEL_FORCESNLPsolver > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) 
		{
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* call solver */

	exitflag = FORCESNLPsolver_solve(&params, &output, &info, mem, fp, pt2function_FORCESNLPsolver);
	
	#if SET_PRINTLEVEL_FORCESNLPsolver > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			mexPrintf("%c",i);
		}
		fclose(fp);
	#endif

	/* copy output to matlab arrays */
	plhs[0] = mxCreateStructMatrix(1, 1, 40, outputnames);
		/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x01[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x01", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x02[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x02", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x03[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x03", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x04[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x04", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x05[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x05", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x06[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x06", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x07[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x07", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x08[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x08", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x09[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x09", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x10[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x10", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x11[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x11", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x12[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x12", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x13[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x13", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x14[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x14", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x15[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x15", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x16[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x16", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x17[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x17", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x18[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x18", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x19[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x19", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x20[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x20", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x21[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x21", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x22[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x22", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x23[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x23", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x24[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x24", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x25[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x25", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x26[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x26", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x27[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x27", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x28[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x28", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x29[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x29", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x30[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x30", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x31[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x31", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x32[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x32", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x33[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x33", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x34[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x34", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x35[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x35", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x36[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x36", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x37[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x37", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x38[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x38", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x39[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x39", outvar);


	/* column vector of length 12 */
	outvar = mxCreateDoubleMatrix(12, 1, mxREAL);
	copyCArrayToM_double((&(output.x40[0])), mxGetPr(outvar), 12);
	mxSetField(plhs[0], 0, "x40", outvar);


	/* copy exitflag */
	if( nlhs > 1 )
	{
	plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
	*mxGetPr(plhs[1]) = (double)exitflag;
	}

	/* copy info struct */
	if( nlhs > 2 )
	{
	plhs[2] = mxCreateStructMatrix(1, 1, 10, infofields);
				/* scalar: iteration number */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_solver_int32_default((&(info.it)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "it", outvar);


		/* scalar: inf-norm of equality constraint residuals */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_FORCESNLPsolver_float((&(info.res_eq)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "res_eq", outvar);


		/* scalar: norm of stationarity condition */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_FORCESNLPsolver_float((&(info.rsnorm)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "rsnorm", outvar);


		/* scalar: primal objective */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_FORCESNLPsolver_float((&(info.pobj)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "pobj", outvar);


		/* scalar: total solve time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_FORCESNLPsolver_float((&(info.solvetime)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "solvetime", outvar);


		/* scalar: time spent in function evaluations */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_FORCESNLPsolver_float((&(info.fevalstime)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "fevalstime", outvar);


		/* scalar: time spent solving inner QPs */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_FORCESNLPsolver_float((&(info.QPtime)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "QPtime", outvar);


		/* scalar: iterations spent in solving inner QPs */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_solver_int32_default((&(info.QPit)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "QPit", outvar);


		/* scalar: last exitflag of inner QP solver */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		copyCArrayToM_solver_int32_default((&(info.QPexitflag)), mxGetPr(outvar), 1);
		mxSetField(plhs[2], 0, "QPexitflag", outvar);


		/* column vector of length 8: solver ID of FORCESPRO solver */
		outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
		copyCArrayToM_solver_int32_default((&(info.solver_id[0])), mxGetPr(outvar), 8);
		mxSetField(plhs[2], 0, "solver_id", outvar);

	}
}
