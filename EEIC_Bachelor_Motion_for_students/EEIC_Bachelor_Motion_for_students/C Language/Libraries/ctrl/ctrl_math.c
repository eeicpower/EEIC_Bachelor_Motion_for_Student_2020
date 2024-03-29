/************************************************************************************
CONTROL MATH MODULE
-------------------
Descr.:		mathematic module for control algebra calculations
Boards:		PE-Expert3, C6713A DSP (floating point calculation)
System:		Yuna single axis ball-screw experimental setup
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/
/**
 * 改変メモ: float -> double に置換しました by 坂井
 */
#include "ctrl_math.h"

void ctrl_math_state(double A[], double x[], double B[], double u[], double *dx, int nrofs, int nrofi)
{
	double C1[NMAX] = { 0.0 }; double C2[NMAX] = { 0.0 };
	ctrl_matrix_prod(A, x, C1, nrofs, nrofs, 1);
	ctrl_matrix_prod(B, u, C2, nrofs, nrofi, 1);
	ctrl_matrix_add(C1, C2, &*dx, nrofs, 1);
}


void ctrl_math_output(double C[], double x[], double D[], double u[], double *y, int nrofs, int nrofi, int nrofo)
{
	double C1[NMAX] = { 0.0 }; double C2[NMAX] = { 0.0 };
	ctrl_matrix_prod(C, x, C1, nrofo, nrofs, 1);
	ctrl_matrix_prod(D, u, C2, nrofo, nrofi, 1);
	ctrl_matrix_add(C1, C2, &*y, nrofo, 1);
}


void ctrl_matrix_prod(double a[], double b[], double *c, int row_a, int col_a, int col_b)
{
	int i, j, k;
	for (i = 0; i < row_a; i++){
		for (j = 0; j < col_b; j++){ double sum = 0;
			for (k = 0; k < col_a; k++){ sum += a[i*col_a + k] * b[k*col_b + j]; }
			c[i*col_b + j] = sum;
		}
	}
}


void ctrl_matrix_add(double a[], double b[], double *c, int row, int col)
{
	int i, j;
	for (i = 0; i < row; i++){
		for (j = 0; j < col; j++){
			c[i*col + j] = a[i*col + j] + b[i*col + j];
		}
	}
}

