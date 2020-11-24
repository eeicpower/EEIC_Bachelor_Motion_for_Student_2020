/************************************************************************************
FUNCTION MODULE
----------------------
Descr.:		setting gain of FF controller 
Boards:		
Author:		Yui Shirato
Koseki Lab, University of Tokyo, 2018
*************************************************************************************/


#ifndef CTRL_FUNC_H
#define	CTRL_FUNC_H

typedef struct {
	double	dA[2];
	double	dB[2];
	double	dInPre[1];
	double	dOutPre[1];
} TF1_INF;


typedef struct {
	double	dA[3];
	double	dB[3];
	double	dInPre[2];
	double	dOutPre[2];
} TF2_INF;


extern volatile TF2_INF		gstCpidInf[1];
extern volatile TF2_INF		gstLPFInf[1];
extern volatile TF1_INF		gstCpdInf[1];
extern volatile TF2_INF		LFmath[1];
extern volatile TF2_INF		INVQmath[1];

/*	func_TF2StateInit
**	-------------------
**	DES:	initialization of controller
*/
void	func_TF1StateInit( volatile TF1_INF *stpInf);
void	func_TF2StateInit( volatile TF2_INF *stpInf );

void	func_CpidParaInit( volatile TF2_INF *stpInf );

void	func_CpdParaInit(volatile TF1_INF *stpInf);
void	func_LPFParaInit( volatile TF2_INF *stpInf );
void	func_LFmathParaInit( volatile TF2_INF *stpInf);
void	func_INVQmathParaInit(volatile TF2_INF *stpInf);
/* func_TF2Exe
**	-------------------
**	DES:	making controller output 1st order
**	IN:		controller input
**	OUT:	data of controller input, pre-input and output,pre-output
*/
double 	func_TF1Exe( double dIn, volatile TF1_INF *stpInf );
double 	func_TF2Exe( double dIn, volatile TF2_INF *stpInf );
double	func_TF2Exe_AntiWindUp( double dIn, volatile TF2_INF *stpInf, double dLimit_low, double dLimit_high);
//double  func_TFOUT( double dIn[],double dOut[], volatile CTRL_dob *stpInf);
#endif