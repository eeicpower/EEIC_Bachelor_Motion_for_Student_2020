#include <gplib.h>
#include <stdio.h>
#include <dlfcn.h>
//----------------------------------------------------------------------------------
// pp_proj.h is the C header for accessing PMAC Global, CSGlobal, Ptr vars
// _PPScriptMode_ for Pmac Script like access global & csglobal
// _EnumMode_ for Pmac enum data type checking on Set & Get global functions
//------------------------------------------------------------------------------------
// #define _PPScriptMode_	// uncomment for Pmac Script type access
// #define _EnumMode_			// uncomment for Pmac enum data type checking on Set & Get global functions		

#include "../../Include/pp_proj.h"
#include <RtGpShm.h>

#include "../../Libraries/ctrl/ctrl_step.h"
#include "../../Libraries/ctrl/ctrl_chirp.h"
#include "../../Libraries/ctrl/ctrl_func.h"
#include "../../Libraries/hardw_cdrv/hardw_cdrv.h"
#include "rticplc.h"

int counter=0; // time in millesec
double t=0.0;  // real time in sec
int flag_init =0; //if flag_init = 0, initialize
int flag_exptype=0;// if flag_exptype = 1, chirp signal / flag_exptype=2, PID control algorithm starts  
double ran; // range
double ref_out = 0.0;// reference command generated by function ctrl_step      
extern volatile TF2_INF		gstCpidInf[1];
extern volatile TF1_INF		gstCpdInf[1];
extern volatile TF2_INF		gstLPFInf[1];
double ctrl_cmd=0.0; //input to a motor[V]
double v0 = 0.0;
double v1 = 0.0;
double v2 = 0.0;
double temp1 = 0.0;
double temp2 = 0.0;
double vdistsim = 0.0;
double y = 0.0;
double const_ref_num = 0.0;
double initial_pos_num = 0.0;
double amp_sinref_num = 0.0;
double qout = 0.0;
double invqout = 0.0;
double qout_buff[3];
double invqout_buff[3];
double v1_buff[3];
double actPos_buff[3];
double vdistest = 0.0;
extern volatile TF2_INF		LFmath[1];
extern volatile TF2_INF		INVQmath[1];
int flg_chirp = 0;
int jj = 0;

void realtimeinterrupt_plcc()
{	
	//don't change
	volatile struct GateArray3  *MyGate3;//
	MyGate3 = GetGate3MemPtr(0); //

	flag_exptype=pshm->P[50];
	pshm->P[1]=pshm->Motor[1].ActVel;//velocity[number/s]
	pshm->P[2]=ref_out;//reference [number]
	pshm->P[3]=pshm->Motor[1].PosError;//error [number]
	pshm->P[5] = ctrl_cmd;
	pshm->P[6] = pshm->Motor[1].ActPos;//output [number]
	pshm->P[7] = v0; 
	pshm->P[8] = v1;
	pshm->P[9] = vdistsim;
	pshm->P[10] = vdistest;
	pshm->P[98]=t;//[s]
	pshm->P[99]=counter;
	pshm->P[100] = initial_pos_num;
	

	//initialize shirato added for FB control
	if(flag_init == 0){
		func_TF2StateInit( &gstCpidInf[0] ); //initialize of Cfb
		func_CpidParaInit( &gstCpidInf[0]); //setting parameters of Cfb
		func_TF1StateInit( &gstCpdInf[0]);
		func_CpdParaInit( &gstCpdInf[0]);
		func_TF2StateInit( &LFmath[0]);
		func_LFmathParaInit( &LFmath[0]);
		func_TF2StateInit( &INVQmath[0]);
		func_INVQmathParaInit( &INVQmath[0]); 

		for (jj = 0; jj < 2 + 1; jj++) {
		qout_buff[jj] = 0;	
		invqout_buff[jj] = 0;
		v1_buff[jj] = 0;
		actPos_buff[jj] = 0;
        }

		ctrl_cmd = 0.0;
		flag_init = 1;
		initial_pos_num = pshm->Motor[1].ActPos;//num
	}
 
 
	if(flag_exptype == 0){//no output
		//output
		ctrl_cmd = 0.0;
		hardw_vref(0); //change ctrl_cmd[V] to -32768-32767 and output
	
	}

	if(flag_exptype == 1){//chirp sine identification

		ctrl_chirp(t, &ctrl_cmd); // linear chirp

		//time count
		t=counter/1000.0;
		counter=counter+1;
		hardw_vref(ctrl_cmd); //change ctrl_cmd[V] to -32768-32767 and output
	
	}
	if(flag_exptype==2)//PID OK
	{
		//step
		hardw_angle_fromdeg_tonum(STEP_REF_AMP_DEG, &const_ref_num); //change reference from deg to num
		ctrl_step_input_with_end(const_ref_num+initial_pos_num, 0, 5, t, &ref_out); //make step reference
		//sin
		//hardw_angle_fromdeg_tonum(STEP_REF_AMP_DEG, &amp_sinref_num); 
		//ref_out = initial_pos_num + amp_sinref_num*sin(1*2*3.14*t);

		pshm->Motor[1].DesPos=ref_out; 
		pshm->Motor[1].PosError=pshm->Motor[1].DesPos-pshm->Motor[1].ActPos;  
		if (t<5){
		//ctrl_cmd = func_TF2Exe_AntiWindUp( pshm->Motor[1].PosError, &gstCpidInf[0],-5.0,5.0);//FB control output: ctrl_cmd[V]
		ctrl_cmd = func_TF1Exe( pshm->Motor[1].PosError, &gstCpdInf[0]);//FB control output: ctrl_cmd[V]
		}
		else{
		ctrl_cmd = 0.0;
		}
		//time count
			t=counter/1000.0;
			counter=counter+1;
			hardw_vref(ctrl_cmd); //change ctrl_cmd[V] to -32768-32767 and output

	}
	if(flag_exptype==3){//disturbance observer
		//reference
		const_ref_num = 0.0;
		if(t>0){
		ref_out = const_ref_num + initial_pos_num;
		} 

		//fbout=v0, qin = v1
		pshm->Motor[1].DesPos=ref_out; 
		pshm->Motor[1].PosError=pshm->Motor[1].DesPos-pshm->Motor[1].ActPos;  
		v0=func_TF1Exe( pshm->Motor[1].PosError, &gstCpdInf[0]);//FB control output: ctrl_cmd[V]
		v2 = qout - invqout;
		// v1 = v0 + v2; // shut loop
		v1 = v0;
		vdistest = -v2;
		qout = func_TF2Exe( v1, &LFmath);
		invqout = func_TF2Exe(pshm->Motor[1].ActPos, &INVQmath);

		//disturbance input
		vdistsim = 2;
		ctrl_cmd = v1 + vdistsim;

		//time count
		t=counter/1000.0;
		counter=counter+1;
		hardw_vref(ctrl_cmd); //change ctrl_cmd[V] to -32768-32767 and output

		for(jj =2; jj > 0; jj--){ 
				qout_buff[jj] = qout_buff[jj-1];  
				invqout_buff[jj] =  invqout_buff[jj-1];
				v1_buff[jj] = v1_buff[jj-1];
				actPos_buff[jj] = actPos_buff[jj-1];
		   }
	}

}


	 







