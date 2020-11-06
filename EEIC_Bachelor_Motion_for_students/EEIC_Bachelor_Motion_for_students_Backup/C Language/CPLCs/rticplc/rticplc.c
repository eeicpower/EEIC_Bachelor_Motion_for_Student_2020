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
extern volatile TF2_INF		gstObsBlk1Inf[1];
extern volatile TF2_INF		gstObsBlk2Inf[1];
extern volatile TF2_INF		gstLPFInf[1];
double ctrl_cmd=0.0; //input to a motor[V]
double v0 = 0.0;
double v1 = 0.0;
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
extern volatile TF2_INF		LFmath[1];
extern volatile TF2_INF		INVQmath[1];
int flg_chirp = 0;


void realtimeinterrupt_plcc()
{	
	//don't change
	volatile struct GateArray3  *MyGate3;// Gate3?p?\?????????`
	MyGate3 = GetGate3MemPtr(0); // Acc24E3[0]??A?h???X????

	flag_exptype=pshm->P[50];
	pshm->P[1]=pshm->Motor[1].ActVel;//velocity[number/s]
	pshm->P[2]=ref_out;//reference [number]
	pshm->P[3]=pshm->Motor[1].PosError;//error [number]
	pshm->P[5] = ctrl_cmd;
	pshm->P[6] = pshm->Motor[1].ActPos;//output [number]
	pshm->P[7] = v0; 
	pshm->P[8] = v1;
	pshm->P[9] = vdistsim;
	pshm->P[10] = flg_chirp;
	pshm->P[98]=t;//[s]
	pshm->P[99]=counter;
	pshm->P[100] = initial_pos_num;
	//ctrl_cmd = pshm->P[100]; 
	

	//initialize shirato added for FB control
	if(flag_init == 0){
		func_TF2StateInit( &gstCpidInf[0] ); //initialize of Cfb
		func_CpidParaInit( &gstCpidInf[0]); //setting parameters of Cfb
		func_TF2StateInit( &gstObsBlk1Inf[0]); //initialize of first block of disturbance observer
		func_ObsBlk1ParaInit( &gstObsBlk1Inf[0] ); //setting paramteters of first block of disturbance observer
		func_TF1StateInit( &gstObsBlk2Inf[0]); 
		func_ObsBlk2ParaInit( &gstObsBlk2Inf[0] ); 
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
	ctrl_chirp(t, &ctrl_cmd);
	/*
	if(t < CHIRP_TIME){
		flg_chirp = 1;
		ctrl_chirp(t, &ctrl_cmd);
	}
	else{
		flg_chirp = 0;
	}
	*/
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
	/*
	//does not work well
	pshm->Motor[1].DesPos=ref_out; 
	pshm->Motor[1].PosError=pshm->Motor[1].DesPos-pshm->Motor[1].ActPos;  
	//v0 = func_TF2Exe( pshm->Motor[1].PosError, &gstCpidInf[0]);//FB control output: ctrl_cmd[V]
	v0 = func_TF1Exe( pshm->Motor[1].PosError, &gstCpdInf[0]);//FB control output: ctrl_cmd[V]
	y = pshm->Motor[1].ActPos;
	temp1 = func_TF2Exe( v0, &gstObsBlk1Inf[0] );
	temp2 = func_TF1Exe( y, &gstObsBlk2Inf[0] );
	v1 = temp1-temp2;
	*/
	

	//fbout=v0, qin = v1
	 pshm->Motor[1].DesPos=ref_out; 
	 pshm->Motor[1].PosError=pshm->Motor[1].DesPos-pshm->Motor[1].ActPos;  
	 v0=func_TF1Exe( pshm->Motor[1].PosError, &gstCpdInf[0]);//FB control output: ctrl_cmd[V]
	 v1 = v0 - invqout + qout;
	 qout = func_TF2Exe( v1, &LFmath);
	 invqout = func_TF2EXe(pshm->Motor[1].ActPos, &INVQmath);
	//qout_buff[0] = func_TFOUT(v1_buff, qout_buff, &LFmath[0]); //if func_TF2Exe error
	//invqout_buff[0] = func_TFOUT(actPos_buff, invqout_buff, &INVQmath[0]);

	//disturbance input
	vdistsim = 0.0;
	//ctrl_step_input_with_end(STEP_DIST_AMP_V, 2, 5, t, &vdistsim); //make step reference
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

if(flag_exptype == 4){//for check of hardware output
	//check output voltage
	//ctrl_cmd = 1; //check if output voltage is 1[V] with a tester
	//ctrl_cmd = 1.5 * sin(2*3.14 * 1*t);
	ctrl_chirp(t, &ctrl_cmd);
	//check reference 
	hardw_angle_fromdeg_tonum(STEP_REF_AMP_DEG, &const_ref_num); //check const_ref_num = 78125
	ctrl_step_input_with_end(const_ref_num, 0, 5, t, &ref_out); //check ref_out is a step of 78125
		//time count
		t=counter/1000.0;
		counter=counter+1;
		hardw_vref(ctrl_cmd); //change ctrl_cmd[V] to -32768-32767 and output
}

if(flag_exptype == 5){//test openloop
	
	//ctrl_cmd = 1.3 + 0.1 * sin(2 * 1*3.14 * t);
	ctrl_cmd = 1;
	
	//time count
	t=counter/1000.0;
	counter=counter+1;
	hardw_vref(ctrl_cmd); //change ctrl_cmd[V] to -32768-32767 and output
	
}


	
}


	 







