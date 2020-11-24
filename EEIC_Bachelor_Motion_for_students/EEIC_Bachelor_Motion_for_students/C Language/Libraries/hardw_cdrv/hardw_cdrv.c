#include <gplib.h>
#include <RtGpShm.h>	// Global Rt/Gp Shared memory pointers
//-------------------------------------------------------------
// The following is a projpp created file from the User defines
//-------------------------------------------------------------
#include "../../Include/pp_proj.h"

/************************************************************************************
MOTOR POWER-ELEC MODULE
-----------------------
Descr.:		hardware module for current driver
Boards:		PMAC CK3M AX1212N NX-ECC203 AD4608 
PWelec:		ServoTechno LA220
Author:		Yui Shirato
			Koseki Lab, University of Tokyo, 2018
************************************************************************************/
//#include <gplib.h>
//#include <stdio.h>
//#include <dlfcn.h>


#include <math.h>

//----------------------------------------------------------------------------------
// pp_proj.h is the C header for accessing PMAC Global, CSGlobal, Ptr vars
// _PPScriptMode_ for Pmac Script like access global & csglobal
// _EnumMode_ for Pmac enum data type checking on Set & Get global functions
//------------------------------------------------------------------------------------
// #define _PPScriptMode_	// uncomment for Pmac Script type access
// #define _EnumMode_			// uncomment for Pmac enum data type checking on Set & Get global functions		


//#include "../../Include/ECATMap.h" //add ADin
#include "hardw_cdrv.h"

#define TRQ_MAX 5 // [Nm]
#define MAX_VAL 32767
#define TRQ_CONST 0.9800 // [Nm/V]

// hardware input 10 V -> (32767 << 16)
void hardw_trqref(double trqref)
{
	volatile struct GateArray3  *MyGate3;
	MyGate3 = GetGate3MemPtr(0);
	
	// saturation
	if (trqref > TRQ_MAX){
    trqref = TRQ_MAX;
	} else if (trqref < -TRQ_MAX) {
		trqref = -TRQ_MAX;
	}

	MyGate3->Chan[0].Dac[0] = (int)(trqref / TRQ_CONST * MAX_VAL) << 16;
}