#define S_FUNCTION_LEVEL 2 
#define S_FUNCTION_NAME RFOC
#include "simstruc.h" 
#include <math.h> 

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/ 

static void mdlInitializeSizes(SimStruct *S){
	ssSetNumDiscStates(S, 25); // 25 DISCRETE STATE USED
	if (!ssSetNumInputPorts(S, 1)) return; 
	ssSetInputPortWidth(S, 0, 18); // 18 INPUT
	ssSetInputPortDirectFeedThrough(S, 0, 1); 
	ssSetInputPortOverWritable(S, 0, 1); 
	if (!ssSetNumOutputPorts(S, 1)) return; 
	ssSetOutputPortWidth(S, 0, 11); // 11 OUTPUT
	ssSetNumSampleTimes(S, 1); 

	ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE 
	| SS_OPTION_DISCRETE_VALUED_OUTPUT));
	
} 

static void mdlInitializeSampleTimes(SimStruct *S){ 
	ssSetSampleTime(S, 0, 1e-4); 
	ssSetOffsetTime(S, 0, 0.0);
} 

#define MDL_INITIALIZE_CONDITIONS 
static void mdlInitializeConditions(SimStruct *S){ 
	real_T *X0 = ssGetRealDiscStates(S); 
	int_T nXStates = ssGetNumDiscStates(S); 
	InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 
	int_T i; 

	/* initialize the states to 0.0 */ 
	for (i=0; i < nXStates; i++) { 
	X0[i] = 0.0; 
	}
} 

static void mdlOutputs(SimStruct *S, int_T tid){ 
	real_T *Y = ssGetOutputPortRealSignal(S,0); 
	real_T *X = ssGetRealDiscStates(S); 
	
    // OUTPUT
    real_T Va, Vb, Vc;
    real_T Imr, Te, theta_e;
    real_T Isd_ref, Isd;
    real_T Isq_ref, Isq;
    real_T speed_est;
    
    Imr         = X[5];
    theta_e     = X[6];
    Te          = X[14];
    Isd_ref     = X[15];
    Isd         = X[16];
    Isq_ref     = X[17];
    Isq         = X[18];
    speed_est   = X[19];
    Va          = X[22];
    Vb          = X[23];
    Vc          = X[24];
    
    Y[0] = Va;
    Y[1] = Vb;
    Y[2] = Vc;
    Y[3] = Imr;
    Y[4] = Te;
    Y[5] = theta_e;
    Y[6] = Isd_ref;
    Y[7] = Isd;
    Y[8] = Isq_ref;
    Y[9] = Isq;
    Y[10] = speed_est;
} 

#define MDL_UPDATE 
static void mdlUpdate(SimStruct *S, int_T tid) {
 
	real_T *X = ssGetRealDiscStates(S); 
	InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 

	real_T dt = 1e-4;
	
	// IM MODEL'S PARAMETER (SKRIPSI)
    real_T N    = 2;
    real_T Lm   = 0.687409044;
    real_T Ls   = 0.702131818;
    real_T Lr   = 0.702131818;
    real_T Rs   = 14.03;
    real_T Rr   = 13.29;
    real_T J    = 0.00247;
    real_T B    = 0.00005;
    real_T C    = Lm * Lm - Lr * Ls;
    
    // IM MODEL'S PARAMETER (PAPER PROF FERI)
    /*
    real_T N    = 2;
    real_T Lm   = 0.2279;
    real_T Ls   = 0.2349;
    real_T Lr   = 0.2349;
    real_T Rs   = 2.76;
    real_T Rr   = 2.9;
    real_T J    = 0.0436;
    real_T B    = 0.0005;
    real_T C    = Lm * Lm - Lr * Ls;
    */
    
    // DECLARE VARIABLE
    real_T Isd_reff, Isd_ref, Isq_ref;
    
    real_T Ia, Ib, Ic;
    real_T Ialpha, Ibeta;
    real_T Isd_act, Isq_act;
    real_T Isd, Isq;
    
    real_T Va, Vb, Vc;
    real_T Vasat, Vbsat, Vcsat;
    real_T Valpha, Vbeta;
    real_T Vcd, Vcq;
    real_T Vsd, Vsq;
    real_T Vs_max;
    
    real_T error_Isd, error_Isq;
	real_T integral_error_Isd, integral_error_Isq;
	real_T integral_error_Isd_prv, integral_error_Isq_prv;
    real_T Kidp, Kidi, Kiqp, Kiqi;
    real_T Imr, Imr_prv;
    real_T Isd1, Isd1_prv;
    real_T Isq1, Isq1_prv;
    real_T Imr1, Imr1_prv;
    real_T usd, usq;
    real_T sigma;
    
    real_T omega_m_act;
    
    real_T omega_e_est;
    real_T theta_e, theta_e_est;
	real_T omega_sl;
    real_T Te;
    
    real_T Tdd, Tqd, Td;
    real_T T2   = Lr / Rr;
    
    real_T pi = 3.141592654;
    real_T pi2  = 2 * 22 / 7;

    real_T K = 0.8164965809; // akar(2/3)
    real_T L = 0.8660254038; // akar(3/2)
	
    sigma = 1 - Lm * Lm / (Ls * Lr);
    
    // FIELDWEAK
    real_T Vin, V_flt;
    real_T evab;
    real_T xevab, xevab_prv;
    real_T Kimrp, Kimri;
    real_T Imr_max, Imr_min;
    real_T Isq_max;
    real_T Ismax2;
    
    // OBSERVER
    real_T k, Kp_obv, Ki_obv;
    real_T Tr = (Lr / Rr);
    real_T a0 = (-Rs / (sigma * Ls) - (1 - sigma) / (sigma * Tr));
    real_T a1 = (Lm / (sigma * Ls * Lr * Tr));
    real_T a2 = (Lm / (sigma * Ls * Lr));
    real_T a3 = (Lm / Tr);
    real_T a4 = (1 / Tr);
    real_T b0 = 1 / (sigma * Ls);
    
    real_T Isd_est, Isd_est_prv;
    real_T Isq_est, Isq_est_prv;
    real_T error_Isd_obv, error_Isq_obv;
    real_T A, integral_A, integral_A_prv;
    real_T G1, G2, G3, G4;
    real_T fluks_d_est, fluks_d_est_prv;
    real_T fluks_q_est, fluks_q_est_prv;
    real_T speed_est;
    
    // INPUT
	// Current feedback
	Ia = U(0);
    Ib = U(1);
    Ic = U(2);
    
    // Current reference
    Isd_ref = U(3);
    Isq_ref = U(4);
    
    Vs_max = U(5);
    
    // Speed actual feedback
    omega_m_act = U(6);
    
    // Tuning
    Tdd = U(7);
    Tqd = U(8);
    Td = U(9);
    
    // Field Weakening
    Kimrp = U(10);
    Kimri = U(11);
    Imr_max = U(12);
    Imr_min = U(13);
    Ismax2 = U(14);
    
    // Observer
    k = U(15);
    Kp_obv = U(16);
    Ki_obv = U(17);
    
    Kidp = sigma * Ls / Tdd;
    Kidi = Rs / Tdd;
    Kiqp = sigma * Ls / Tqd;
    Kiqi = Rs / Tqd;
    
    // STATE
    integral_error_Isd_prv      = X[0];
    integral_error_Isq_prv      = X[1];
    Imr_prv                     = X[2];
    Isd1_prv                    = X[3];
    Isq1_prv                    = X[4];
    Imr1_prv                    = X[5];
    theta_e_est                 = X[6];
    xevab_prv                   = X[7];
    V_flt                       = X[8];
    Isd_est_prv                 = X[9];
    Isq_est_prv                 = X[10];
    fluks_d_est_prv             = X[11];
    fluks_q_est_prv             = X[12];
    integral_A_prv              = X[13];
    Isd                         = X[16];
    Isq                         = X[18];
    speed_est                   = X[19];
    Valpha                      = X[20];
    Vbeta                       = X[21];
    
    theta_e = theta_e_est;
    
    // TRANSFORM ABC TO ALPHA-BETA
	Ialpha = K * (Ia - 0.5 * Ib - 0.5 * Ic);
    Ibeta = K * L * (Ib - Ic);
	
	// TRANSFORM ALPHA-BETA TO DQ
	Isd_act = cos(theta_e) * Ialpha + sin(theta_e) * Ibeta;
    Isq_act = -sin(theta_e) * Ialpha + cos(theta_e) * Ibeta;
    
    Isd = (1 - dt * 150) * Isd + dt * 150 * Isd_act;
    Isq = (1 - dt * 150) * Isq + dt * 150 * Isq_act;
   
    // Isd = Isd_act;
    // Isq = Isq_act;
    
    Imr = Imr_prv + ((Isd - Imr_prv) * dt) / T2;
    if(fabs(Imr) <= 0.001) { Imr = 0.001; }
    
    Te = N * (1 - sigma) * Ls * Isq * Imr;
        
    // FLUX MODEL
    omega_sl = (Rr * Isq) / (Lr * Imr);
    omega_e_est = N * omega_m_act + omega_sl;
    theta_e_est = theta_e_est + omega_e_est * dt;
    // while(theta_e >= pi2) { theta_e -= pi2; }
    // while(theta_e < 0) { theta_e += pi2; }
    
    theta_e = theta_e_est;
    
    // FIELD WEAKENING
    /*
    Vin = (Valpha * Valpha) + (Vbeta * Vbeta);
    V_flt = (1 - dt * 10) * V_flt + dt * 10 * Vin;
    evab = Vs_max * Vs_max - V_flt;
    xevab = xevab_prv + dt * evab;
    Isd_ref = Kimrp * evab + Kimri * xevab;
    
    // Isd & Isq Limiter
    if(Isd_ref >= Imr_max) {Isd_ref = Imr_max;}
    if(Isd_ref <= Imr_min) {Isd_ref = Imr_min;}
    
    Isq_max = sqrt(Ismax2 - Isd_ref * Isd_ref);
    
    if(Isq_ref > Isq_max) {Isq_ref = Isq_max;}
    if(Isq_ref < -Isq_max) {Isq_ref = -Isq_max;}
    */
    // Isd_ref = ( Ls / Lm ) * sqrt((2 * Lr * (1 + (sigma * sigma)) * Te) / (2 * N));
    
    error_Isd = Isd_ref - Isd;
    error_Isq = Isq_ref - Isq;
    
    integral_error_Isd = integral_error_Isd_prv + error_Isd * dt;
    integral_error_Isq = integral_error_Isq_prv + error_Isq * dt;
    
    usd = Kidp * error_Isd + Kidi * integral_error_Isd;
    usq = Kiqp * error_Isq + Kiqi * integral_error_Isq;
    
    // DECOUPLING
    Isd1 = (Isd_ref * dt + Isd1_prv * Td) / (Td + dt);
    Isq1 = (Isq_ref * dt + Isq1_prv * Td) / (Td + dt);
    Imr1 = (Isd_ref * dt + Imr1_prv * T2) / (T2 + dt);
    
	Vcd = -(omega_e_est * Ls * sigma * Isq1);
    Vcq = omega_e_est * Ls * sigma * Isd1 + Ls * (1 - sigma) * omega_e_est * Imr1;
    
    Vsd = usd + Vcd;
    Vsq = usq + Vcq;
    
    // TRANSFORM DQ TO ALFA-BETA
    Valpha = Vsd * cos(theta_e) - Vsq * sin(theta_e);
    Vbeta = Vsd * sin(theta_e) + Vsq * cos(theta_e);
    
    // TRANSFORM ALFA-BETA TO ABC  
    Va = K * Valpha;
    Vb = K * (-0.5 * Valpha + L * Vbeta);
    Vc = K * (-0.5 * Valpha - L * Vbeta);
    
    Vasat = Va;
    Vbsat = Vb;
    Vcsat = Vc;
    
    //Voltage Limiter
    if(Va > Vs_max){
        Vasat = Vs_max; 
    }
    
    if(Va < (-Vs_max)){ 
        Vasat = -Vs_max; 
    }
    
    if(Vb > (Vs_max)){ 
        Vbsat = Vs_max; 
    }
    
    if(Vb < (-Vs_max)){ 
        Vbsat = -Vs_max; 
    }
    
    if(Vc > Vs_max){ 
        Vcsat = Vs_max; 
    }
    
    if(Vc < (-Vs_max)){ 
        Vcsat = -Vs_max; 
    }
    
    Va = Vasat;
    Vb = Vbsat;
    Vc = Vcsat;
    
    // TRANSFORM ABC TO ALPHA-BETA
    Valpha = K * (Va - 0.5 * Vb - 0.5 * Vc);
    Vbeta = K * L * (Vb - Vc);
	
	// TRANSFORM ALPHA-BETA TO DQ
	Vsd = cos(theta_e) * Valpha + sin(theta_e) * Vbeta;
    Vsq = -sin(theta_e) * Valpha + cos(theta_e) * Vbeta; 
    
    // OBSERVER
    Isd_est = Isd_est_prv;
    Isq_est = Isq_est_prv;
    fluks_d_est = fluks_d_est_prv;
    fluks_q_est = fluks_q_est_prv;
    
    error_Isd_obv = Isd - Isd_est;
	error_Isq_obv = Isq - Isq_est;
	A = (error_Isd_obv * fluks_q_est) - (error_Isq_obv * fluks_d_est);
	integral_A = integral_A_prv + A * dt;
	speed_est = Kp_obv * A + Ki_obv * integral_A;
	
	G1 = ((k - 1.0) / k) * ((-Rs / (sigma * Ls)) - (Rr / (sigma * Lr)));
	G2 = -((k - 1.0) / k) * (N * speed_est);
    // G3 = ((k - 1.0) / (k * (speed_est * speed_est * N * N * Tr * Tr + 1.0) * Lm)) * ((Rs * Rr * Tr) + (Ls * Rr) - (sigma * Tr * Ls * Lr * speed_est * speed_est * N * N));
    G3 = ((k - 1.0) / (k * (speed_est * speed_est * N * N * Tr * Tr + 1.0) * Lm)) * ((Rs * Lr) + (Ls * Rr) - (sigma * Tr * Ls * Lr * speed_est * speed_est * N * N));
    G4 = ((k - 1.0) / (k * (speed_est * speed_est * N * N * Tr * Tr + 1.0) * Lm)) * (((Rs * Lr * Tr) + (Rr * Ls * Tr) + (sigma * Ls * Lr)) * speed_est * N);
	
	Isd_est = Isd_est_prv + dt * (a0 * Isd_est_prv + omega_e_est * Isq_est_prv + 
            a1 * fluks_d_est_prv + a2 * N * speed_est * fluks_q_est_prv + b0 * Vsd + 
            G1 * error_Isd_obv - G2 * error_Isq_obv);
    
	Isq_est = Isq_est_prv + dt * (-omega_e_est * Isd_est_prv + a0 * Isq_est_prv - 
            a2 * N * speed_est * fluks_d_est_prv + a1 * fluks_q_est_prv + b0 * Vsq + 
            G2 * error_Isd_obv + G1 * error_Isq_obv);
	
	fluks_d_est = fluks_d_est_prv + dt * (a3 * Isd_est_prv - a4 * fluks_d_est_prv + 
            (omega_e_est - (speed_est * N)) * fluks_q_est_prv + G3 * error_Isd_obv - 
           G4 * error_Isq_obv);
	
	fluks_q_est = fluks_q_est_prv + dt * (a3 * Isq_est_prv - 
            (omega_e_est - (speed_est * N)) * fluks_d_est_prv - a4 * fluks_q_est_prv + 
            G4 * error_Isd_obv + G3 * error_Isq_obv);
    
    // UPDATE STATE
    X[0] = integral_error_Isd;
    X[1] = integral_error_Isq;
    X[2] = Imr;
    X[3] = Isd1;
    X[4] = Isq1;
    X[5] = Imr1;
    X[6] = theta_e;
    X[7] = xevab;
    X[8] = V_flt;
    X[9] = Isd_est;
    X[10] = Isq_est;
    X[11] = fluks_d_est;
    X[12] = fluks_q_est;
    X[13] = integral_A;
    X[14] = Te;
    X[15] = Isd_ref;
    X[16] = Isd;
    X[17] = Isq_ref;
    X[18] = Isq;
    X[19] = speed_est;
    X[20] = Valpha;
    X[21] = Vbeta;
    X[22] = Va;
    X[23] = Vb;
    X[24] = Vc;
}

static void mdlTerminate(SimStruct *S) 
{ } /*Keep this function empty since no memory is allocated*/ 

#ifdef MATLAB_MEX_FILE 
/* Is this file being compiled as a MEX-file? */ 
#include "simulink.c" /*MEX-file interface mechanism*/ 
#else 
#include "cg_sfun.h" /*Code generation registration function*/ 
#endif
