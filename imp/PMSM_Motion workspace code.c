/* =================================================================================
File name  : PMSM_Motion.c
Modify     : MAIWEI /sales@mcudsp.com.tw
Website    : http://www.mcudsp.com.tw
Version    : V3.3
Target     : easyDSP-Motor-Driver V4.0
Description: PMSM Motor Motion Application
==================================================================================*/
#include "DSP28x_Project.h"      // Device Header file and Examples Include File
#include "easyDSP-F283xMotor.h"  // easyDSP-Driver Board function definitions
#include "easyDSP_Motion_Lab_Settings.h"
#include "IQmathLib.h"
#include "PMSM_Sensorless.h"
#include <math.h>

///LEVEL 3 Current sensor offset calibration Modify Here////////
_iq cal_offset_A = _IQ15(0.503775775);
_iq cal_offset_B = _IQ15(0.50223428);
////////////////////////////////////////////////////////////////

// Global variables used in this system
float32 T = 0.001/ISR_FREQUENCY;    // Samping period (sec), see parameter.h
Uint16  SpeedLoopPrescaler = 10;     // Speed loop prescaler
Uint16  SpeedLoopCount = 1;          // Speed loop counter
Uint16  lsw=0;
Uint16  Init_IFlag=0;
Uint16  RunMotor = 0;
int16   MotorSpeed_RPM=0;
static Uint16 kpUpdateCnt = 0;
Uint16 kpUpdatePrescaler = 20;
float   DcBusVolt =0;
#define DcBusVolt_Gain 326.78

_iq VdTesting = _IQ(-0.1);         // Vd reference (pu)
_iq VqTesting = _IQ(0.1);           // Vq reference (pu)
_iq IdRef = _IQ(0.035);             // Id reference (pu)
_iq IqRef = _IQ(0.0);             // Iq reference (pu)
_iq IdRefprev = _IQ(0.0);
_iq IqRefprev = _IQ(0.0);
_iq SpeedRef = _IQ(0.0);           // Speed reference (pu)
_iq cal_filt_gain;
_iq SpeedRef_CMD = _IQ(0.0);        // Speed Ref Input (pu)
_iq ramper(_iq, _iq, _iq);          // slew programmable ramper

_iq Ipeak = _IQ(0.0);   //Three phase motor current peak
_iq a[700];
_iq b[700];

int n=0;
int s=0;
int sc=0;
int sc1=0;

extern volatile Uint16 EnableFlag;

int16 PwmDacCh1=0;
int16 PwmDacCh2=0;
int16 PwmDacCh3=0;
int16 PwmDacCh4=0;

int16 DlogCh1 = 0;
int16 DlogCh2 = 0;
int16 DlogCh3 = 0;
int16 DlogCh4 = 0;

#if (BUILDLEVEL==LEVEL1)
Uint16 DRV_RESET = 1;
#else
Uint16 DRV_RESET = 0;
#endif

// Instance a QEP interface driver
QEP qep1 = QEP_DEFAULTS;

// Instance a position estimator
SMOPOS smo1 = SMOPOS_DEFAULTS;

// Instance a sliding-mode position observer constant Module
SMOPOS_CONST smo1_const = SMOPOS_CONST_DEFAULTS;

// Instance a few transform objects
CLARKE clarke1 = CLARKE_DEFAULTS;
PARK park1 = PARK_DEFAULTS;
IPARK ipark1 = IPARK_DEFAULTS;

// Instance PID regulators to regulate the d and q  axis currents, and speed
PID_GRANDO_CONTROLLER pid1_id = {PID_TERM_DEFAULTS,PID_PARAM_DEFAULTS,PID_DATA_DEFAULTS};
PID_GRANDO_CONTROLLER pid1_iq = {PID_TERM_DEFAULTS,PID_PARAM_DEFAULTS,PID_DATA_DEFAULTS};
PID_GRANDO_CONTROLLER pid1_spd = {PID_TERM_DEFAULTS,PID_PARAM_DEFAULTS,PID_DATA_DEFAULTS};

// Instance a PWM driver instance
PWMGEN pwm1 = PWMGEN_DEFAULTS;

// Instance a PWM DAC driver instance
PWMDAC pwmdac1 = PWMDAC_DEFAULTS;

// Instance a Space Vector PWM modulator. This modulator generates a, b and c
// phases based on the d and q stationery reference frame inputs
SVGENDQ svgen_dq1 = SVGENDQ_DEFAULTS;

// Instance a ramp controller to smoothly ramp the frequency
RMPCNTL rc1 = RMPCNTL_DEFAULTS;

//  Instance a ramp generator to simulate an Anglele
RAMPGEN rg1 = RAMPGEN_DEFAULTS;

//  Instance a phase voltage calculation
PHASEVOLTAGE volt1 = PHASEVOLTAGE_DEFAULTS;

// Instance a speed calculator based on QEP
SPEED_MEAS_QEP speed1 = SPEED_MEAS_QEP_DEFAULTS;

// Instance a speed calculator based on sliding-mode position observer
SPEED_ESTIMATION speed3 = SPEED_ESTIMATION_DEFAULTS;

// Create an instance of DATALOG Module
DLOG_4CH dlog = DLOG_4CH_DEFAULTS;

void PMSM_Motion_Init()
{
// CPU_TIMER2_ISR_TASK,ISR functions found within this file.
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TINT2 = &CPU_TIMER2_ISR_TASK;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    InitCpuTimers();   // For this example, only initialize the Cpu Timers
// Configure CPU-Timer 2 to interrupt every second: 150MHz CPU Frequency, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer2, 150, 500000);
    CpuTimer2Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0

// Initialize PWM module
    pwm1.PeriodMax = SYSTEM_FREQUENCY*1000000*T/2;  // Prescaler X1 (T1), ISR period = T x 1
    PWM_INIT_MACRO(pwm1)

// Initialize PWMDAC module
    pwmdac1.PeriodMax = 500;   // @60Mhz: 1500->20kHz, 1000-> 30kHz, 500->60kHz
    pwmdac1.PwmDacInPointer0 = &PwmDacCh1;
    pwmdac1.PwmDacInPointer1 = &PwmDacCh2;
    pwmdac1.PwmDacInPointer2 = &PwmDacCh3;
    pwmdac1.PwmDacInPointer3 = &PwmDacCh4;
    PWMDAC_INIT_MACRO(pwmdac1);

// Initialize DATALOG module
    dlog.iptr1 = &DlogCh1;
    dlog.iptr2 = &DlogCh2;
    dlog.iptr3 = &DlogCh3;
    dlog.iptr4 = &DlogCh4;
    dlog.trig_value = 0x1;
    dlog.size = 0x00c8;
    dlog.prescalar = 5;
    dlog.init(&dlog);

// Initialize ADC module
    ADC_MACRO();

// Initialize QEP module
    qep1.LineEncoder = Encoder;
    qep1.MechScaler = _IQ30(0.25/qep1.LineEncoder);
    qep1.PolePairs = POLES/2;
    qep1.CalibratedAngle = 0;
    QEP_INIT_MACRO(qep1);

// Initialize the Speed module for QEP based speed calculation
    speed1.K1 = _IQ21(1/(BASE_FREQ*T));
    speed1.K2 = _IQ(1/(1+T*2*PI*5));  // Low-pass cut-off frequency
    speed1.K3 = _IQ(1)-speed1.K2;
    speed1.BaseRpm = 120*(BASE_FREQ/POLES);

// Initialize the SPEED_EST module SMOPOS based speed calculation
    speed3.K1 = _IQ21(1/(BASE_FREQ*T));
    speed3.K2 = _IQ(1/(1+T*2*PI*5));  // Low-pass cut-off frequency
    speed3.K3 = _IQ(1)-speed3.K2;
    speed3.BaseRpm = 120*(BASE_FREQ/POLES);

// Initialize the RAMPGEN module
    rg1.StepAngleMax = _IQ(BASE_FREQ*T);

// Initialize the SMOPOS constant module
    smo1_const.Rs = RS;
    smo1_const.Ls = LS;
    smo1_const.Ib = BASE_CURRENT;
    smo1_const.Vb = BASE_VOLTAGE;
    smo1_const.Ts = T;
    SMO_CONST_MACRO(smo1_const);

// Initialize the SMOPOS module
    smo1.Fsmopos = _IQ(smo1_const.Fsmopos);
    smo1.Gsmopos = _IQ(smo1_const.Gsmopos);
    smo1.Kslide = _IQ(0.05308703613);
    smo1.Kslf = _IQ(0.1057073975);

// Initialize the PID_GRANDO_CONTROLLER module for Id
    pid1_id.param.Kp = _IQ(2.176*BASE_CURRENT/BASE_VOLTAGE);
    pid1_id.param.Kr = _IQ(1.0);
    pid1_id.param.Ki = _IQ(T/0.005);
    pid1_id.param.Kd = _IQ(0/T);
    pid1_id.param.Km = _IQ(1.0);
    pid1_id.param.Umax = _IQ(0.8);
    pid1_id.param.Umin = _IQ(-0.8);
    pid1_id.param.Q = _IQ(18.0);
    pid1_id.param.G = _IQ(90.0);
    pid1_id.param.B1 = _IQ(2.0);
    pid1_id.param.B2 = _IQ(2.0);
    pid1_id.param.Rs = _IQ(3.65);
    pid1_id.param.Ld = _IQ(0.00783);
    pid1_id.param.Lq = _IQ(0.00979);
    pid1_id.param.Psi = _IQ(0.06);
    pid1_id.param.cd = _IQ(1000.0);
    pid1_id.param.cq = _IQ(1000.0);
    pid1_id.param.Etad = _IQ(1000.0);
    pid1_id.param.Etaq = _IQ(1000.0);
    pid1_id.param.Alpha = _IQ(10000.0);
    pid1_id.param.Beta = _IQ(10000.0);
    pid1_id.param.alphad = _IQ(0.08);
    pid1_id.param.alphaq = _IQ(0.1);
    pid1_id.param.betad = _IQ(0.05);
    pid1_id.param.betaq = _IQ(0.05);
    pid1_id.param.KQ = _IQ(5.0);
    pid1_id.param.KD = _IQ(3.0);
    pid1_id.param.KPqint = _IQ(5.0);
    pid1_id.param.KPdint = _IQ(3.0);

// Initialize the PID_GRANDO_CONTROLLER module for Iq
    pid1_iq.param.Kp = _IQ(2.3*BASE_CURRENT/BASE_VOLTAGE);
    pid1_iq.param.Kr = _IQ(1.0);
    pid1_iq.param.Ki = _IQ(T/0.0005);
    pid1_iq.param.Kd = _IQ(0/T);
    pid1_iq.param.Km = _IQ(1.0);
    pid1_iq.param.Umax = _IQ(0.9);      //Limit Current Output
    pid1_iq.param.Umin = _IQ(-0.9);     //Limit Current Output
    pid1_iq.param.Q = _IQ(18.0);
    pid1_iq.param.G = _IQ(90.0);
    pid1_iq.param.B1 = _IQ(2.0);
    pid1_iq.param.B2 = _IQ(2.0);
    pid1_iq.param.Rs = _IQ(3.65);
    pid1_iq.param.Ld = _IQ(0.00783);
    pid1_iq.param.Lq = _IQ(0.00979);
    pid1_iq.param.Psi = _IQ(0.06);
    pid1_iq.param.cd = _IQ(5000.0);
    pid1_iq.param.cq = _IQ(5000.0);
    pid1_iq.param.Etad = _IQ(1000.0);
    pid1_iq.param.Etaq = _IQ(1000.0);
    pid1_iq.param.Alpha = _IQ(20000.0);
    pid1_iq.param.Beta = _IQ(20000.0);
    pid1_iq.param.alphad = _IQ(0.08);
    pid1_iq.param.alphaq = _IQ(0.1);
    pid1_iq.param.betad = _IQ(0.05);
    pid1_iq.param.betaq = _IQ(0.05);
    pid1_iq.param.KQ = _IQ(5.0);
    pid1_iq.param.KD = _IQ(3.0);
    pid1_iq.param.KPqint = _IQ(5.0);
    pid1_iq.param.KPdint = _IQ(3.0);

// Initialize the PID_GRANDO_CONTROLLER module for Speed
    //pid1_spd.param.Kp = _IQ(0.2*2*BASE_FREQ/BASE_CURRENT/(POLES/2));
    pid1_spd.param.Kp=_IQ(0.8);
    pid1_spd.param.Kr = _IQ(1.0);
     //pid1_spd.param.Ki = _IQ(T*SpeedLoopPrescaler/0.3);
    pid1_spd.param.Ki = _IQ(0.0025166666671);
    pid1_spd.param.Kd = _IQ(0/(T*SpeedLoopPrescaler));
    pid1_spd.param.Km = _IQ(1.0);
    pid1_spd.param.Umax = _IQ(0.95);
    pid1_spd.param.Umin = _IQ(-0.95);
    pid1_spd.param.Q = _IQ(19.5);
    pid1_spd.param.G = _IQ(69.0);
    pid1_spd.param.B1 = _IQ(0.5);
    pid1_spd.param.B2 = _IQ(2.0);
    pid1_spd.param.Rs = _IQ(3.65);
    pid1_spd.param.Ld = _IQ(0.00783);
    pid1_spd.param.Lq = _IQ(0.00979);
    pid1_spd.param.Psi = _IQ(0.06);
    pid1_spd.param.cd = _IQ(100.0);
    pid1_spd.param.cq = _IQ(1000.0);
    pid1_spd.param.Etad = _IQ(5000.0);
    pid1_spd.param.Etaq = _IQ(2000.0);
    pid1_spd.param.Alpha = _IQ(5000.0);
    pid1_spd.param.Beta = _IQ(5000.0);
    pid1_spd.param.alphad = _IQ(0.0);
    pid1_spd.param.alphaq = _IQ(0.0);
    pid1_spd.param.betad = _IQ(0.0);
    pid1_spd.param.betaq = _IQ(0.0);
    pid1_spd.param.KQ = _IQ(0.0);
    pid1_spd.param.KD = _IQ(0.0);
    pid1_spd.param.KPqint = _IQ(0.0);
    pid1_spd.param.KPdint = _IQ(0.0);


// Initialize the phase current offset calibration filter
    cal_filt_gain = _IQ15(T/(T+TC_CAL));

    // Enable CNT_zero interrupt using EPWM1 Time-base
    EPwm1Regs.ETSEL.bit.INTEN = 1;   // Enable EPWM1INT generation
    EPwm1Regs.ETSEL.bit.INTSEL = 1;  // Enable interrupt CNT_zero event
    EPwm1Regs.ETPS.bit.INTPRD = 1;   // Generate interrupt on the 1st event
    EPwm1Regs.ETCLR.bit.INT = 1;     // Enable more interrupts
    Driver_Protection();
    DELAY_US(100000);
}

void PMSM_Motion_LEVEL1()
{
#if (BUILDLEVEL==LEVEL1)
// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    rc1.TargetValue = SpeedRef_CMD;
    RC_MACRO(rc1);
// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1);
// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ipark1.Ds = VdTesting;
    ipark1.Qs = VqTesting;
    ipark1.Sine=_IQsinPU(rg1.Out);
    ipark1.Cosine=_IQcosPU(rg1.Out);
    IPARK_MACRO(ipark1);
// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    svgen_dq1.Ualpha = ipark1.Alpha;
    svgen_dq1.Ubeta = ipark1.Beta;
    SVGEN_MACRO(svgen_dq1);
// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = _IQtoQ15(-svgen_dq1.Ta);
    pwm1.MfuncC2 = _IQtoQ15(-svgen_dq1.Tb);
    pwm1.MfuncC3 = _IQtoQ15(-svgen_dq1.Tc);
    PWM_MACRO(pwm1);                        // Calculate the new PWM compare values

    EPwm1Regs.CMPA.half.CMPA=pwm1.PWM1out;  // PWM 1A - PhaseA
    EPwm2Regs.CMPA.half.CMPA=pwm1.PWM2out;  // PWM 2A - PhaseB
    EPwm3Regs.CMPA.half.CMPA=pwm1.PWM3out;  // PWM 3A - PhaseC
// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    PwmDacCh1 = _IQtoQ15(svgen_dq1.Ta);
    PwmDacCh2 = _IQtoQ15(svgen_dq1.Tb);
    PwmDacCh3 = _IQtoQ15(svgen_dq1.Tc);
    PwmDacCh4 = _IQtoQ15(svgen_dq1.Tb-svgen_dq1.Tc);
// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(svgen_dq1.Ta);
    DlogCh2 = _IQtoQ15(svgen_dq1.Tb);
    DlogCh3 = _IQtoQ15(svgen_dq1.Tc);
    DlogCh4 = _IQtoQ15(svgen_dq1.Tb-svgen_dq1.Tc);
// ------------------------------------------------------------------------------
//    Call the PWMDAC update macro.
// ------------------------------------------------------------------------------
    PWMDAC_MACRO(pwmdac1);
// ------------------------------------------------------------------------------
//    Call the DATALOG update function.
// ------------------------------------------------------------------------------
    dlog.update(&dlog);

#endif // (BUILDLEVEL==LEVEL1)
}

void PMSM_Motion_LEVEL2()
{
#if (BUILDLEVEL==LEVEL2)
// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    rc1.TargetValue = SpeedRef_CMD;
    if(rc1.TargetValue > _IQ(0.25))  rc1.TargetValue = _IQ(0.25); //Limit LEVEL2
    if(rc1.TargetValue < _IQ(-0.25)) rc1.TargetValue = _IQ(-0.25); //Limit LEVEL2
    RC_MACRO(rc1);
// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1);
// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//  Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
    clarke1.As=((AdcMirror.ADCRESULT0)*0.00024414-cal_offset_A)*2; // Phase A curr.
    clarke1.Bs=((AdcMirror.ADCRESULT1)*0.00024414-cal_offset_B)*2; // Phase B curr.
    CLARKE_MACRO(clarke1);
// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
    park1.Alpha = clarke1.Alpha;
    park1.Beta = clarke1.Beta;
    park1.Angle = rg1.Out;
    park1.Sine = _IQsinPU(park1.Angle);
    park1.Cosine = _IQcosPU(park1.Angle);
    PARK_MACRO(park1);
// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ipark1.Ds = VdTesting;
    ipark1.Qs = VqTesting;
    ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
    IPARK_MACRO(ipark1);
// ------------------------------------------------------------------------------
//  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
// ------------------------------------------------------------------------------
    volt1.DcBusVolt = ((AdcMirror.ADCRESULT2)*0.00024414); // DC Bus voltage meas.
    DcBusVolt = volt1.DcBusVolt*DcBusVolt_Gain;

    volt1.MfuncV1 = svgen_dq1.Ta;
    volt1.MfuncV2 = svgen_dq1.Tb;
    volt1.MfuncV3 = svgen_dq1.Tc;
    VOLT_MACRO(volt1);
// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    svgen_dq1.Ualpha = ipark1.Alpha;
    svgen_dq1.Ubeta = ipark1.Beta;
    SVGEN_MACRO(svgen_dq1);
// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = _IQtoQ15(-svgen_dq1.Ta);
    pwm1.MfuncC2 = _IQtoQ15(-svgen_dq1.Tb);
    pwm1.MfuncC3 = _IQtoQ15(-svgen_dq1.Tc);
    PWM_MACRO(pwm1);                                   // Calculate the new PWM compare values

    EPwm1Regs.CMPA.half.CMPA=pwm1.PWM1out;  // PWM 1A - PhaseA
    EPwm2Regs.CMPA.half.CMPA=pwm1.PWM2out;  // PWM 2A - PhaseB
    EPwm3Regs.CMPA.half.CMPA=pwm1.PWM3out;  // PWM 3A - PhaseC
// ------------------------------------------------------------------------------
//  Connect inputs of the PWCCMDAC module
// ------------------------------------------------------------------------------
    PwmDacCh1 = _IQtoQ15(volt1.VphaseA);
    PwmDacCh2 = _IQtoQ15(volt1.VphaseB);
    PwmDacCh3 = _IQtoQ15(volt1.Valpha );
    PwmDacCh4 = _IQtoQ15(volt1.Vbeta);
// ------------------------------------------------------------------------------
//  Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
//  Phase 2A Test
    DlogCh1 = _IQtoQ15(volt1.VphaseA);
    DlogCh2 = _IQtoQ15(volt1.VphaseB);
    DlogCh3 = _IQtoQ15(clarke1.As);//svolt1.Valpha);
    DlogCh4 = _IQtoQ15(clarke1.Bs);//volt1.Vbeta);
/*
//  Phase 2B Test
  DlogCh1 = _IQtoQ15(volt1.Valpha);
    DlogCh2 = _IQtoQ15(volt1.Vbeta);
    DlogCh3 = _IQtoQ15(clarke1.Alpha);
    DlogCh4 = _IQtoQ15(clarke1.Beta);
    */
// ------------------------------------------------------------------------------
//    Call the PWMDAC update macro.
// ------------------------------------------------------------------------------
    PWMDAC_MACRO(pwmdac1);
// ------------------------------------------------------------------------------
//    Call the DATALOG update function.
// ------------------------------------------------------------------------------
    dlog.update(&dlog);

#endif // (BUILDLEVEL==LEVEL2)
}

void PMSM_Motion_LEVEL3()
{
#if (BUILDLEVEL==LEVEL3)
    _iq IAfdbk;
    _iq IBfdbk;
// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
// ------------------------------------------------------------------------------
    IAfdbk=((AdcMirror.ADCRESULT0)*0.00024414-cal_offset_A)*2; // Phase A curr.
    IBfdbk=((AdcMirror.ADCRESULT1)*0.00024414-cal_offset_B)*2; // Phase B curr.// ((ADCmeas(q12)/2^12)-0.5)*2
// ------------------------------------------------------------------------------
//  LPF to average the calibration offsets
//  Use the offsets calculated here to initialize cal_offset_A and cal_offset_B
//  so that they are used for the remaining build levels
// ------------------------------------------------------------------------------
    cal_offset_A = _IQ15mpy(cal_filt_gain,_IQtoIQ15(IAfdbk)) + cal_offset_A;
    cal_offset_B = _IQ15mpy(cal_filt_gain,_IQtoIQ15(IBfdbk)) + cal_offset_B;

// ------------------------------------------------------------------------------
//  force all PWMs to 0% duty cycle
// ------------------------------------------------------------------------------
    EPwm1Regs.CMPA.half.CMPA=pwm1.PeriodMax;    // PWM 1A - PhaseA
    EPwm2Regs.CMPA.half.CMPA=pwm1.PeriodMax;    // PWM 2A - PhaseB
    EPwm3Regs.CMPA.half.CMPA=pwm1.PeriodMax;    // PWM 3A - PhaseC

// ------------------------------------------------------------------------------
//  Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    PwmDacCh1 = _IQtoQ15(IAfdbk);
    PwmDacCh2 = _IQtoQ15(IBfdbk);
    PwmDacCh2 = _IQtoQ15(cal_offset_A);
    PwmDacCh3 = _IQtoQ15(cal_offset_B);

// ------------------------------------------------------------------------------
//  Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(IAfdbk);
    DlogCh2 = _IQtoQ15(IBfdbk);
    DlogCh3 = _IQtoQ15(cal_offset_A);
    DlogCh4 = _IQtoQ15(cal_offset_B);
// ------------------------------------------------------------------------------
//    Call the PWMDAC update macro.
// ------------------------------------------------------------------------------
    PWMDAC_MACRO(pwmdac1);
// ------------------------------------------------------------------------------
//    Call the DATALOG update function.
// ------------------------------------------------------------------------------
    dlog.update(&dlog);

#endif // (BUILDLEVEL==LEVEL3)
}

void PMSM_Motion_LEVEL4()
{
#if (BUILDLEVEL==LEVEL4)
// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    if(lsw==0)rc1.TargetValue = 0;
    else {
        rc1.TargetValue = SpeedRef_CMD;
        if(rc1.TargetValue > POS_MAX_SPEED) rc1.TargetValue = POS_MAX_SPEED;
        if(rc1.TargetValue < NEG_MAX_SPEED) rc1.TargetValue = NEG_MAX_SPEED;
        }
    RC_MACRO(rc1);
// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1);
// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//  Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
    clarke1.As=((AdcMirror.ADCRESULT0)*0.00024414-cal_offset_A)*2; // Phase A curr.
    clarke1.Bs=((AdcMirror.ADCRESULT1)*0.00024414-cal_offset_B)*2; // Phase B curr.
    CLARKE_MACRO(clarke1);
// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
    park1.Alpha = clarke1.Alpha;
    park1.Beta = clarke1.Beta;
    if(lsw==0) park1.Angle = 0;
    else if(lsw==1) park1.Angle = rg1.Out;
    else if(lsw==2) park1.Angle = speed1.ElecTheta;

    park1.Sine = _IQsinPU(park1.Angle);
    park1.Cosine = _IQcosPU(park1.Angle);

    PARK_MACRO(park1);
// ------------------------------------------------------------------------------
//  Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID IQ controller macro
// ------------------------------------------------------------------------------
    if(lsw==0) {
        pid1_id.term.Ref = 0;
        pid1_id.term.RefId = 0;
        pid1_id.term.RefIq = 0;
    }
    else if(lsw==1) {
        pid1_id.term.Ref = IdRef;
        pid1_id.term.RefId = IdRef;
        pid1_id.term.RefIq = IqRef;
    }
    else if(lsw==2) {
        pid1_id.term.Ref = IdRef;
        pid1_id.term.RefId = IdRef;
        pid1_id.term.RefIq = IqRef;
    }

    pid1_id.term.Fbk = park1.Ds;
    pid1_id.term.FbkId = park1.Ds;
    pid1_id.term.FbkIq = park1.Qs;

    if(lsw==0) pid1_id.term.We = 0;
    else if(lsw==1) pid1_id.term.We = _IQmpy(_IQmpy(SpeedRef, PI), _IQ(500.0));
    else if(lsw==2) pid1_id.term.We = _IQdiv(_IQmpy(speed1.SpeedRpm, PI), _IQ(6.0));
    //pid1_id.term.t = _IQ(0.0001);
    pid1_id.term.t = T;
    PID_GR_MACRO(pid1_id);

    if(lsw==0) {
        pid1_iq.term.Ref = 0;
        pid1_iq.term.RefId = 0;
        pid1_iq.term.RefIq = 0;
    }
    else if(lsw==1) {
        pid1_iq.term.Ref = IqRef;
        pid1_iq.term.RefId = IdRef;
        pid1_iq.term.RefIq = IqRef;
    }
    else if(lsw==2) {
        pid1_iq.term.Ref = IqRef;
        pid1_iq.term.RefId = IdRef;
        pid1_iq.term.RefIq = IqRef;
    }

    pid1_iq.term.Fbk = park1.Qs;
    pid1_iq.term.FbkId = park1.Ds;
    pid1_iq.term.FbkIq = park1.Qs;

    if(lsw==0) pid1_iq.term.We = 0;
    else if(lsw==1) pid1_iq.term.We = _IQmpy(_IQmpy(SpeedRef, PI), _IQ(500.0));
    else if(lsw==2) pid1_iq.term.We = _IQdiv(_IQmpy(speed1.SpeedRpm, PI), _IQ(6.0));
    //pid1_id.term.t = _IQ(0.0001);
    pid1_iq.term.t = T;
    PID_GR_MACRO(pid1_iq);
// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ipark1.Ds = pid1_id.term.Out;
    ipark1.Qs = pid1_iq.term.Out;
    ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
    IPARK_MACRO(ipark1);
// ------------------------------------------------------------------------------
//  Call the QEP calculation module
// -----------------------------------------------------------------------------
    if (lsw==0) {EQep1Regs.QPOSCNT=0; EQep1Regs.QCLR.bit.IEL = 1;} // Reset position cnt.
    if(lsw!=0)
        QEP_MACRO(qep1);
// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------
    speed1.ElecTheta = qep1.ElecTheta;
    speed1.DirectionQep = (int32)(qep1.DirectionQep);
    SPEED_FR_MACRO(speed1);
    MotorSpeed_RPM = speed1.SpeedRpm;

// ------------------------------------------------------------------------------
//  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
// ------------------------------------------------------------------------------
    volt1.DcBusVolt = ((AdcMirror.ADCRESULT2)*0.00024414); // DC Bus voltage meas.
    DcBusVolt = volt1.DcBusVolt*DcBusVolt_Gain;

    volt1.MfuncV1 = svgen_dq1.Ta;
    volt1.MfuncV2 = svgen_dq1.Tb;
    volt1.MfuncV3 = svgen_dq1.Tc;
    VOLT_MACRO(volt1);

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    svgen_dq1.Ualpha = ipark1.Alpha;
    svgen_dq1.Ubeta = ipark1.Beta;
    SVGEN_MACRO(svgen_dq1);
// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = _IQtoQ15(-svgen_dq1.Ta);
    pwm1.MfuncC2 = _IQtoQ15(-svgen_dq1.Tb);
    pwm1.MfuncC3 = _IQtoQ15(-svgen_dq1.Tc);
    PWM_MACRO(pwm1)                         // Calculate the new PWM compare values

    EPwm1Regs.CMPA.half.CMPA=pwm1.PWM1out;  // PWM 1A - PhaseA
    EPwm2Regs.CMPA.half.CMPA=pwm1.PWM2out;  // PWM 2A - PhaseB
    EPwm3Regs.CMPA.half.CMPA=pwm1.PWM3out;  // PWM 3A - PhaseC
// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    PwmDacCh1 = _IQtoQ15(rg1.Out);
    PwmDacCh2 = _IQtoQ15(speed1.ElecTheta);
    PwmDacCh3 = _IQtoQ15(clarke1.As);
    PwmDacCh4 = _IQtoQ15(clarke1.Bs);
// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(speed1.ElecTheta);
    DlogCh2 = _IQtoQ15(svgen_dq1.Ta);
    DlogCh3 = _IQtoQ15(clarke1.As);
    DlogCh4 = _IQtoQ15(clarke1.Bs);
// ------------------------------------------------------------------------------
//    Call the PWMDAC update macro.
// ------------------------------------------------------------------------------
    PWMDAC_MACRO(pwmdac1);
// ------------------------------------------------------------------------------
//    Call the DATALOG update function.
// ------------------------------------------------------------------------------
    dlog.update(&dlog);

#endif // (BUILDLEVEL==LEVEL4)
}

#pragma CODE_SECTION(PMSM_Motion_LEVEL5,"ramfuncs");
void PMSM_Motion_LEVEL5()
{
#if (BUILDLEVEL==LEVEL5)
    if((IdRef != 0) && (IqRef != 0) && (sc <= 30000)){
        sc=sc+1;
    }
    if((IdRef != 0) && (IqRef != 0) && (sc1 <= 30000)){
        sc1=sc1+1;
    }
    if(lsw == 0){
        for(n=0;n<700;n++){
            a[n]=_IQ(0.0);
            b[n]=_IQ(0.0);
        }
    }if((lsw == 2)&&(IdRef == 0) && (IqRef == 0)){
        n=0;
    }
// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    if(lsw==0)rc1.TargetValue = 0;
    else {
        rc1.TargetValue = SpeedRef_CMD;
        if(rc1.TargetValue > POS_MAX_SPEED) rc1.TargetValue = POS_MAX_SPEED;
        if(rc1.TargetValue < NEG_MAX_SPEED) rc1.TargetValue = NEG_MAX_SPEED;
        }
    RC_MACRO(rc1);
// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1);
// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//  Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
    clarke1.As=((AdcMirror.ADCRESULT0)*0.00024414-cal_offset_A)*2; // Phase A curr.
    clarke1.Bs=((AdcMirror.ADCRESULT1)*0.00024414-cal_offset_B)*2; // Phase B curr.
    CLARKE_MACRO(clarke1);
// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
    park1.Alpha = clarke1.Alpha;
    park1.Beta = clarke1.Beta;
    if(lsw==0) park1.Angle = 0;
    else if(lsw==1) park1.Angle = rg1.Out;
    else if(lsw==2) park1.Angle = speed1.ElecTheta;

    park1.Sine = _IQsinPU(park1.Angle);
    park1.Cosine = _IQcosPU(park1.Angle);
    PARK_MACRO(park1);
// ------------------------------------------------------------------------------
// Current
// ------------------------------------------------------------------------------
    if(Ipeak==_IQ(0.5)){
        IdRef = _IQ(-0.00045);
        IqRef = _IQ(0.0278);
    }
    else if(Ipeak==_IQ(1.0)){
        IdRef = _IQ(-0.0018);
        IqRef = _IQ(0.055);
    }
    else if(Ipeak==_IQ(1.5)){
        IdRef = _IQ(-0.004);
        IqRef = _IQ(0.083);
    }
    else if(Ipeak==_IQ(2.0)){
        IdRef = _IQ(-0.007);
        IqRef = _IQ(0.111);
    }
    else if(Ipeak==_IQ(2.5)){
        IdRef = _IQ(-0.012);
        IqRef = _IQ(0.14);
    }
    else if(Ipeak==_IQ(3.0)){
        IdRef = _IQ(-0.016);
        IqRef = _IQ(0.166);
    }
// ------------------------------------------------------------------------------
//  Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID IQ controller macro
// ------------------------------------------------------------------------------
    if(lsw==0) {
        pid1_id.term.Ref = IdRef;//_IQ(0.035) Origin:_IQ(0.025);
        pid1_id.term.RefId = IdRef;
        pid1_id.term.RefIq = 0;
    }
    else if(lsw==1) {
        pid1_id.term.Ref = IdRef;
        pid1_id.term.RefId = IdRef;
        pid1_id.term.RefIq = IqRef;
    }
    else if(lsw==2) {
        pid1_id.term.Ref = IdRef;
        pid1_id.term.RefId = IdRef;
        pid1_id.term.RefIq = IqRef;
    }

    pid1_id.term.Fbk = park1.Ds;
    pid1_id.term.FbkId = park1.Ds;
    pid1_id.term.FbkIq = park1.Qs;

    if(lsw==0) pid1_id.term.We = 0;
    else if(lsw==1) pid1_id.term.We = _IQmpy(_IQmpy(SpeedRef, PI), _IQ(500.0));
    else if(lsw==2) pid1_id.term.We = _IQdiv(_IQmpy(speed1.SpeedRpm, PI), _IQ(6.0)); //P is pole

    if(lsw==0)pid1_id.term.run = 0;
    else if(lsw==1)pid1_id.term.run = 0;
    else if(lsw==2)pid1_id.term.run = 2;
    //pid1_id.term.t = _IQ(0.0001);
    pid1_id.term.t = T;
    if((IqRef != IqRefprev) || (IdRef != IdRefprev)){
        pid1_id.term.Ichange = 1;
    }else pid1_id.term.Ichange = 0;
    kpUpdateCnt++;
    if(kpUpdateCnt >= kpUpdatePrescaler){
        kpUpdateCnt = 0;
        pid1_id.data.KPdreg = pid1_id.data.KPd;
        pid1_id.data.KPqreg = pid1_id.data.KPq;
        if(!pid1_id.data.KPdlocked){
            if((pid1_id.data.ED <= _IQ(0.002)) && (pid1_id.data.ED >= _IQ(-0.002))){pid1_id.data.KPdlocked = 1;}
            else{pid1_id.data.KPd = pid1_id.data.KPdreg + _IQmpy(_IQdiv(_IQdiv(_IQmpy(pid1_id.data.ED, pid1_id.data.ED), pid1_id.param.Ld), pid1_id.param.betad), pid1_id.term.t);}}
            if(!pid1_id.data.KPqlocked){
            if((pid1_id.data.EQ <= _IQ(0.001)) && (pid1_id.data.EQ >= _IQ(-0.01))){pid1_id.data.KPqlocked = 1;}
            else{pid1_id.data.KPq = pid1_id.data.KPqreg + _IQmpy(_IQdiv(_IQdiv(_IQmpy(pid1_id.data.EQ, pid1_id.data.EQ), pid1_id.param.Lq), pid1_id.param.betaq), pid1_id.term.t);}}
    }
    PID_GR_MACRO(pid1_id);
// && (sc%4 == 0)
    if((IdRef != 0) && (IqRef != 0) && (sc%43 == 0)){
        if(n != 700){
            a[n]=pid1_id.data.IqFbk;
            b[n]=pid1_id.data.IdFbk;
            n++;
            if(sc == 30000)
                n=700;
        }
    }
/*    if((IdRef != 0) && (IqRef != 0)){
        if(sc%8 == 0){
            if(n1 != 2000){
                a[n1]=pid1_id.data.varQ;
            n1++;
            if(sc == 4000)
                n1=2000;
            }
        }
        if(sc1%4 == 0){
            if(n2 != 2000){
                b[n2]=pid1_id.data.varQ;
            n2++;
            if(sc1 == 2000)
                n2=2000;
            }
        }
    }*/

    /*if(lsw==0) {
        pid1_iq.term.Ref = 0;
        pid1_iq.term.RefId = IdRef;//ramper(0.025, pid1_id.term.Ref, _IQ(0.0001));
        pid1_iq.term.RefIq = 0;
    }
    else if(lsw==1) {
        pid1_iq.term.Ref = IqRef;
        pid1_iq.term.RefId = IdRef;
        pid1_iq.term.RefIq = IqRef;
    }
    else if(lsw==2) {
        pid1_iq.term.Ref = IqRef;
        pid1_iq.term.RefId = IdRef;
        pid1_iq.term.RefIq = IqRef;
    }

    pid1_iq.term.Fbk = park1.Qs;
    pid1_iq.term.FbkId = park1.Ds;
    pid1_iq.term.FbkIq = park1.Qs;

    if(lsw==0) pid1_iq.term.We = 0;
    else if(lsw==1) pid1_iq.term.We = _IQmpy(_IQmpy(SpeedRef, PI), _IQ(500.0));
    else if(lsw==2) pid1_iq.term.We = _IQdiv(_IQmpy(speed1.SpeedRpm, PI), _IQ(6.0));
    //pid1_id.term.t = _IQ(0.0001);
    pid1_iq.term.t = T;
    PID_GR_MACRO(pid1_iq)*/
// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    if(s==2){
        ipark1.Ds = VdTesting;
        ipark1.Qs = VqTesting;
    }
    /*else if((sc >= 1000) && (sc <= 1100)){
        ipark1.Ds = pid1_id.term.VdOut + _IQ(0.08);
        ipark1.Qs = pid1_id.term.VqOut + _IQ(0.08);
    }*/
    else{
        ipark1.Ds = pid1_id.term.Vdout;
        ipark1.Qs = pid1_id.term.Vqout;
        //ipark1.Ds = pid1_id.term.Out;
        //ipark1.Qs = pid1_iq.term.Out;
    }
    ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
    IPARK_MACRO(ipark1);

    if(lsw != 0){
        IdRefprev = IdRef;
        IqRefprev = IqRef;
    }
// ------------------------------------------------------------------------------
//    Detect calibration angle and call the QEP module
// ------------------------------------------------------------------------------
    if (lsw==0) {
        EQep1Regs.QPOSCNT = 0;
        EQep1Regs.QCLR.bit.IEL = 1;
    } // Reset position cnt.
    if ((EQep1Regs.QFLG.bit.IEL==1) && (Init_IFlag==0)) {              // Check the index occurrence
        qep1.CalibratedAngle= EQep1Regs.QPOSILAT;
        Init_IFlag++;
    }   // Keep the latched pos. at the first index

    if (lsw!=0)
      QEP_MACRO(qep1);

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------
    speed1.ElecTheta = qep1.ElecTheta;
    speed1.DirectionQep = (int32)(qep1.DirectionQep);
    SPEED_FR_MACRO(speed1);
    MotorSpeed_RPM = speed1.SpeedRpm;
// ------------------------------------------------------------------------------
//  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
// ------------------------------------------------------------------------------
    volt1.DcBusVolt = ((AdcMirror.ADCRESULT2)*0.00024414); // DC Bus voltage meas.
    DcBusVolt = volt1.DcBusVolt*DcBusVolt_Gain;

    volt1.MfuncV1 = svgen_dq1.Ta;
    volt1.MfuncV2 = svgen_dq1.Tb;
    volt1.MfuncV3 = svgen_dq1.Tc;
    VOLT_MACRO(volt1);

// ------------------------------------------------------------------------------
//    Connect inputs of the SMO_POS module and call the sliding-mode observer macro
// ------------------------------------------------------------------------------
    smo1.Ialpha = clarke1.Alpha;
    smo1.Ibeta  = clarke1.Beta;
    smo1.Valpha = volt1.Valpha;
    smo1.Vbeta  = volt1.Vbeta;
    SMO_MACRO(smo1);

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_EST module and call the estimated speed macro
// ------------------------------------------------------------------------------
    speed3.EstimatedTheta = smo1.Theta;
    SE_MACRO(speed3);

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    svgen_dq1.Ualpha = ipark1.Alpha;
    svgen_dq1.Ubeta = ipark1.Beta;
    SVGEN_MACRO(svgen_dq1);
// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = _IQtoQ15(-svgen_dq1.Ta);
    pwm1.MfuncC2 = _IQtoQ15(-svgen_dq1.Tb);
    pwm1.MfuncC3 = _IQtoQ15(-svgen_dq1.Tc);
    PWM_MACRO(pwm1)                         // Calculate the new PWM compare values

    EPwm1Regs.CMPA.half.CMPA=pwm1.PWM1out;  // PWM 1A - PhaseA
    EPwm2Regs.CMPA.half.CMPA=pwm1.PWM2out;  // PWM 2A - PhaseB
    EPwm3Regs.CMPA.half.CMPA=pwm1.PWM3out;  // PWM 3A - PhaseC
// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    PwmDacCh1 = _IQtoQ15(rg1.Out);
    PwmDacCh2 = _IQtoQ15(speed1.ElecTheta);
    PwmDacCh3 = _IQtoQ15(clarke1.As);
    PwmDacCh4 = _IQtoQ15(clarke1.Bs);
// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(AdcMirror.ADCRESULT0);
    DlogCh2 = _IQtoQ15(AdcMirror.ADCRESULT1);
    DlogCh3 = _IQtoQ15(svgen_dq1.Ta);
    DlogCh4 = _IQtoQ15(svgen_dq1.Tb);
/*
    DlogCh1 = _IQtoQ15(smo1.Theta);
    DlogCh2 = _IQtoQ15(rg1.Out);
    DlogCh3 = _IQtoQ15(clarke1.As);
    DlogCh4 = _IQtoQ15(clarke1.Bs);
*/
// ------------------------------------------------------------------------------
//    Call the PWMDAC update macro.
// ------------------------------------------------------------------------------
    PWMDAC_MACRO(pwmdac1);
// ------------------------------------------------------------------------------
//    Call the DATALOG update function.
// ------------------------------------------------------------------------------
    dlog.update(&dlog);

#endif // (BUILDLEVEL==LEVEL5)
}

_iq  cntr=0,
     alignCnt = 20000;
_iq  IdRef_start = _IQ(0.025),
     IdRef_run   = _IQ(0.025);

#pragma CODE_SECTION(PMSM_Motion_LEVEL6,"ramfuncs");
void PMSM_Motion_LEVEL6()
{
#if (BUILDLEVEL==LEVEL6)
// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    if(lsw==0)rc1.TargetValue = 0;
    else {
        rc1.TargetValue = SpeedRef_CMD;
        if(rc1.TargetValue > POS_MAX_SPEED) rc1.TargetValue = POS_MAX_SPEED;
        if(rc1.TargetValue < NEG_MAX_SPEED) rc1.TargetValue = NEG_MAX_SPEED;
        }
    RC_MACRO(rc1);
// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1);
// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//  Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
    clarke1.As=((AdcMirror.ADCRESULT0)*0.00024414-cal_offset_A)*2; // Phase A curr.
    clarke1.Bs=((AdcMirror.ADCRESULT1)*0.00024414-cal_offset_B)*2; // Phase B curr.
    CLARKE_MACRO(clarke1);
// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
    park1.Alpha = clarke1.Alpha;
    park1.Beta = clarke1.Beta;

    if(lsw==0) park1.Angle = 0;
    else if(lsw==1) park1.Angle = rg1.Out;
    else if(lsw==2) park1.Angle = rg1.Out;//rg1.Out-->positive/Reversing
    else if(lsw==3) park1.Angle = speed1.ElecTheta;

    park1.Sine = _IQsinPU(park1.Angle);
    park1.Cosine = _IQcosPU(park1.Angle);

    PARK_MACRO(park1);
// ------------------------------------------------------------------------------
//    Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID speed controller macro
// ------------------------------------------------------------------------------
  if (SpeedLoopCount==SpeedLoopPrescaler)
     {
      pid1_spd.term.Ref = rc1.SetpointValue;
      pid1_spd.term.Fbk = speed1.Speed;
      PID_GR_MACRO(pid1_spd);
      SpeedLoopCount=1;
     }
  else SpeedLoopCount++;

  if(lsw==0 || lsw==1)
  {
    pid1_spd.data.ui=0;
    pid1_spd.data.i1=0;
  }
// ------------------------------------------------------------------------------
//    Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID IQ controller macro
// ------------------------------------------------------------------------------
    if(lsw==0) pid1_iq.term.Ref = 0;
    else if(lsw==1) pid1_iq.term.Ref = IqRef;
    else pid1_iq.term.Ref = pid1_spd.term.Out; //(pid1_spd.term.Out->Level 6A, IqRef->Level 6B)
    pid1_iq.term.Fbk = park1.Qs;
    PID_GR_MACRO(pid1_iq);

// ------------------------------------------------------------------------------
//    Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID ID controller macro
// ------------------------------------------------------------------------------
    if(lsw==0) pid1_id.term.Ref = ramper(0.025, pid1_id.term.Ref, _IQ(0.0001));//_IQ(0.025);
    else pid1_id.term.Ref = ramper(0, pid1_id.term.Ref, _IQ(0.0001));//IdRef;

    pid1_id.term.Fbk = park1.Ds;
    PID_GR_MACRO(pid1_id);

// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ipark1.Ds = pid1_id.term.Out;
    ipark1.Qs = pid1_iq.term.Out;
    ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
    IPARK_MACRO(ipark1);

// ------------------------------------------------------------------------------
//    Detect calibration angle and call the QEP module
// ------------------------------------------------------------------------------
    if (lsw==0) {EQep1Regs.QPOSCNT=0; EQep1Regs.QCLR.bit.IEL = 1;} // Reset position cnt.
    if ((EQep1Regs.QFLG.bit.IEL==1) && Init_IFlag==0)              // Check the index occurrence
         {qep1.CalibratedAngle= EQep1Regs.QPOSILAT; Init_IFlag++;} // Keep the latched pos. at the first index

    if (lsw!=0)
      QEP_MACRO(qep1);

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------
    speed1.ElecTheta = qep1.ElecTheta;
    speed1.DirectionQep = (int32)(qep1.DirectionQep);
    SPEED_FR_MACRO(speed1);
    MotorSpeed_RPM = speed1.SpeedRpm;
// ------------------------------------------------------------------------------
//    Connect inputs of the VOLT_CALC module and call the phase voltage macro
// ------------------------------------------------------------------------------
    volt1.DcBusVolt = ((AdcMirror.ADCRESULT2)*0.00024414); // DC Bus voltage meas.
    DcBusVolt = volt1.DcBusVolt*DcBusVolt_Gain;

    volt1.MfuncV1 = svgen_dq1.Ta;
    volt1.MfuncV2 = svgen_dq1.Tb;
    volt1.MfuncV3 = svgen_dq1.Tc;
    VOLT_MACRO(volt1);
// ------------------------------------------------------------------------------
//    Connect inputs of the SMO_POS module and call the sliding-mode observer macro
// ------------------------------------------------------------------------------
    smo1.Ialpha = clarke1.Alpha;
    smo1.Ibeta  = clarke1.Beta;
    smo1.Valpha = volt1.Valpha;
    smo1.Vbeta  = volt1.Vbeta;
    SMO_MACRO(smo1);
// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_EST module and call the estimated speed macro
// ------------------------------------------------------------------------------
    speed3.EstimatedTheta = smo1.Theta;
    SE_MACRO(speed3);
// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    svgen_dq1.Ualpha = ipark1.Alpha;
    svgen_dq1.Ubeta = ipark1.Beta;
    SVGEN_MACRO(svgen_dq1);
// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = _IQtoQ15(-svgen_dq1.Ta);
    pwm1.MfuncC2 = _IQtoQ15(-svgen_dq1.Tb);
    pwm1.MfuncC3 = _IQtoQ15(-svgen_dq1.Tc);
    PWM_MACRO(pwm1);                       // Calculate the new PWM compare values

    EPwm1Regs.CMPA.half.CMPA=pwm1.PWM1out;  // PWM 1A - PhaseA
    EPwm2Regs.CMPA.half.CMPA=pwm1.PWM2out;  // PWM 2A - PhaseB
    EPwm3Regs.CMPA.half.CMPA=pwm1.PWM3out;  // PWM 3A - PhaseC

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    PwmDacCh1 = _IQtoQ15(rg1.Out);
    PwmDacCh2 = _IQtoQ15(speed1.ElecTheta);
    PwmDacCh3 = _IQtoQ15(clarke1.As);
    PwmDacCh4 = _IQtoQ15(clarke1.Bs);

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(rg1.Out);
    DlogCh2 = _IQtoQ15(speed1.ElecTheta);
    DlogCh3 = _IQtoQ15(clarke1.As);
    DlogCh4 = _IQtoQ15(clarke1.Bs);
// ------------------------------------------------------------------------------
//    Call the PWMDAC update macro.
// ------------------------------------------------------------------------------
    PWMDAC_MACRO(pwmdac1);
// ------------------------------------------------------------------------------
//    Call the DATALOG update function.
// ------------------------------------------------------------------------------
    dlog.update(&dlog);

#endif // (BUILDLEVEL==LEVEL6)
}

//=================================================================================
//  CPU_TIMER2_ISR - TASKS (executed in every 500 msec)
//=================================================================================
__interrupt void CPU_TIMER2_ISR_TASK(void)
{
    LD2_TOGGLE();
    if(lsw>=1){LD3_TOGGLE();}

    if (EnableFlag == FALSE)
    {
        Motor_DRV_Disable();
        RunMotor = FALSE;
        EALLOW;
        EPwm1Regs.TZFRC.bit.OST=1;
        EPwm2Regs.TZFRC.bit.OST=1;
        EPwm3Regs.TZFRC.bit.OST=1;

        EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
        EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low
        EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
        EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low
        EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
        EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low
        EDIS;
        EPwm1Regs.CMPA.half.CMPA=0;  // PWM 1A - PhaseA
        EPwm2Regs.CMPA.half.CMPA=0;  // PWM 2A - PhaseB
        EPwm3Regs.CMPA.half.CMPA=0;  // PWM 3A - PhaseC
    }
    else if((EnableFlag == TRUE) && (RunMotor == FALSE))
    {
        if(DRV_RESET == 0)
        {
            Motor_DRV_Enable();
        }
        speed3.EstimatedSpeed=0;
        speed3.EstimatedTheta=0;
        speed3.OldEstimatedTheta=0;
        speed3.EstimatedSpeedRpm=0;
        rg1.Freq=0;
        rg1.Out=0;
        rg1.Angle=0;
        rc1.TargetValue=0;
        rc1.SetpointValue=0;
        smo1.Theta=0;
        smo1.Ealpha=0;
        smo1.Ebeta=0;
        pid1_id.data.d1 = 0;
        pid1_id.data.d2 = 0;
        pid1_id.data.i1 = 0;
        pid1_id.data.ud = 0;
        pid1_id.data.ui = 0;
        pid1_id.data.up = 0;
        pid1_id.data.v1 = 0;
        pid1_id.data.w1 = 0;
        pid1_id.term.Out = 0;
        pid1_id.data.Id = 0;
        pid1_id.data.Iq = 0;
        pid1_id.data.IdFbk = 0;
        pid1_id.data.IqFbk = 0;
        pid1_id.data.Idfbk = 0;
        pid1_id.data.Iqfbk = 0;
        pid1_id.data.ED = 0;
        pid1_id.data.EQ = 0;
        pid1_id.data.Ed = 0;
        pid1_id.data.Eq = 0;
        pid1_id.data.Sgnd = 0;
        pid1_id.data.Sgnq = 0;
        pid1_id.data.SigmaD = 0;
        pid1_id.data.SigmaQ = 0;
        pid1_id.data.Sigmad = 0;
        pid1_id.data.Sigmaq = 0;
        pid1_id.data.varD = 0;
        pid1_id.data.varQ = 0;
        pid1_id.data.varD1 = 0;
        pid1_id.data.varQ1 = 0;
        pid1_id.data.Vd = 0;
        pid1_id.data.Vd1 = 0;
        pid1_id.data.Vd2 = 0;
        pid1_id.data.Vd3 = 0;
        pid1_id.data.Vd4 = 0;
        pid1_id.data.Vq = 0;
        pid1_id.data.Vq1 = 0;
        pid1_id.data.Vq2 = 0;
        pid1_id.data.Vq3 = 0;
        pid1_id.data.Vq4 = 0;
        pid1_id.data.Vq5 = 0;
        pid1_id.data.vd = 0;
        pid1_id.data.vq = 0;
        pid1_id.data.KPd = 0;
        pid1_id.data.KPq = 0;
        pid1_id.data.KPdreg = 0;
        pid1_id.data.KPqreg = 0;
        pid1_id.data.deltad = 0;
        pid1_id.data.deltaq = 0;
        pid1_id.data.deltadreg = 0;
        pid1_id.data.deltaqreg = 0;
        pid1_id.data.Iqreg = 0;
        pid1_id.data.Idreg = 0;
        pid1_id.data.KPdlocked = 0;
        pid1_id.data.KPqlocked = 0;
        pid1_id.term.VdOut = 0;
        pid1_id.term.VqOut = 0;
        pid1_id.term.Vdout = 0;
        pid1_id.term.Vqout = 0;
        pid1_id.term.run = 0;

        pid1_iq.data.d1 = 0;
        pid1_iq.data.d2 = 0;
        pid1_iq.data.i1 = 0;
        pid1_iq.data.ud = 0;
        pid1_iq.data.ui = 0;
        pid1_iq.data.up = 0;
        pid1_iq.data.v1 = 0;
        pid1_iq.data.w1 = 0;
        pid1_iq.term.Out = 0;
        pid1_iq.data.Id = 0;
        pid1_iq.data.Iq = 0;
        pid1_iq.data.IdFbk = 0;
        pid1_iq.data.IqFbk = 0;
        pid1_iq.data.Idfbk = 0;
        pid1_iq.data.Iqfbk = 0;
        pid1_iq.data.ED = 0;
        pid1_iq.data.EQ = 0;
        pid1_iq.data.Ed = 0;
        pid1_iq.data.Eq = 0;
        pid1_iq.data.Sgnd = 0;
        pid1_iq.data.Sgnq = 0;
        pid1_iq.data.SigmaD = 0;
        pid1_iq.data.SigmaQ = 0;
        pid1_iq.data.Sigmad = 0;
        pid1_iq.data.Sigmaq = 0;
        pid1_iq.data.varD = 0;
        pid1_iq.data.varQ = 0;
        pid1_iq.data.varD1 = 0;
        pid1_iq.data.varQ1 = 0;
        pid1_iq.data.Vd = 0;
        pid1_iq.data.Vd1 = 0;
        pid1_iq.data.Vd2 = 0;
        pid1_iq.data.Vd3 = 0;
        pid1_iq.data.Vd4 = 0;
        pid1_iq.data.Vq = 0;
        pid1_iq.data.Vq1 = 0;
        pid1_iq.data.Vq2 = 0;
        pid1_iq.data.Vq3 = 0;
        pid1_iq.data.Vq4 = 0;
        pid1_iq.data.Vq5 = 0;
        pid1_iq.data.vd = 0;
        pid1_iq.data.vq = 0;
        pid1_iq.data.KPd = 0;
        pid1_iq.data.KPq = 0;
        pid1_iq.data.KPdreg = 0;
        pid1_iq.data.KPqreg = 0;
        pid1_iq.data.deltad = 0;
        pid1_iq.data.deltaq = 0;
        pid1_iq.data.deltadreg = 0;
        pid1_iq.data.deltaqreg = 0;
        pid1_iq.data.Iqreg = 0;
        pid1_iq.data.Idreg = 0;
        pid1_iq.data.KPdlocked = 0;
        pid1_iq.data.KPqlocked = 0;
        pid1_iq.term.VdOut = 0;
        pid1_iq.term.VqOut = 0;
        pid1_iq.term.Vdout = 0;
        pid1_iq.term.Vqout = 0;
        pid1_iq.term.run = 0;


        pid1_spd.data.d1 = 0;
        pid1_spd.data.d2 = 0;
        pid1_spd.data.i1 = 0;
        pid1_spd.data.ud = 0;
        pid1_spd.data.ui = 0;
        pid1_spd.data.up = 0;
        pid1_spd.data.v1 = 0;
        pid1_spd.data.w1 = 0;
        pid1_spd.term.Out = 0;
        pid1_spd.data.Id = 0;
        pid1_spd.data.Iq = 0;
        pid1_spd.data.IdFbk = 0;
        pid1_spd.data.IqFbk = 0;
        pid1_spd.data.Idfbk = 0;
        pid1_spd.data.Iqfbk = 0;
        pid1_spd.data.ED = 0;
        pid1_spd.data.EQ = 0;
        pid1_spd.data.Ed = 0;
        pid1_spd.data.Eq = 0;
        pid1_spd.data.Sgnd = 0;
        pid1_spd.data.Sgnq = 0;
        pid1_spd.data.SigmaD = 0;
        pid1_spd.data.SigmaQ = 0;
        pid1_spd.data.Sigmad = 0;
        pid1_spd.data.Sigmaq = 0;
        pid1_spd.data.varD = 0;
        pid1_spd.data.varQ = 0;
        pid1_spd.data.varD1 = 0;
        pid1_spd.data.varQ1 = 0;
        pid1_spd.data.Vd = 0;
        pid1_spd.data.Vd1 = 0;
        pid1_spd.data.Vd2 = 0;
        pid1_spd.data.Vd3 = 0;
        pid1_spd.data.Vd4 = 0;
        pid1_spd.data.Vq = 0;
        pid1_spd.data.Vq1 = 0;
        pid1_spd.data.Vq2 = 0;
        pid1_spd.data.Vq3 = 0;
        pid1_spd.data.Vq4 = 0;
        pid1_spd.data.Vq5 = 0;
        pid1_spd.data.vd = 0;
        pid1_spd.data.vq = 0;
        pid1_spd.data.KPd = 0;
        pid1_spd.data.KPq = 0;
        pid1_spd.data.KPdreg = 0;
        pid1_spd.data.KPqreg = 0;
        pid1_spd.data.deltad = 0;
        pid1_spd.data.deltaq = 0;
        pid1_spd.data.deltadreg = 0;
        pid1_spd.data.deltaqreg = 0;
        pid1_spd.data.Iqreg = 0;
        pid1_spd.data.Idreg = 0;
        pid1_spd.data.KPdlocked = 0;
        pid1_spd.data.KPqlocked = 0;
        pid1_spd.term.VdOut = 0;
        pid1_spd.term.VqOut = 0;
        pid1_spd.term.Vdout = 0;
        pid1_spd.term.Vqout = 0;
        pid1_spd.term.run = 0;

        lsw=0;
        RunMotor = TRUE;
        EALLOW;
        EPwm1Regs.TZCLR.bit.OST=1;
        EPwm2Regs.TZCLR.bit.OST=1;
        EPwm3Regs.TZCLR.bit.OST=1;
        EDIS;

        SpeedRef_CMD =_IQ(0.05);
    }

    if((EnableFlag == TRUE) && (RunMotor == TRUE))//Start Run
    {
        SpeedRef_CMD = ramper(_IQ(SpeedRef),SpeedRef_CMD, _IQ(0.02));
    }
}

// ------------------------------------------------------------------------------
//    slew programmable ramper
// ------------------------------------------------------------------------------
_iq ramper(_iq in, _iq out, _iq rampDelta)
{
    _iq err;
    err = in - out;
    if (err > rampDelta)
        return(out + rampDelta);
    else if (err < -rampDelta)
        return(out - rampDelta);
    else
        return(in);
}
//===========================================================================
// No more.
//===========================================================================
