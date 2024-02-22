//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define RWh         0.1946
#define WR          0.56759
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);
__interrupt void ADCD_ISR (void);
__interrupt void ADCA_ISR (void);
__interrupt void ADCB_ISR (void);

void setupSPIB(void);
void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);

uint16_t adcd0result = 0;
uint16_t adcd1result = 0;
uint16_t adca0result = 0;
uint16_t adca1result = 0;
uint16_t adcb0result = 0;
float adcind0scaled = 0.0;
float adcind1scaled = 0.0;
float adcinb0scaled = 0.0;
int32_t adcdcount = 0;
int32_t adcbcount = 0;

//xk is the current ADC reading, xk_1 is the ADC reading one millisecond ago, xk_2 two milliseconds ago, etc
float xk = 0;
float xn = 0;
float xk_1 = 0;
float xk_2 = 0;
float xk_3 = 0;
float xk_4 = 0;
//yk is the filtered value
float yk = 0;
float yn = 0;
//b is the filter coefficients
#define ARRAYSIZE 101
float b[ARRAYSIZE] ={2.0089131384901197e-03,
                     6.4032873499578040e-04,
                     -1.7662310132503288e-03,
                     -1.8966231855838251e-03,
                     7.9038298787438197e-04,
                     2.8250866960543826e-03,
                     9.7274726769560108e-04,
                     -2.8535932093218977e-03,
                     -3.2069079180517828e-03,
                     1.3777460739364028e-03,
                     5.0108857805228734e-03,
                     1.7369488778204004e-03,
                     -5.0869489066624630e-03,
                     -5.6717260737981379e-03,
                     2.4066077632725297e-03,
                     8.6179538038498871e-03,
                     2.9352017836365030e-03,
                     -8.4357135384937401e-03,
                     -9.2235281203421979e-03,
                     3.8369713729420702e-03,
                     1.3470983718227284e-02,
                     4.4992711557421761e-03,
                     -1.2684979985041140e-02,
                     -1.3611937750688167e-02,
                     5.5600514925787251e-03,
                     1.9176967391055018e-02,
                     6.2956283333650978e-03,
                     -1.7455271677881148e-02,
                     -1.8429536833842467e-02,
                     7.4103785848253561e-03,
                     2.5171457314971404e-02,
                     8.1418571044648731e-03,
                     -2.2250769713411937e-02,
                     -2.3165078063428872e-02,
                     9.1879041586407240e-03,
                     3.0795414085640505e-02,
                     9.8318928762857697e-03,
                     -2.6528873794684965e-02,
                     -2.7276081156801475e-02,
                     1.0686709091186523e-02,
                     3.5390668308456406e-02,
                     1.1166118673320274e-02,
                     -2.9780034614308684e-02,
                     -3.0269173855075916e-02,
                     1.1725680290077527e-02,
                     3.8398491060813049e-02,
                     1.1981403290429368e-02,
                     -3.1604759414221834e-02,
                     -3.1774940699058361e-02,
                     1.2176082500102338e-02,
                     3.9444917234878515e-02,
                     1.2176082500102338e-02,
                     -3.1774940699058361e-02,
                     -3.1604759414221834e-02,
                     1.1981403290429368e-02,
                     3.8398491060813049e-02,
                     1.1725680290077527e-02,
                     -3.0269173855075916e-02,
                     -2.9780034614308684e-02,
                     1.1166118673320274e-02,
                     3.5390668308456406e-02,
                     1.0686709091186523e-02,
                     -2.7276081156801475e-02,
                     -2.6528873794684965e-02,
                     9.8318928762857697e-03,
                     3.0795414085640505e-02,
                     9.1879041586407240e-03,
                     -2.3165078063428872e-02,
                     -2.2250769713411937e-02,
                     8.1418571044648731e-03,
                     2.5171457314971404e-02,
                     7.4103785848253561e-03,
                     -1.8429536833842467e-02,
                     -1.7455271677881148e-02,
                     6.2956283333650978e-03,
                     1.9176967391055018e-02,
                     5.5600514925787251e-03,
                     -1.3611937750688167e-02,
                     -1.2684979985041140e-02,
                     4.4992711557421761e-03,
                     1.3470983718227284e-02,
                     3.8369713729420702e-03,
                     -9.2235281203421979e-03,
                     -8.4357135384937401e-03,
                     2.9352017836365030e-03,
                     8.6179538038498871e-03,
                     2.4066077632725297e-03,
                     -5.6717260737981379e-03,
                     -5.0869489066624630e-03,
                     1.7369488778204004e-03,
                     5.0108857805228734e-03,
                     1.3777460739364028e-03,
                     -3.2069079180517828e-03,
                     -2.8535932093218977e-03,
                     9.7274726769560108e-04,
                     2.8250866960543826e-03,
                     7.9038298787438197e-04,
                     -1.8966231855838251e-03,
                     -1.7662310132503288e-03,
                     6.4032873499578040e-04,
                     2.0089131384901197e-03};; // 0.2 is 1/5th therefore a 5 point average
float past_states[ARRAYSIZE];
float adcach0_past[ARRAYSIZE];
float adcach1_past[ARRAYSIZE];

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numTimer1calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t spivalue3 = 0;
int16_t pwm_value = 0;
int16_t pwm_increment = 0;
float adcvalue1 = 0.0;
float adcvalue2 = 0.0;
float adcvalue3 = 0.0;
int16_t gyroXraw = 0;
int16_t gyroYraw = 0;
int16_t gyroZraw = 0;
int16_t accelXraw = 0;
int16_t accelYraw = 0;
int16_t accelZraw = 0;
float gyrox = 0;
float gyro_value_1 = 0;
float gyrorate_dot = 0;
float gyrorate_dot_1 = 0;
float gyroy = 0;
float gyroz = 0;
float accelx = 0;
float accely = 0;
float accelz = 0;

float leftWheel = 0.0;
float rightWheel = 0.0;

float radtoft = 5.4;

float leftDist = 0.0;
float rightDist = 0.0;
float leftDist_1 = 0;
float rightDist_1 = 0;

float vLeft = 0;
float vRight = 0;

float uLeft = 0.0;
float uRight = 0.0;
float ubal = 0.0;

float Vref = 0;
float Kp = 3.0;
float Ki = 20.0;
float Kd = 0.08;
float errorLeft = 0.0;
float errorRight = 0.0;
float error_1Left = 0.0;
float error_1Right = 0.0;
float IkLeft = 0.0;
float IkRight = 0.0;
float Ik_1Left = 0.0;
float Ik_1Right = 0.0;

float turn = 0.0;
float errorTurn = 0.0;
float KpTurn = 3.0;

float printLV3 = 0;
float printLV4 = 0;
float printLV5 = 0;
float printLV6 = 0;
float printLV7 = 0;
float printLV8 = 0;
extern uint16_t NewLVData;
extern float fromLVvalues[LVNUM_TOFROM_FLOATS];
extern LVSendFloats_t DataToLabView;
extern char LVsenddata[LVNUM_TOFROM_FLOATS*4+2];
extern uint16_t newLinuxCommands;
extern float LinuxCommands[CMDNUM_FROM_FLOATS];

float thetaL = 0.0;
float thetaR = 0.0;
float thetaL_1 = 0.0;
float thetaR_1 = 0.0;
float thetadotL = 0.0;
float thetadotR = 0.0;
float phiR = 0.0;
float xR = 0.0;
float yR = 0.0;
float xR_1 = 0.0;
float yR_1 = 0.0;
float xdotR_1 = 0.0;
float ydotR_1 = 0.0;
float xdotR = 0.0;
float ydotR = 0.0;
float thetaAvg = 0.0;
float thetadotAvg = 0.0;

// Needed global Variables
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset = 0;
float gyroy_offset = 0;
float gyroz_offset = 0;
float accelzBalancePoint = -.62; //-.76
int16 IMU_data[9];
uint16_t temp=0;
int16_t doneCal = 0;
float tilt_value = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value = 0;
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheel = 0;
float RightWheel = 0;
float WhlDiff = 0;
float WhlDiff_1 = 0;
float vel_WhlDiff_1 = 0;
float vel_WhlDiff = 0;
float turnref = 0;
float errorDiff = 0;
float errorDiff_1 = 0;
float intDiff = 0;
float intDiff_1 = 0;
float uDiff = 0;
float turnrate = 0;
float turnrate_1 = 0;
float turnref_1 = 0;
float LeftWheel_1 = 0;
float RightWheel_1 = 0;
float LeftWheelSpeed = 0;
float RightWheelSpeed = 0;
float LeftWheelSpeed_1 = 0;
float RightWheelSpeed_1 = 0;
float Segbot_refSpeed = 0;
float eSpeed = 0;
float eSpeed_1 = 0;
float IK_eSpeed = 0;
float IK_eSpeed_1 = 0;
float KpSpeed = 0.35;
float KiSpeed = 1.5;
float ForwardBackwardCommand = 0;
float LeftWheelArray[4] = {0,0,0,0};
float RightWheelArray[4] = {0,0,0,0};
// Kalman Filter vars
float T = 0.001; //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000;//50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t SpibNumCalls = -1;
float pred_P = 0;
float kalman_K = 0;
int32_t timecount = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;

//This function sets DACA to the voltage between 0V and 3V passed to this function.
//If outside 0V to 3V the output is saturated at 0V to 3V
//Example code
//float myu = 2.25;
//setDACA(myu); // DACA will now output 2.25 Volts
void setDACA(float dacouta0) {
    int16_t DACOutInt = 0;
    DACOutInt = dacouta0*4096.0/3.0; // perform scaling of 0 – almost 3V to 0 - 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;
    DacaRegs.DACVALS.bit.DACVALS = DACOutInt;
}

void setDACB(float dacouta1) {
    int16_t DACOutInt = 0;
    DACOutInt = dacouta1*4096.0/3.0; // perform scaling of 0 – almost 3V to 0 - 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;
    DacbRegs.DACVALS.bit.DACVALS = DACOutInt;
}

//adcd1 pie interrupt
// VQM This interrupt filters the ADC output and calls our previous setDAC function to convert the value back to an analog signal
__interrupt void ADCD_ISR (void) {
    adcd0result = AdcdResultRegs.ADCRESULT0; // VQM save the ADC output value
    adcd1result = AdcdResultRegs.ADCRESULT1;
    adcind0scaled = adcd0result*3.0/4096.0;// Here covert ADCIND0 to volts
    adcind1scaled = adcd1result*3.0/4096.0;// Here covert ADCIND0 to volts
    xk = adcind0scaled;
    yk = 0;
    //    yk = b[0]*xk + b[1]*xk_1 + b[2]*xk_2 + b[3]*xk_3 + b[4]*xk_4;
    //
    //    //Save past states before exiting from the function so that next sample they are the older state
    //    xk_4 = xk_3;
    //    xk_3 = xk_2;
    //    xk_2 = xk_1;
    //    xk_1 = xk;


    // VQM This allows for the size of the filter to be variable, by updating previous values iteratively
    for (int i = ARRAYSIZE - 1; i > 0; i--) {
        past_states[i] = past_states[i - 1];
        past_states[0] = xk;
    }
    for (int i = 0; i < ARRAYSIZE; i++) {
        yk += past_states[i]*b[i];
    }

    setDACA(yk);// Here write voltages value to DACA

    // Print ADCIND0’s voltage value to TeraTerm every 100ms
    adcdcount++;
    if (adcdcount % 100 == 0) {
        UARTPrint = 1;
    }
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//adca1 pie interrupt
// VQM same interrupt as above, just using both channels instead of only one
__interrupt void ADCA_ISR (void) {
    adca0result = AdcaResultRegs.ADCRESULT0;
    adca1result = AdcaResultRegs.ADCRESULT1;
    adcind0scaled = adca0result*3.0/4096.0;// Here covert ADCIND0 to volts
    adcind1scaled = adca1result*3.0/4096.0;// Here covert ADCIND0 to volts
    xk = adcind0scaled;
    xn = adcind1scaled;
    yk = 0;
    yn = 0;
    //    yk = b[0]*xk + b[1]*xk_1 + b[2]*xk_2 + b[3]*xk_3 + b[4]*xk_4;
    //
    //    //Save past states before exiting from the function so that next sample they are the older state
    //    xk_4 = xk_3;
    //    xk_3 = xk_2;
    //    xk_2 = xk_1;
    //    xk_1 = xk;

    for (int i = ARRAYSIZE - 1; i > 0; i--) {
        adcach0_past[i] = adcach0_past[i - 1];
        adcach0_past[0] = xk;
    }
    for (int i = 0; i < ARRAYSIZE; i++) {
        yk += adcach0_past[i]*b[i];
    }

    for (int i = ARRAYSIZE - 1; i > 0; i--) {
        adcach1_past[i] = adcach1_past[i - 1];
        adcach1_past[0] = xn;
    }
    for (int i = 0; i < ARRAYSIZE; i++) {
        yn += adcach1_past[i]*b[i];
    }

    // VQM Exercise 3 MPU values
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Need to select MPU chip with GPIO 66
    SpibRegs.SPIFFRX.bit.RXFFIL = 8; // Issue the SPIB_RX_INT when two values are in the RX FIFO
    SpibRegs.SPITXBUF = 0xBA00; // VQM Starting at INT_STATUS so all low and high values of accel and gyro are together
    SpibRegs.SPITXBUF = 0x0000; // VQM transmit zeros to receive AccelX high and low
    SpibRegs.SPITXBUF = 0x0000; // VQM transmit zeros to receive AccelY high and low
    SpibRegs.SPITXBUF = 0x0000; // VQM transmit zeros to receive AccelZ high and low
    SpibRegs.SPITXBUF = 0x0000; // VQM transmit zeros to receive Temp high and low
    SpibRegs.SPITXBUF = 0x0000; // VQM transmit zeros to receive gyroX high and low
    SpibRegs.SPITXBUF = 0x0000; // VQM transmit zeros to receive gyroY high and low
    SpibRegs.SPITXBUF = 0x0000; // VQM transmit zeros to receive gyroZ high and low

    setDACA(yk);// Here write voltages value to DACA
    setDACB(yn);

    // Print ADCIND0’s voltage value to TeraTerm every 100ms
    adcdcount++;
    if (adcdcount % 200 == 0) {
        UARTPrint = 1;
    }
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//adcb1 pie interrupt
// VQM This interrupt is the same, but also uses the GPIO52 pin to measure the time it takes for the sound filter to run
__interrupt void ADCB_ISR (void) {
    GpioDataRegs.GPBSET.bit.GPIO52 = 1;
    adcb0result = AdcbResultRegs.ADCRESULT0;
    adcinb0scaled = adcb0result*3.0/4096.0;// Here covert ADCIND0 to volts
    xk = adcinb0scaled;
    yk = 0;

    for (int i = ARRAYSIZE - 1; i > 0; i--) {
        past_states[i] = past_states[i - 1];
        past_states[0] = xk;
    }
    for (int i = 0; i < ARRAYSIZE; i++) {
        yk += past_states[i]*b[i];
    }

    setDACA(yk+1.5);// Here write voltages value to DACA

    // Print ADCIND0’s voltage value to TeraTerm every 100ms
    adcbcount++;
    if (adcbcount % 100 == 0) {
        UARTPrint = 1;
    }
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
}

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // VQM EPWM2A
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO2 = 1;

    // VQM EPWM2B
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO3 = 1;


    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.ADCD1_INT = &ADCD_ISR; // VQM added interrupt functions for the ADCs
    PieVectTable.ADCA1_INT = &ADCA_ISR;
    PieVectTable.ADCB1_INT = &ADCB_ISR;

    PieVectTable.SPIB_RX_INT = &SPIB_isr;
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 4000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 10000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);

    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; // For EPWM2B
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”)
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;

    // VQM EPWM2A settings
    EPwm2Regs.TBCTL.bit.CTRMODE = 0; // Changed from default to Up-Count
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2; // Changed from default to Free Run
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; // Default
    EPwm2Regs.TBCTL.bit.PHSEN = 0; // Default
    EPwm2Regs.TBCTR = 0; // Start timer at 0
    EPwm2Regs.TBPRD = 2500; // Measure period of 1/20000 s
    EPwm2Regs.CMPA.bit.CMPA = 0; // Duty cycle percentage = CMPA/TBPRD
    EPwm2Regs.AQCTLA.bit.CAU = 1; // Fall after reaching CMPA
    EPwm2Regs.AQCTLA.bit.ZRO = 2; // Rise when counter drops to zero
    EPwm2Regs.TBPHS.bit.TBPHS = 0; // Set phase to zero

    // VQM EPWM2B settings
    EPwm2Regs.AQCTLB.bit.CBU = 1; // Fall after reaching CMPB
    EPwm2Regs.AQCTLB.bit.ZRO = 2; // Rise when counter drops to zero
    EPwm2Regs.CMPB.bit.CMPB = 0; // Duty cycle percentage = CMPB/TBPRD

    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag
    //Many statements commented out, To be used when using ADCA or ADCB
    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0x2; //SOC0 will convert Channel you choose Does not have to be A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0D;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0x3; //SOC1 will convert Channel you choose Does not have to be A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0x0D;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0x1; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0x4; //SOC0 will convert Channel you choose Does not have to be B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0D; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    //  AdcbRegs.ADCSOC1CTL.bit.CHSEL = ???; //SOC1 will convert Channel you choose Does not have to be B1
    //  AdcbRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //  AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    //  AdcbRegs.ADCSOC2CTL.bit.CHSEL = ???; //SOC2 will convert Channel you choose Does not have to be B2
    //  AdcbRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //  AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC2
    //  AdcbRegs.ADCSOC3CTL.bit.CHSEL = ???; //SOC3 will convert Channel you choose Does not have to be B3
    //  AdcbRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //  AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC3
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0x0; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //ADCD
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0x0; // set SOC0 to convert pin D0
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0D; // EPWM5 ADCSOCA will trigger SOC0
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 0x1; //set SOC1 to convert pin D1
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 0x0D; // EPWM5 ADCSOCA will trigger SOC1
    //AdcdRegs.ADCSOC2CTL.bit.CHSEL = ???; //set SOC2 to convert pin D2
    //AdcdRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC2
    //AdcdRegs.ADCSOC3CTL.bit.CHSEL = ???; //set SOC3 to convert pin D3
    //AdcdRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC3
    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 0x1; //set to SOC1, the last converted, and it will set INT1 flag ADCD1
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;

    setupSPIB();

    init_eQEPs();

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6; // VQM SPIB

    // Enable TINT0 in the PIE: Group 1 interrupt 1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    // Enable TINT0 in the PIE: Group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    init_serialSCIB(&SerialB,115200);
    init_serialSCIC(&SerialC,115200);
    init_serialSCID(&SerialD,115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA,"Tilt Value: .3f Gyro Value: \r\n", tilt_value, gyro_value, LeftWheel, RightWheel);
            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......
    LeftWheel = readEncLeft();
    RightWheel = readEncRight();
    LeftWheelSpeed = 100*(LeftWheel-LeftWheel_1) + 0.6*LeftWheelSpeed_1;
    RightWheelSpeed = 100*(RightWheel-RightWheel_1) + 0.6*RightWheelSpeed_1;

    LeftWheel_1 = LeftWheel;
    RightWheel_1 = RightWheel;
    LeftWheelSpeed_1 = LeftWheelSpeed;
    RightWheelSpeed_1 = RightWheelSpeed;

    turnref = turnref_1 + 0.004*(turnrate + turnrate_1)/2;
    WhlDiff = LeftWheel - RightWheel;
    vel_WhlDiff = 0.3333*vel_WhlDiff_1 + 166.667*WhlDiff - 166.667*WhlDiff_1;
    errorDiff = turnref - WhlDiff;
    intDiff = intDiff_1 + 0.004*(errorDiff + errorDiff_1)/2;
    uDiff = Kp*errorDiff + Ki*intDiff - Kd*vel_WhlDiff;
    if (fabs(uDiff) > 3) {
        intDiff = intDiff_1;
    }
    if (uDiff > 4) {
        uDiff = 4;
    }
    if ((uDiff) < -4) {
        uDiff = -4;
    }

    WhlDiff_1 = WhlDiff;
    errorDiff_1 = errorDiff;
    vel_WhlDiff_1 = vel_WhlDiff;
    intDiff_1 = intDiff;
    turnrate_1 = turnrate;
    turnref_1 = turnref;

    eSpeed = (Segbot_refSpeed - (LeftWheelSpeed + RightWheelSpeed) / 2.0);
    IK_eSpeed = IK_eSpeed_1 + 0.004 * (eSpeed + eSpeed_1) / 2.0;
    ForwardBackwardCommand = KpSpeed * eSpeed + KiSpeed * IK_eSpeed;

    if (fabs(ForwardBackwardCommand) > 3) {
        IK_eSpeed = IK_eSpeed_1;
    }
    if (ForwardBackwardCommand > 4) {
        ForwardBackwardCommand = 4;
    }
    if ((ForwardBackwardCommand) < -4) {
        ForwardBackwardCommand = -4;
    }

    eSpeed_1 = eSpeed;
    IK_eSpeed_1 = IK_eSpeed;

    gyrorate_dot = 100*(gyro_value - gyro_value_1) + 0.6*gyrorate_dot_1;
    gyro_value_1 = gyro_value;
    gyrorate_dot_1 = gyrorate_dot;

    ubal = 60*tilt_value + 4.5*gyro_value + 1.1*(LeftWheelSpeed + RightWheelSpeed)/2.0 + 0.1*gyrorate_dot;

    uLeft = ubal/2 + uDiff - ForwardBackwardCommand;
    uRight = ubal/2 - uDiff - ForwardBackwardCommand;

    setEPWM2A(uRight);
    setEPWM2B(-uLeft);

    thetaL = LeftWheel;
    thetaR = RightWheel;
    thetaAvg = 0.5*(thetaL + thetaR);
    thetadotL = (thetaL - thetaL_1)/0.004;
    thetadotR = (thetaR - thetaR_1)/0.004;
    thetadotAvg = 0.5*(thetadotL + thetadotR);
    phiR = RWh*(thetaR - thetaL)/WR;
    xdotR = RWh*thetadotAvg*cos(phiR);
    ydotR = RWh*thetadotAvg*sin(phiR);
    xR = xR_1 + (xdotR + xdotR_1)*0.002;
    yR = yR_1 + (ydotR + ydotR_1)*0.002;

    xR_1 = xR;
    yR_1 = yR;
    xdotR_1 = xdotR;
    ydotR_1 = ydotR;
    thetaL_1 = thetaL;
    thetaR_1 = thetaR;

    if (NewLVData == 1) {
        NewLVData = 0;
        Segbot_refSpeed = fromLVvalues[0];
        turnrate = fromLVvalues[1];
        printLV3 = fromLVvalues[2];
        printLV4 = fromLVvalues[3];
        printLV5 = fromLVvalues[4];
        printLV6 = fromLVvalues[5];
        printLV7 = fromLVvalues[6];
        printLV8 = fromLVvalues[7];
    }
    if((numSWIcalls%13) == 0) { // change to the counter variable of you selected 4ms. timer
        DataToLabView.floatData[0] = xR;
        DataToLabView.floatData[1] = yR;
        DataToLabView.floatData[2] = phiR;
        DataToLabView.floatData[3] = 2.0*((float)numSWIcalls)*.001;
        DataToLabView.floatData[4] = 3.0*((float)numSWIcalls)*.001;
        DataToLabView.floatData[5] = (float)numSWIcalls;
        DataToLabView.floatData[6] = (float)numSWIcalls*4.0;
        DataToLabView.floatData[7] = (float)numSWIcalls*5.0;
        LVsenddata[0] = '*'; // header for LVdata
        LVsenddata[1] = '$';
        for (int i=0;i<LVNUM_TOFROM_FLOATS*4;i++) {
            if (i%2==0) {
                LVsenddata[i+2] = DataToLabView.rawData[i/2] & 0xFF;
        } else {
            LVsenddata[i+2] = (DataToLabView.rawData[i/2]>>8) & 0xFF;
        }
    }
    serial_sendSCID(&SerialD, LVsenddata, 4*LVNUM_TOFROM_FLOATS + 2);
    }


    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }
    //    Clear GPIO9 Low to act as a Slave Select. Right now, just to scope. Later to select DAN28027 chip
    //    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
    if (pwm_value >= 3000) {
        pwm_increment = -10;
    } else if (pwm_value <= 0) {
        pwm_increment = 10;
    }
    pwm_value += pwm_increment;

    // VQM Exercise 2 PWM shifting
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 3; // Issue the SPIB_RX_INT when two values are in the RX FIFO
    //    SpibRegs.SPITXBUF = 0x00DA; // 0x4A3B and 0xB517 have no special meaning. Wanted to send
    //    SpibRegs.SPITXBUF = pwm_value; // something so you can see the pattern on the Oscilloscope
    //    SpibRegs.SPITXBUF = pwm_value;

    if ((numTimer0calls%25) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    if ((numTimer0calls%50) == 0) {
        // Blink LaunchPad Red LED
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    numTimer1calls++;

    //    leftWheel = readEncLeft();
    //    rightWheel = readEncRight();
    //
    //    leftDist = leftWheel/radtoft;
    //    rightDist = rightWheel/radtoft;
    //
    //    vLeft = (leftDist - leftDist_1)/0.004;
    //    vRight = (rightDist - rightDist_1)/0.004;
    //
    //    errorTurn = turn + (vLeft - vRight);
    //    errorLeft = Vref - vLeft - KpTurn*errorTurn;
    //    errorRight = Vref - vRight + KpTurn*errorTurn;
    //
    //    if (abs(uLeft) > 10 || abs(uRight) > 10) {
    //        IkLeft = Ik_1Left;
    //        IkRight = Ik_1Right;
    //    } else {
    //        IkLeft = Ik_1Left + 0.002*(errorLeft + error_1Left);
    //        IkRight = Ik_1Right + 0.002*(errorRight + error_1Right);
    //    }
    //
    //    uLeft = Kp*errorLeft + Ki*IkLeft;
    //    uRight = Kp*errorRight + Ki*IkRight;
    //
    //    leftDist_1 = leftDist;
    //    rightDist_1 = rightDist;
    //    error_1Left = errorLeft;
    //    error_1Right = errorRight;
    //    Ik_1Left = IkLeft;
    //    Ik_1Right = IkRight;
    //
    //    setEPWM2A(uRight);
    //    setEPWM2B(-uLeft);
    //
    //    thetaL = leftWheel;
    //    thetaR = rightWheel;
    //    thetaAvg = 0.5*(thetaL + thetaR);
    //    thetadotL = (thetaL - thetaL_1)/0.004;
    //    thetadotR = (thetaR - thetaR_1)/0.004;
    //    thetadotAvg = 0.5*(thetadotL + thetadotR);
    //    phiR = RWh*(thetaR - thetaL)/WR;
    //    xdotR = RWh*thetadotAvg*cos(phiR);
    //    ydotR = RWh*thetadotAvg*sin(phiR);
    //    xR = xR_1 + (xdotR + xdotR_1)*0.002;
    //    yR = yR_1 + (ydotR + ydotR_1)*0.002;
    //
    //    xR_1 = xR;
    //    yR_1 = yR;
    //    xdotR_1 = xdotR;
    //    ydotR_1 = ydotR;
    //    thetaL_1 = thetaL;
    //    thetaR_1 = thetaR;
    //
    //    if (NewLVData == 1) {
    //        NewLVData = 0;
    //        Vref = fromLVvalues[0];
    //        turn = fromLVvalues[1];
    //        printLV3 = fromLVvalues[2];
    //        printLV4 = fromLVvalues[3];
    //        printLV5 = fromLVvalues[4];
    //        printLV6 = fromLVvalues[5];
    //        printLV7 = fromLVvalues[6];
    //        printLV8 = fromLVvalues[7];
    //    }
    //    if((numTimer1calls%13) == 0) { // change to the counter variable of you selected 4ms. timer
    //        DataToLabView.floatData[0] = xR;
    //        DataToLabView.floatData[1] = yR;
    //        DataToLabView.floatData[2] = phiR;
    //        DataToLabView.floatData[3] = 2.0*((float)numTimer1calls)*.001;
    //        DataToLabView.floatData[4] = 3.0*((float)numTimer1calls)*.001;
    //        DataToLabView.floatData[5] = (float)numTimer1calls;
    //        DataToLabView.floatData[6] = (float)numTimer1calls*4.0;
    //        DataToLabView.floatData[7] = (float)numTimer1calls*5.0;
    //        LVsenddata[0] = '*'; // header for LVdata
    //        LVsenddata[1] = '$';
    //        for (int i=0;i<LVNUM_TOFROM_FLOATS*4;i++) {
    //            if (i%2==0) {
    //                LVsenddata[i+2] = DataToLabView.rawData[i/2] & 0xFF;
    //            } else {
    //                LVsenddata[i+2] = (DataToLabView.rawData[i/2]>>8) & 0xFF;
    //            }
    //        }
    //        serial_sendSCID(&SerialD, LVsenddata, 4*LVNUM_TOFROM_FLOATS + 2);
    //    }

    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    //    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 10) == 0) {
    }
}

void setEPWM2A(float controleffort) {
    if (controleffort>10) {
        controleffort = 10;
    }
    if (controleffort<-10) {
        controleffort = -10;
    }
    controleffort = (10.0+controleffort)*0.05;
    EPwm2Regs.CMPA.bit.CMPA = controleffort*EPwm2Regs.TBPRD;
}

void setEPWM2B(float controleffort) {
    if (controleffort>10) {
        controleffort = 10;
    }
    if (controleffort<-10) {
        controleffort = -10;
    }
    controleffort = (10.0+controleffort)*0.05;
    EPwm2Regs.CMPB.bit.CMPB = controleffort*EPwm2Regs.TBPRD;
}

void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}

float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*(-2*PI/(30*400)));
}

float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*(2*PI/(30*400)));
}

__interrupt void SPIB_isr(void) {
    int16_t temp = 0;
    // PWM shifting values
    //    spivalue1 = SpibRegs.SPIRXBUF; // Read first 16-bit value off RX FIFO. Probably is zero since no chip
    //    spivalue2 = SpibRegs.SPIRXBUF; // Read second 16-bit value off RX FIFO. Again probably zero
    //    spivalue3 = SpibRegs.SPIRXBUF;
    //    adcvalue1 = spivalue2*3.3/4096;
    //    adcvalue2 = spivalue3*3.3/4096;

    // Gyro and Accelerometer scaling
    temp = SpibRegs.SPIRXBUF;
    accelXraw = SpibRegs.SPIRXBUF;
    accelYraw = SpibRegs.SPIRXBUF;
    accelZraw = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    gyroXraw = SpibRegs.SPIRXBUF;
    gyroYraw = SpibRegs.SPIRXBUF;
    gyroZraw = SpibRegs.SPIRXBUF;

    gyrox = gyroXraw*250.0/32767.0;
    gyroy = gyroYraw*250.0/32767.0;
    gyroz = gyroZraw*250.0/32767.0;

    accelx = accelXraw*4.0/32767.0;
    accely = accelYraw*4.0/32767.0;
    accelz = accelZraw*4.0/32767.0;

    //Code to be copied into SPIB_ISR interrupt function after the IMU measurements have been collected.
    if(calibration_state == 0){
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 1;
            calibration_count = 0;
        }
    } else if(calibration_state == 1){
        accelx_offset+=accelx;
        accely_offset+=accely;
        accelz_offset+=accelz;
        gyrox_offset+=gyrox;
        gyroy_offset+=gyroy;
        gyroz_offset+=gyroz;
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 2;
            accelx_offset/=2000.0;
            accely_offset/=2000.0;
            accelz_offset/=2000.0;
            gyrox_offset/=2000.0;
            gyroy_offset/=2000.0;
            gyroz_offset/=2000.0;
            calibration_count = 0;
            doneCal = 1;
        }
    } else if(calibration_state == 2){
        accelx -=(accelx_offset);
        accely -=(accely_offset);
        accelz -=(accelz_offset-accelzBalancePoint);
        gyrox -= gyrox_offset;
        gyroy -= gyroy_offset;
        gyroz -= gyroz_offset;
        /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
        float tiltrate = (gyrox*PI)/180.0; // rad/s
        float pred_tilt, z, y, S;
        // Prediction Step
        pred_tilt = kalman_tilt + T*tiltrate;
        pred_P = kalman_P + Q;
        // Update Step
        z = -accelz; // Note the negative here due to the polarity of AccelZ
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P/S;
        kalman_tilt = pred_tilt + kalman_K*y;
        kalman_P = (1 - kalman_K)*pred_P;
        SpibNumCalls++;
        // Kalman Filter used
        tilt_array[SpibNumCalls] = kalman_tilt;
        gyro_array[SpibNumCalls] = tiltrate;
        LeftWheelArray[SpibNumCalls] = readEncLeft();
        RightWheelArray[SpibNumCalls] = readEncRight();
        if (SpibNumCalls >= 3) { // should never be greater than 3
            tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2] + tilt_array[3])/4.0;
            gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3])/4.0;
            LeftWheel=(LeftWheelArray[0]+LeftWheelArray[1]+LeftWheelArray[2]+LeftWheelArray[3])/4.0;
            RightWheel=(RightWheelArray[0]+RightWheelArray[1]+RightWheelArray[2]+RightWheelArray[3])/4.0;
            SpibNumCalls = -1;
            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
        }
    }
    timecount++;
    if((timecount%200) == 0)
    {
        if(doneCal == 0) {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Blink Blue LED while calibrating
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Always Block Red LED
        UARTPrint = 1; // Tell While loop to print
    }
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

    setEPWM2A(uRight);
    setEPWM2B(-uLeft);

    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPIO9 high to end Slave Select. Now Scope. Later to deselect DAN28027
    // Later when actually communicating with the DAN28027 do something with the data. Now do nothing.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt
}

void setupSPIB(void) //Call this function in main() somewhere after the DINT; line of code.
{
    uint16_t temp = 0;
    //Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in
    //between each transfer to 0. Also don’t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65,
    //66 which are also a part of the SPIB setup.
    SpibRegs.SPICCR.bit.SPICHAR = 0xF;
    SpibRegs.SPIFFCT.bit.TXDLY = 0x00;
    SpibRegs.SPICCR.bit.SPISWRESET = 0x0; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 0x1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16-bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 0x1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0x0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK’s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 0x1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 0x1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just  in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 0; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don’t think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL = 0x10; //Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMO
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB
    //---------------------------------------------------------------------------------------------------
    //--------------
    //Step 2.
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending
    //16-bit transfers, so two registers at a time after the first 16-bit transfer.
    // To address 00x13 write 0x00
    SpibRegs.SPITXBUF = (0x1300);
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    SpibRegs.SPITXBUF = (0x0000);
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    SpibRegs.SPITXBUF = (0x0000);
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    SpibRegs.SPITXBUF = (0x0013);
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    SpibRegs.SPITXBUF = (0x0200);
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    SpibRegs.SPITXBUF = (0x0806);
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00
    SpibRegs.SPITXBUF = (0x0000);
    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 0x07);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // VQM read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO, we are sending 7 SPI values and reading 7 SPI values back,
    // but we need to ignore them because they are useless
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.
    // Step 3.
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00
    SpibRegs.SPITXBUF = (0x2300);
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = (0x408C);
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    SpibRegs.SPITXBUF = (0x0288);
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A
    SpibRegs.SPITXBUF = (0x0C0A);
    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 0x04);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.
    // Step 4.
    // perform a single 16-bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = (0x2A81);
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x0019); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x005C); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x0019); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x0028); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x001D); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x00F0); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);
    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

