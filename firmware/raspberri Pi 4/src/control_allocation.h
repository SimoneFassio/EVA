#ifndef _CONTROL_ALLOCATION_H
#define _CONTROL_ALLOCATION_H

// TCM (Thrust Configuration Matrix) parameters of interest
// Indices represent the row and column indexes of the TCM
#define a51 0.2500
#define a61 0.2500
#define a71 0.2500
#define a81 0.2500
#define a52 0.9568
#define a72 0.9568
#define a73 0.9568
#define a83 0.9568
#define a53 -0.9568 
#define a62 -0.9568 
#define a63 -0.9568
#define a82 -0.9568 

// Lookup table interpolation coefficients
#define THETA_1_1 0.0158*1000
#define THETA_1_2 0.1633*1000
#define THETA_1_3 1.4527*1000
#define THETA_2_1 -0.0105*1000
#define THETA_2_2 0.1304*1000
#define THETA_2_3 1.5477*1000
#define THETA_C_1 4.0000*1000
#define THETA_C_2 1.5000*1000

#define EPS  0.01
#define MAX_kgf 3.5 //Max forward force of a single thrust with 12V of power supply
#define MIN_kgf -2.6  //Max reverce force of a single thrust with 12V of power supply


struct OutputValues 
{
    double T5, T6, T7, T8; // Vertical thrusters (that are the ones that affect z roll and pitch)
};

OutputValues compute_PWM(double Fz, double Fr, double Fp); // Convert Force along z and moments around roll and pitch into thrust forces in PWM

#endif