/// Values stored in flash, which are modified by user actions ///

#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// length of float reg is 64
#define I_BW                    __float_reg[2]                                  // Current loop bandwidth
#define I_MAX                   __float_reg[3]                                  // Current limit
#define I_MAX_CONT              __float_reg[4]                                  // Continuous max current
#define I_CAL					__float_reg[5]									// Calibration Current
#define I_FW_MAX                __float_reg[6]                                  // Maximum field weakening current
#define THETA_MIN               __float_reg[7]                                  // Minimum position setpoint
#define THETA_MAX               __float_reg[8]                                  // Maximum position setpoint

//#define K_SCALE					__float_reg[9]									// Current Controller KP scaled w/ BW
//#define KI_D					__float_reg[10]									// Current Controller KI D-axis
//#define KI_Q					__float_reg[11]									// Current Controller KI Q-axis

#define PPAIRS					__float_reg[12]									// Number of motor pole-pairs
#define GR						__float_reg[13]									// Gear ratio
#define KT_OUT						__float_reg[14]									// Torque Constant (N-m/A)
#define L_D						__float_reg[15]									// D-axis inductance
#define L_Q						__float_reg[16]									// Q-axis inductance
#define R_PHASE					__float_reg[17]									// Single phase resistance
#define R_NOMINAL               __float_reg[18]                                  // Nominal motor resistance, set during calibration

#define R_TH					__float_reg[19]									// Thermal resistance (C/W)
#define C_TH					__float_reg[20]									// Thermal mass (C/J)
#define INV_M_TH				__float_reg[21]									// Observer Matrix (K/J)
#define T_AMBIENT				__float_reg[22]									// Ambient temperature at calibration (C)
#define TEMP_MAX                __float_reg[23]                                  // Temperature safety limit

#define P_MIN					__float_reg[24]									// Position setpoint lower limit (rad)
#define P_MAX					__float_reg[25]									// Position setpoint upper bound (rad)
#define V_MIN					__float_reg[26]									// Velocity setpoint lower bound (rad/s)
#define V_MAX					__float_reg[27]									// Velocity setpoint upper bound (rad/s)
#define KP_MAX					__float_reg[28]									// Max position gain (N-m/rad)
#define KD_MAX					__float_reg[29]									// Max velocity gain (N-m/rad/s)
#define T_MIN					__float_reg[30]									// Torque setpoint lower limit (Nm)
#define T_MAX					__float_reg[31]									// Torque setpoint upper limit (Nm)

// length of int reg is 256
#define PHASE_ORDER             __int_reg[0]                                    // Phase swapping during calibration
#define CAN_ID                  __int_reg[1]                                    // CAN bus ID
#define CAN_MASTER              __int_reg[2]                                    // CAN bus "master" ID
#define CAN_TIMEOUT             __int_reg[3]                                    // CAN bus timeout period
#define EN_ENC_FILTER			__int_reg[4]									// Enable encoder filter
#define EN_ENC_LINEARIZE 		__int_reg[5]									// Enable encoder linearization
#define M_ZERO					__int_reg[6]									// Encoder mechanical zero (counts)
#define E_ZERO					__int_reg[7]									// Encoder electrical zero (counts)
#define ENCODER_LUT             __int_reg[8]                                    // Encoder offset LUT - 128 elements long




extern float __float_reg[];
extern int __int_reg[];

#ifdef __cplusplus
}
#endif

#endif
