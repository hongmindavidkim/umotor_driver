/*
 * position_sensor.h
 *
 *  Created on: Jul 26, 2020
 *      Author: Ben
 */

#ifndef INC_POSITION_SENSOR_H_
#define INC_POSITION_SENSOR_H_


//#include "structs.h"
#include "tim.h"
#include "spi.h"
#include "fsm.h"
#include <stdint.h>

#define N_POS_SAMPLES 20		// Number of position samples to store.  should put this somewhere else...
#define N_LUT 128
#define LUT_SHIFT 12					// bits to shift for linearizing LUT
#define LUT_MASK 0x0FFF

typedef struct{
	uint8_t spi_tx_buff[4];
	uint8_t spi_rx_buff[4];
	uint32_t raw;
	float angle_singleturn, old_angle, angle_multiturn[N_POS_SAMPLES], elec_angle, velocity, elec_velocity, ppairs, vel2;
	float output_angle_multiturn;
	float filt_prev_mech, filt_prev_elec;
	float vel_vec[N_POS_SAMPLES], single_vel, filter_check_vel;
	int filt_enable, filt_num_samples, filt_time_us;
	
	int init_status;
	float init_offset;
	int offset_interp, offset1, offset2, offset_ind1, offset_ind2;
	int count, old_count, turns;
	int count_buff[N_POS_SAMPLES];
	int m_zero, e_zero;
	int offset_lut[N_LUT];
	uint8_t first_sample;
	uint16_t status[2];
	//    float position, ElecPosition, ElecOffset, MechPosition, MechOffset,
	//    float modPosition, oldModPosition, oldVel, velVec[40], MechVelocity, ElecVelocity, ElecVelocityFilt,
	//    float prev_mech, prev_elec;
	//    int raw, lut_shift, _CPR, rotations, old_counts, _ppairs, first_sample, raw_bytes[8], enable_filter;

} EncoderStruct;

void ps_warmup(EncoderStruct * encoder, int n);
void ps_sample(EncoderStruct * encoder, float dt);
void ps_print(EncoderStruct * encoder);
void ps_zero(EncoderStruct * encoder);
void WriteLUT(EncoderStruct * encoder, int new_lut[N_LUT]);
void ps_filter_init(EncoderStruct * encoder);
void ps_full_status(EncoderStruct * encoder);
void ps_spi_status(EncoderStruct * encoder);
void ps_activate(EncoderStruct * encoder);
void ps_deactivate(EncoderStruct * encoder);
void ps_abs_reset(EncoderStruct * encoder);
void ps_non_ver(EncoderStruct * encoder);
void ps_set_filter(EncoderStruct * encoder, uint8_t filt);


//    PositionSensoriCMU(int CPR, float offset, int ppairs);
//    virtual void Sample(float dt);
//    virtual float GetMechPosition();
//    virtual float GetMechPositionFixed();
//    virtual float GetElecPosition();
//    virtual float GetMechVelocity();
//    virtual float GetElecVelocity();
//    virtual int GetRawPosition();
//    virtual void ZeroPosition();
//    virtual void SetElecOffset(float offset);
//    virtual void SetMechOffset(float offset);
//    virtual int GetCPR(void);
//    virtual void WriteLUT(int new_lut[128]);
//    virtual void FilterInit();
//    virtual void EnableFilter();
//    virtual void DisableFiter();


#endif /* INC_POSITION_SENSOR_H_ */

