/*
 * position_sensor.c
 *
 *  Created on: Jul 26, 2020
 *      Author: Ben
 */
#include <stdio.h>
#include <string.h>
#include "position_sensor.h"
#include "math_ops.h"
#include "hw_config.h"
#include "user_config.h"


void ps_warmup(EncoderStruct * encoder, int n){
	/* Hall position sensors noisy on startup.  Take a bunch of samples to clear this data */
//	int raw;
	for(int i = 0; i<n; i++){
		encoder->spi_tx_buff[0] = 0xA6;
		encoder->spi_tx_buff[1] = 0x00;
		encoder->spi_tx_buff[2] = 0x00;
		encoder->spi_tx_buff[3] = 0x00;
		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
		delay_us(1);
		HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 4, 100);
		while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
		delay_us(100);
//		raw = ((encoder->spi_rx_buff[1]<<16)|(encoder->spi_rx_buff[2]<<8)|(encoder->spi_rx_buff[3]))>>5;
//		printf("%d\n\r", raw);
		delay_us(100);
	}
}

void ps_sample(EncoderStruct * encoder, float dt){
	/* updates EncoderStruct encoder with the latest sample
	 * after elapsed time dt */

	/* Shift around previous samples */
	encoder->old_angle = encoder->angle_singleturn;
	for(int i = N_POS_SAMPLES-1; i>0; i--){encoder->angle_multiturn[i] = encoder->angle_multiturn[i-1];}
	//for(int i = N_POS_SAMPLES-1; i>0; i--){encoder->count_buff[i] = encoder->count_buff[i-1];}
	//memmove(&encoder->angle_multiturn[1], &encoder->angle_multiturn[0], (N_POS_SAMPLES-1)*sizeof(float)); // this is much slower for some reason

	/* SPI read/write */
	encoder->spi_tx_buff[0] = 0xA6;
	encoder->spi_tx_buff[1] = 0x00;
	encoder->spi_tx_buff[2] = 0x00;
	encoder->spi_tx_buff[3] = 0x00;
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	delay_us(1);
	HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 4, 100);
	while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high

	encoder->raw = ((encoder->spi_rx_buff[1]<<16)|(encoder->spi_rx_buff[2]<<8)|(encoder->spi_rx_buff[3]))>>5;

	/* Linearization */
	encoder->offset_ind1 = (encoder->raw)>>LUT_SHIFT;
	encoder->offset_ind2 = ((encoder->raw>>LUT_SHIFT)+1)%128;
	encoder->offset1 = encoder->offset_lut[(encoder->raw)>>LUT_SHIFT];				// lookup table lower entry
	encoder->offset2 = encoder->offset_lut[((encoder->raw>>LUT_SHIFT)+1)%128];		// lookup table higher entry
	int raw_mod = encoder->raw & LUT_MASK;
	encoder->offset_interp = encoder->offset1 + ( (encoder->offset2-encoder->offset1) * raw_mod / (1<<LUT_SHIFT) );     // Interpolate between lookup table entries


	if (EN_ENC_LINEARIZE == 1){
		encoder->count = encoder->raw + encoder->offset_interp;
	} else {
		encoder->count = encoder->raw;
	}

	/* Real angles in radians */
	// MAPPED FROM -PI to PI, instead of 0 to 2*PI
	encoder->angle_singleturn = TWO_PI_F*((float)(encoder->count-M_ZERO))/((float)ENC_CPR);
//	encoder->angle_singleturn = encoder->angle_singleturn<0 ? encoder->angle_singleturn + TWO_PI_F : encoder->angle_singleturn;
	encoder->angle_singleturn = encoder->angle_singleturn<-PI_F ? encoder->angle_singleturn + TWO_PI_F : encoder->angle_singleturn;
	encoder->angle_singleturn = encoder->angle_singleturn>PI_F  ? encoder->angle_singleturn - TWO_PI_F : encoder->angle_singleturn;

	encoder->elec_angle = (encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR);
	int mod_angle = (int)encoder->elec_angle;
	encoder->elec_angle = TWO_PI_F*(encoder->elec_angle - (float)mod_angle);
	encoder->elec_angle = encoder->elec_angle<0 ? encoder->elec_angle + TWO_PI_F : encoder->elec_angle;	// Add 2*pi to negative numbers

	/* Rollover */
	int rollover = 0;
	float angle_diff = encoder->angle_singleturn - encoder->old_angle;
	if(angle_diff > PI_F){rollover = -1;}
	else if(angle_diff < -PI_F){rollover = 1;}
	if(!encoder->first_sample){
		encoder->turns = 0;
		encoder->first_sample = 1;
	} else {
		encoder->turns += rollover;
	}

	/* Multi-turn position */
	encoder->angle_multiturn[0] = encoder->angle_singleturn + TWO_PI_F*(float)encoder->turns;

	/* Velocity */

	// old velocity calculation modified to match MBed code calculation of velocity!
	//encoder->velocity = (encoder->angle_multiturn[0] - encoder->angle_multiturn[N_POS_SAMPLES-1])/(dt*(float)(N_POS_SAMPLES-1));
	encoder->single_vel = (encoder->angle_multiturn[0] - encoder->angle_multiturn[1])/dt;

	// Filter out bad position samples
	if ( (encoder->filt_enable==1) && ((encoder->single_vel > (V_MAX*GR)) || (encoder->single_vel < (V_MIN*GR))) ) {
		encoder->angle_multiturn[0] = encoder->filt_prev_mech;
		encoder->elec_angle = encoder->filt_prev_elec;
		encoder->single_vel = 0.0;
	}
	else {
		encoder->filt_prev_mech = encoder->angle_multiturn[0];
		encoder->filt_prev_elec = encoder->elec_angle;
	}

	float sum = encoder->single_vel;
	for (int i = 1; i < N_POS_SAMPLES; i++){
		encoder->vel_vec[N_POS_SAMPLES - i] = encoder->vel_vec[N_POS_SAMPLES-i-1];
		sum += encoder->vel_vec[N_POS_SAMPLES-i];
		}
	encoder->vel_vec[0] = encoder->single_vel;
	encoder->velocity =  sum/((float)N_POS_SAMPLES);
	encoder->elec_velocity = encoder->ppairs*encoder->velocity;

}

void ps_print(EncoderStruct * encoder){
	printf("   Raw: %u", (unsigned int)encoder->raw);
//	printf("   LUT ind 1: %d", encoder->offset_ind1);
//	printf("   LUT ind 2: %d", encoder->offset_ind2);
//	printf("   Offset 1: %d", encoder->offset1);
//	printf("   Offset 2: %d", encoder->offset2);
//	printf("   Offset Interp: %d", encoder->offset_interp);
	printf("   Linearized: %d", encoder->count);
	printf("   Single Turn: %.3f", encoder->angle_singleturn);
	printf("   Multiturn: %.3f", encoder->angle_multiturn[0]);
	printf("   Electrical: %.3f", encoder->elec_angle);
	printf("   Turns:  %d", encoder->turns);
	printf("   Vel: %.4f\n\r", encoder->velocity);
	delay_us(10000);
}

void ps_zero(EncoderStruct * encoder){
	encoder->turns = 0;
	encoder->m_zero = 0;
	ps_sample(encoder, DT); //.00025f);
	encoder->m_zero = encoder->angle_singleturn;
}

void WriteLUT(EncoderStruct * encoder,  int new_lut[N_LUT]){
	memcpy(encoder->offset_lut, new_lut, sizeof(encoder->offset_lut));
}

void ps_filter_init(EncoderStruct * encoder){
	encoder->filt_prev_mech = encoder->angle_multiturn[0];
	encoder->filt_prev_elec = encoder->elec_angle;
}






