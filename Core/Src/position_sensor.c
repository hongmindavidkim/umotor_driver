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
	int raw;
	for(int i = 0; i<n; i++){
		encoder->spi_tx_buff[0] = 0xA6;
		encoder->spi_tx_buff[1] = 0x00;
		encoder->spi_tx_buff[2] = 0x00;
		encoder->spi_tx_buff[3] = 0x00;
		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
		HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 4, 100);
		while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
		delay_us(100);
		raw = ((encoder->spi_rx_buff[1]<<16)|(encoder->spi_rx_buff[2]<<8)|(encoder->spi_rx_buff[3]))>>5;
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
	HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 4, 100);
	while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high

	encoder->raw = ((encoder->spi_rx_buff[1]<<16)|(encoder->spi_rx_buff[2]<<8)|(encoder->spi_rx_buff[3]))>>5;

	/* Linearization */
	int off_1 = encoder->offset_lut[(encoder->raw)>>LUT_SHIFT];				// lookup table lower entry
	int off_2 = encoder->offset_lut[((encoder->raw>>LUT_SHIFT)+1)%128];		// lookup table higher entry
	int off_interp = off_1 + ((off_2 - off_1)*(encoder->raw - ((encoder->raw>>LUT_SHIFT)<<LUT_SHIFT))>>LUT_SHIFT);     // Interpolate between lookup table entries
	encoder->count = encoder->raw + off_interp;

	/* Real angles in radians */
	encoder->angle_singleturn = ((float)(encoder->count-M_ZERO))/((float)ENC_CPR);
	int int_angle = encoder->angle_singleturn;
	encoder->angle_singleturn = TWO_PI_F*(encoder->angle_singleturn - (float)int_angle);
	//encoder->angle_singleturn = TWO_PI_F*fmodf(((float)(encoder->count-M_ZERO))/((float)ENC_CPR), 1.0f);
	encoder->angle_singleturn = encoder->angle_singleturn<0 ? encoder->angle_singleturn + TWO_PI_F : encoder->angle_singleturn;

	encoder->elec_angle = (encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR);
	int_angle = (int)encoder->elec_angle;
	encoder->elec_angle = TWO_PI_F*(encoder->elec_angle - (float)int_angle);
	//encoder->elec_angle = TWO_PI_F*fmodf((encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR), 1.0f);
	encoder->elec_angle = encoder->elec_angle<0 ? encoder->elec_angle + TWO_PI_F : encoder->elec_angle;	// Add 2*pi to negative numbers
	/* Rollover */
	int rollover = 0;
	float angle_diff = encoder->angle_singleturn - encoder->old_angle;
	if(angle_diff > PI_F){rollover = -1;}
	else if(angle_diff < -PI_F){rollover = 1;}
	encoder->turns += rollover;
	if(!encoder->first_sample){
		encoder->turns = 0;
		encoder->first_sample = 1;
	}

	/* Multi-turn position */
	encoder->angle_multiturn[0] = encoder->angle_singleturn + TWO_PI_F*(float)encoder->turns;

	/* Velocity */

	// old velocity calculation
//	encoder->velocity = (encoder->angle_multiturn[0] - encoder->angle_multiturn[N_POS_SAMPLES-1])/(dt*(float)(N_POS_SAMPLES-1));

	// modified to match MBed code calculation of velocity!
	encoder->single_vel = (encoder->angle_multiturn[0] - encoder->angle_multiturn[1])/dt;

	//ADD FILTER HERE TO THE UNFILTERED VELOCITY
	if ( ((encoder->single_vel > (V_U12_MAX*RATIO)) || (encoder->single_vel < (V_U12_MIN*RATIO))) && (encoder->filt_enable==1)) {
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
	printf("   Linearized Count: %d", encoder->count);
	printf("   Single Turn: %f", encoder->angle_singleturn);
	printf("   Multiturn: %f", encoder->angle_multiturn[0]);
	printf("   Electrical: %f", encoder->elec_angle);
	printf("   Turns:  %d", encoder->turns);
	printf("   Vel: %f\n\r", encoder->velocity);
	delay_us(10000);
}

void pc_zero(EncoderStruct * encoder){
	// TODO: check this!
	encoder->turns = 0;
	encoder->m_zero = 0;
	ps_sample(encoder, .00025f);
	encoder->m_zero = encoder->angle_singleturn;
}

void WriteLUT(EncoderStruct * encoder,  int new_lut[N_LUT]){
	memcpy(encoder->offset_lut, new_lut, sizeof(encoder->offset_lut));
}

void ps_filter_init(EncoderStruct * encoder){
	encoder->filt_prev_mech = encoder->angle_multiturn[0];
	encoder->filt_prev_elec = encoder->elec_angle;
}

//void Sample(EncoderCMUStruct * encoder, float dt){
////	unsigned long t1 = DWT->CYCCNT;
//    /* SPI read/write */
//	for(int i = 0; i<4; i++){
//		if (i%4 == 0){ // alternate between A600 and 0000 to get A6 00 00 00
//			encoder->spi_tx_byte = 0xA6;
//		} else {
//			encoder->spi_tx_byte = 0x00;
//		}
////		printf("encoder tx: %x \n\r", (unsigned int)encoder->spi_tx_byte);
//
//		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
////		HAL_SPI_Transmit(&ENC_SPI, (uint8_t*)&encoder->spi_tx_byte, 1, 100);
//		HAL_SPI_TransmitReceive(&ENC_SPI, (uint8_t *)&encoder->spi_tx_byte, (uint8_t *)&encoder->spi_rx_byte, 1, 2000);
//		while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
//		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
//		encoder->raw_bytes[i] = encoder -> spi_rx_byte;
////		printf("encoder rx: %x \n\r", (unsigned int)encoder->spi_rx_byte);
//	}
//
//
//	encoder->raw = encoder->raw_bytes[1]<<16 | encoder->raw_bytes[2]<<8 | encoder->raw_bytes[3];
//	// From 24 received bits: 14bits MAS, 5 bits NON, 5 zero bits
//	// With default filter, get 14 bits of usable data
//	// But, still extract 19 bits
//	encoder->raw = encoder->raw>>5;
////	printf("encoder raw: %d \n\r", (unsigned int)encoder->raw);
////	printf("encoder raw: %x  %x  %x  %x \n\r", (unsigned int)encoder->raw_bytes[3], (unsigned int)encoder->raw_bytes[2], (unsigned int)encoder->raw_bytes[1], (unsigned int)encoder->raw_bytes[0]);
//
//
//	//GPIOA->ODR |= (1 << 15);
//	int off_1 = encoder->offset_lut[encoder->raw>>encoder->lut_shift]; // shift 12 bits to go from 19-bit to 7-bit lut index
//	int off_2 = encoder->offset_lut[((encoder->raw>>encoder->lut_shift)+1)%128];
//
//
//	int off_interp = off_1 + ((off_2 - off_1)*(encoder->raw - ((encoder->raw>>encoder->lut_shift)<<encoder->lut_shift))>>encoder->lut_shift);        // Interpolate between lookup table entries
//	int angle = encoder->raw + off_interp;                                               // Correct for nonlinearity with lookup table from calibration
//
//	if(encoder->first_sample){
//		if(angle - encoder->old_counts > encoder->_CPR/2){
//			encoder->rotations -= 1;
//			}
//		else if (angle - encoder->old_counts < -(encoder->_CPR)/2){
//			encoder->rotations += 1;
//			}
//	}
//	if(!encoder->first_sample){encoder->first_sample = 1;}
//
//	encoder->old_counts = angle;
//	encoder->oldModPosition = encoder->modPosition;
//	encoder->modPosition = ((2.0f*PI_F * ((float) angle))/ (float)encoder->_CPR);
//	encoder->position = (2.0f*PI_F * ((float) angle+(encoder->_CPR*encoder->rotations)))/ (float)encoder->_CPR;
//
//	encoder->MechPosition = encoder->position - encoder->MechOffset; // is this mech position of the rotor or output?
//
//	float elec = ((2.0f*PI_F/(float)encoder->_CPR) * (float) ((encoder->_ppairs*angle)%encoder->_CPR)) + encoder->ElecOffset;
//	if(elec < 0) elec += 2.0f*PI_F;
//	else if(elec > 2.0f*PI_F) elec -= 2.0f*PI_F ;
//	encoder->ElecPosition = elec;
//
//	float vel;
//	//if(modPosition<.1f && oldModPosition>6.1f){
//
//	if((encoder->modPosition-encoder->oldModPosition) < -3.0f){
//		vel = (encoder->modPosition - encoder->oldModPosition + 2.0f*PI_F)/dt;
//		}
//	//else if(modPosition>6.1f && oldModPosition<0.1f){
//	else if((encoder->modPosition - encoder->oldModPosition) > 3.0f){
//		vel = (encoder->modPosition - encoder->oldModPosition - 2.0f*PI_F)/dt;
//		}
//	else{
//		vel = (encoder->modPosition-encoder->oldModPosition)/dt;
//	}
//
//	int n = 40;
//	float sum = vel;
//	for (int i = 1; i < (n); i++){
//		encoder->velVec[n - i] = encoder->velVec[n-i-1];
//		sum += encoder->velVec[n-i];
//		}
//	encoder->velVec[0] = vel;
//	encoder->MechVelocity =  sum/((float)n); // is this mech velocity of the rotor or output?
//
//	encoder->ElecVelocity = encoder->MechVelocity*encoder->_ppairs;
//	encoder->ElecVelocityFilt = 0.99f*encoder->ElecVelocityFilt + 0.01f*encoder->ElecVelocity;
//
////	unsigned long t2 = DWT->CYCCNT;
////	unsigned long diff = t2 - t1;
////	printf("diff: %d", (int)diff);
//}
//
//





