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

extern int loop_time;

void ps_warmup(EncoderStruct * encoder, int n){
	/* Hall position sensors noisy on startup.  Take a bunch of samples to clear this data */
//	int raw;
	HAL_StatusTypeDef hal_status;
	encoder->spi_tx_buff[0] = 0xA6;
	encoder->spi_tx_buff[1] = 0x00;
	encoder->spi_tx_buff[2] = 0x00;
	encoder->spi_tx_buff[3] = 0x00;

	for(int i = 0; i<n; i++){
		while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
		hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 4, 100);
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
	HAL_StatusTypeDef hal_status;
	encoder->spi_tx_buff[0] = 0xA6;
	encoder->spi_tx_buff[1] = 0x00;
	encoder->spi_tx_buff[2] = 0x00;
	encoder->spi_tx_buff[3] = 0x00;
	while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 4, 100);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high

//	encoder->raw = ((encoder->spi_rx_buff[1]<<16)|(encoder->spi_rx_buff[2]<<8)|(encoder->spi_rx_buff[3]))>>5;

	if (hal_status == HAL_OK){ // only update on HAL OK
		encoder->raw = ((encoder->spi_rx_buff[1]<<16)|(encoder->spi_rx_buff[2]<<8)|(encoder->spi_rx_buff[3]))>>5;
	} else {
		encoder->raw = 0;
	}

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

void ps_activate(EncoderStruct * encoder){

	// activate op code is 0xB0
	HAL_StatusTypeDef hal_status;

	encoder->spi_tx_buff[0] = 0xB0;
	encoder->spi_tx_buff[1] = 0b10000011; // set RACTIVE and PACTIVE for one device

	while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 2, 100);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high

}

void ps_deactivate(EncoderStruct * encoder){

	HAL_StatusTypeDef hal_status;

	encoder->spi_tx_buff[0] = 0xB0;
	encoder->spi_tx_buff[1] = 0b10000000; // clear RACTIVE and PACTIVE for one device

	while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 2, 100);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
}

void ps_spi_status(EncoderStruct * encoder){

	HAL_StatusTypeDef hal_status;

	encoder->spi_tx_buff[0] = 0xAD; // register status command
	encoder->spi_tx_buff[1] = 0x00;
	encoder->spi_tx_buff[2] = 0x00;

	while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 3, 100);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high

	encoder->status[0] = (encoder->spi_rx_buff[1]&0xFFFF);

}

void ps_abs_reset(EncoderStruct * encoder){

	// write register: 0xD2
	// command register address is 0x75
	// ABS_RESET command is 0x03
	// register status/data: 0xAD

	HAL_StatusTypeDef hal_status;

	encoder->spi_tx_buff[0] = 0xD2; // write register command
	encoder->spi_tx_buff[1] = 0x75; // register address to write to
	encoder->spi_tx_buff[2] = 0x03; // ABS_RESET command

	while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 3, 100);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
	delay_us(10);

	encoder->spi_tx_buff[0] = 0xAD; // register status command
	encoder->spi_tx_buff[1] = 0x00;
	encoder->spi_tx_buff[2] = 0x00;

	while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 3, 100);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high

	encoder->status[0] = (encoder->spi_rx_buff[1]<<8)|(encoder->spi_rx_buff[2]);

}

void ps_non_ver(EncoderStruct * encoder){

	// write register: 0xD2
	// command register address is 0x75
	// NON_VER command is 0x04
	// register status/data: 0xAD

	HAL_StatusTypeDef hal_status;

	encoder->spi_tx_buff[0] = 0xD2; // write register command
	encoder->spi_tx_buff[1] = 0x75; // register address to write to
	encoder->spi_tx_buff[2] = 0x04; // NON_VER command

	while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 3, 100);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
	delay_us(10);

	encoder->spi_tx_buff[0] = 0xAD; // register status command
	encoder->spi_tx_buff[1] = 0x00;
	encoder->spi_tx_buff[2] = 0x00;

	while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 3, 100);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high

	encoder->status[0] = (encoder->spi_rx_buff[1]<<8)|(encoder->spi_rx_buff[2]);
}

void ps_set_filter(EncoderStruct * encoder, uint8_t filt){

	// write register: 0xD2
	// command register address is 0x0E
	// register status/data: 0xAD

	HAL_StatusTypeDef hal_status;

	encoder->spi_tx_buff[0] = 0xD2; // write register command
	encoder->spi_tx_buff[1] = 0x0E; // register address to write to
	encoder->spi_tx_buff[2] = (filt|0b00000111); // set bits 2:0

	while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 3, 100);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
	delay_us(10);

	encoder->spi_tx_buff[0] = 0xAD; // register status command
	encoder->spi_tx_buff[1] = 0x00;
	encoder->spi_tx_buff[2] = 0x00;

	while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 3, 100);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high

	encoder->status[0] = (encoder->spi_rx_buff[1]<<8)|(encoder->spi_rx_buff[2]);

}

void ps_read_reg(EncoderStruct * encoder, uint8_t addr){

    // read register: 0x97
    // register status/data: 0xAD

	HAL_StatusTypeDef hal_status;

	encoder->spi_tx_buff[0] = 0x97; // read register command
	encoder->spi_tx_buff[1] = addr; // register address to read

	while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 2, 100);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
	delay_us(10);

	encoder->spi_tx_buff[0] = 0xAD; // register status command
	encoder->spi_tx_buff[1] = 0x00;
	encoder->spi_tx_buff[2] = 0x00;

	while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 3, 100);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high

	encoder->status[0] = (encoder->spi_rx_buff[1]<<8)|(encoder->spi_rx_buff[2]);
}

void ps_full_status(EncoderStruct * encoder){

    // read register: 0x97
    // register status/data: 0xAD

    // read both status registers:
    // status0 is address 0x76
    // status1 is address 0x77

	HAL_StatusTypeDef hal_status;

	encoder->spi_tx_buff[0] = 0x97; // read register command
	encoder->spi_tx_buff[1] = 0x76; // register address to read

	while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 2, 100);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
	delay_us(10);

	encoder->spi_tx_buff[0] = 0xAD; // register status command
	encoder->spi_tx_buff[1] = 0x00;
	encoder->spi_tx_buff[2] = 0x00;

	while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 3, 100);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high

	encoder->status[0] = (encoder->spi_rx_buff[1]<<8)|(encoder->spi_rx_buff[2]);
	delay_us(10);

	encoder->spi_tx_buff[0] = 0x97; // read register command
	encoder->spi_tx_buff[1] = 0x77; // register address to read

	while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 2, 100);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
	delay_us(10);

	encoder->spi_tx_buff[0] = 0xAD; // register status command
	encoder->spi_tx_buff[1] = 0x00;
	encoder->spi_tx_buff[2] = 0x00;

	while( HAL_SPI_GetState(&ENC_SPI) != HAL_SPI_STATE_READY){;}
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	hal_status = HAL_SPI_TransmitReceive(&ENC_SPI, encoder->spi_tx_buff, encoder->spi_rx_buff, 3, 100);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high

	encoder->status[1] = (encoder->spi_rx_buff[1]<<8)|(encoder->spi_rx_buff[2]);

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
	printf("   Vel: %.4f", encoder->velocity);
	printf("   Main loop time: %d", loop_time);

	uint16_t b0 = encoder->status[0];
	uint16_t b1 = encoder->status[1];
	printf("   Status: "); // try to print these in binary
	printf("0b%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d, ",
			(b0 >> 15) & 1, (b0 >> 14) & 1, (b0 >> 13) & 1, (b0 >> 12) & 1,
		    (b0 >> 11) & 1, (b0 >> 10) & 1, (b0 >> 9) & 1, (b0 >> 8) & 1,
		    (b0 >> 7) & 1, (b0 >> 6) & 1, (b0 >> 5) & 1, (b0 >> 4) & 1,
		    (b0 >> 3) & 1, (b0 >> 2) & 1, (b0 >> 1) & 1, (b0 >> 0) & 1);
	printf("0b%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d\n\r",
			(b1 >> 15) & 1, (b1 >> 14) & 1, (b1 >> 13) & 1, (b1 >> 12) & 1,
			(b1 >> 11) & 1, (b1 >> 10) & 1, (b1 >> 9) & 1, (b1 >> 8) & 1,
			(b1 >> 7) & 1, (b1 >> 6) & 1, (b1 >> 5) & 1, (b1 >> 4) & 1,
			(b1 >> 3) & 1, (b1 >> 2) & 1, (b1 >> 1) & 1, (b1 >> 0) & 1);

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






