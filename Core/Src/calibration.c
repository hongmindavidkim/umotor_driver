/*
 * calibration.c
 *
 *  Created on: Aug 11, 2020
 *      Author: ben
 */


#include "calibration.h"
#include "hw_config.h"
#include "user_config.h"
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
#include "math_ops.h"

void order_phases(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){
	/* Checks phase order, to ensure that positive Q current produces
	   torque in the positive direction wrt the position sensor */
	PHASE_ORDER = 0;

	if(!cal->started){
		printf("\n\r\n\rChecking phase sign, pole pairs\r\n");
		cal->started = 1;
		cal->start_count = loop_count;
	}
	cal->time = (float)(loop_count - cal->start_count)*DT;

    if(cal->time < T1){
        // Set voltage angle to zero, wait for rotor position to settle
        cal->theta_ref = 0;//W_CAL*cal->time;
        cal->cal_position.elec_angle = cal->theta_ref;
        cal->cal_position.elec_velocity = 0;
        controller->i_d_des = I_CAL;
        controller->i_q_des = 0.0f;
        commutate(controller, &cal->cal_position);
    	cal->theta_start = encoder->angle_multiturn[0];
    	return;
    }

    else if(cal->time < T1+2.0f*PI_F/W_CAL){
    	// rotate voltage vector through one electrical cycle
    	cal->theta_ref = W_CAL*(cal->time-T1);
    	cal->cal_position.elec_angle = cal->theta_ref;
		commutate(controller, &cal->cal_position);
    	return;
    }

	reset_foc(controller);

	float theta_end = encoder->angle_multiturn[0];
	cal->ppairs = round(2.0f*PI_F/fabsf(theta_end-cal->theta_start));

	if(cal->theta_start < theta_end){
		cal->phase_order = 0;
		printf("Phase order correct\r\n");
	}
	else{
		cal->phase_order = 1;
		printf("Swapping phase sign\r\n");
	}
    printf("Pole Pairs: %d\r\n", cal->ppairs);
    printf("Start: %.3f   End: %.3f\r\n", cal->theta_start, theta_end);
    PHASE_ORDER = cal->phase_order;
//    PPAIRS = (float)cal->ppairs;
    cal->started = 0;
    cal->done_ordering = 1;	// Finished checking phase order
}

void calibrate_encoder(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){
	/* Calibrates e-zero and encoder nonlinearity */

	if(!cal->started){
			printf("Starting offset cal and linearization\r\n");
			printf("Theta ref, count ref, count error\r\n");
			cal->started = 1;
			cal->start_count = loop_count;
			cal->next_sample_time = T1;
			cal->sample_count = 0;
		}

	cal->time = (float)(loop_count - cal->start_count)*DT;

    if(cal->time < T1){
        // Set voltage angle to zero, wait for rotor position to settle
        cal->theta_ref = 0;//W_CAL*cal->time;
        cal->cal_position.elec_angle = cal->theta_ref;
        controller->i_d_des = I_CAL;
        controller->i_q_des = 0.0f;
        commutate(controller, &cal->cal_position);

    	cal->theta_start = encoder->angle_multiturn[0];
    	cal->next_sample_time = cal->time;
    	return;
    }
    else if (cal->time < T1+2.0f*PI_F*PPAIRS/W_CAL){
    	// rotate voltage vector through one mechanical rotation in the positive direction
		cal->theta_ref += W_CAL*DT;//(cal->time-T1);
		cal->cal_position.elec_angle = cal->theta_ref;
		commutate(controller, &cal->cal_position);

		// sample SAMPLES_PER_PPAIR times per pole-pair
		if(cal->time > cal->next_sample_time){

			int count_ref = cal->theta_ref * (float)ENC_CPR/(2.0f*PI_F*PPAIRS);
			int error = encoder->raw - count_ref;//- encoder->raw;
			cal->error_arr[cal->sample_count] = error + ENC_CPR*(error<0);
			printf("%.3f %d %d\r\n", cal->theta_ref, count_ref, cal->error_arr[cal->sample_count]);

			cal->next_sample_time += 2.0f*PI_F/(W_CAL*SAMPLES_PER_PPAIR);
			if(cal->sample_count == PPAIRS*SAMPLES_PER_PPAIR-1){
				return;
			}
			cal->sample_count++;

		}
		return;
    }
	else if (cal->time < T1+4.0f*PI_F*PPAIRS/W_CAL){
		// rotate voltage vector through one mechanical rotation in the negative direction
		cal->theta_ref -= W_CAL*DT;//(cal->time-T1);
		controller->i_d_des = I_CAL;
		controller->i_q_des = 0.0f;
		cal->cal_position.elec_angle = cal->theta_ref;
		commutate(controller, &cal->cal_position);

		// sample SAMPLES_PER_PPAIR times per pole-pair
		if((cal->time > cal->next_sample_time)&&(cal->sample_count>0)){

			int count_ref = cal->theta_ref * (float)ENC_CPR/(2.0f*PI_F*PPAIRS);
			int error = encoder->raw - count_ref;// - encoder->raw;
			error = error + ENC_CPR*(error<0);
			cal->error_arr[cal->sample_count] = (cal->error_arr[cal->sample_count] + error)/2;
			printf("%.3f %d %d\r\n", cal->theta_ref, count_ref, cal->error_arr[cal->sample_count]);

			cal->sample_count--;
			cal->next_sample_time += 2.0f*PI_F/(W_CAL*SAMPLES_PER_PPAIR);
		}
		return;
    }

    reset_foc(controller);

    // Calculate average offset
    int ezero_mean = 0;
	for(int i = 0; i<((int)PPAIRS*SAMPLES_PER_PPAIR); i++){
		ezero_mean += cal->error_arr[i];
	}
	cal->ezero = ezero_mean/(SAMPLES_PER_PPAIR*PPAIRS);

	// check for valid calibration...ezero mean is in counts, should be less than CPR
	if (cal->ezero < ENC_CPR){
		printf("Valid calibration. Mean elec zero: %d\n\r", cal->ezero);
		cal->valid_cal = 1;

		// Moving average to filter out cogging ripple
		int window = SAMPLES_PER_PPAIR;
		int lut_offset = (ENC_CPR-cal->error_arr[0])*N_LUT/ENC_CPR;
		for(int i = 0; i<N_LUT; i++){
				int moving_avg = 0;
				for(int j = (-window)/2; j<(window)/2; j++){
					int index = i*PPAIRS*SAMPLES_PER_PPAIR/N_LUT + j;
					if(index<0){index += (SAMPLES_PER_PPAIR*PPAIRS);}
					else if(index>(SAMPLES_PER_PPAIR*PPAIRS-1)){index -= (SAMPLES_PER_PPAIR*PPAIRS);}
					moving_avg += cal->error_arr[index];
				}
				moving_avg = moving_avg/window;
				int lut_index = lut_offset + i;
				if(lut_index>(N_LUT-1)){lut_index -= N_LUT;}
				cal->lut_arr[lut_index] = moving_avg - cal->ezero;
				printf("%d  %d\r\n", lut_index, moving_avg - cal->ezero);
			}

	} else {
		printf("Bad calibration, won't save the data. Mean elec zero: %d\n\r", cal->ezero);
		cal->valid_cal = 0;

	}

	cal->started = 0;
	cal->done_cal = 1;

}


void measure_lr(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){
	// TODO: implement this?
}


int check_encoder_init(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal){

	printf("\n\r Checking encoder initialization\n\r");

	float theta_elec_read = 0.0f;
	float theta_elec_err = 0.0f;
	int theta_elec_counts = 0;

	for(int i = 0; i<10000; i++){		// Set voltage angle to zero, wait for rotor position to settle
		cal->theta_ref = PI_F/2.0f;//W_CAL*cal->time;
		cal->cal_position.elec_angle = cal->theta_ref;
		cal->cal_position.elec_velocity = 0;
		controller->i_d_des = I_CAL;
		controller->i_q_des = 0.0f;
		commutate(controller, &cal->cal_position);
	}

	for(int i = 0; i<20000; i++){		// Set voltage angle to zero, wait for rotor position to settle
		cal->theta_ref = 0.0f;//W_CAL*cal->time;
		cal->cal_position.elec_angle = cal->theta_ref;
		cal->cal_position.elec_velocity = 0;
		controller->i_d_des = I_CAL;
		controller->i_q_des = 0.0f;
		commutate(controller, &cal->cal_position);
		if (i==15000) {
			theta_elec_read = encoder->elec_angle;
			theta_elec_counts = encoder->count;
		}
	}

	// how far from elec angle of 0?
	if (theta_elec_read > PI_F) { theta_elec_err = theta_elec_read - 2.0f*PI_F; } // wrap from -PI to PI instead of 0 to 2*PI
	else { theta_elec_err = theta_elec_read; };

	float diff_zeros = ((float)(theta_elec_counts-E_ZERO))*PPAIRS/((float)ENC_CPR);
	int diff_int = diff_zeros;
	diff_zeros = diff_zeros - (float)diff_int;
	diff_zeros = diff_zeros>0.5 ? diff_zeros-1.0 : diff_zeros<-0.5 ? diff_zeros+1.0 : diff_zeros;

	// Print difference and status of initialization
	if ((theta_elec_err < (PI_F/2.0f)) && (theta_elec_err > (-PI_F/2.0f)) ) { // initialization is good
		printf(" Good initialization! \n\r");
		printf(" Angle Error = %.2f, Old Zero = %d, New Zero = %d, Zero Diff (Elec Rots) = %.2f\r\n", theta_elec_err, E_ZERO, theta_elec_counts, diff_zeros);
		encoder->init_status = 1;
	} else { // electrical angle error is larger than 90deg
		printf(" BAD initialization! \n\r");
		printf(" Angle Error = %.2f, Old Zero = %d, New Zero = %d, Zero Diff (Elec Rots) = %.2f\r\n", theta_elec_err, E_ZERO, theta_elec_counts, diff_zeros);
		encoder->init_status = 0;
	}

	// return encoder->elec_angle? encoder->count?
	return theta_elec_counts; //theta_elec_read;

} // end check_encoder_init function









