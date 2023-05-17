/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


/// high-bandwidth 3-phase motor control for robots
/// Written by Ben Katz, with much inspiration from Bayley Wang, Nick Kirkby, Shane Colton, David Otten, and others
/// Hardware documentation can be found at build-its.blogspot.com

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "structs.h"
#include <stdio.h>
#include <string.h>

#include "stm32f4xx_flash.h"
#include "flash_writer.h"
#include "position_sensor.h"
#include "preference_writer.h"
#include "hw_config.h"
#include "user_config.h"
#include "fsm.h"
#include "drv8323.h"
#include "foc.h"
#include "math_ops.h"
#include "calibration.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define VERSION_NUM 3.142f // incremented to 3.01 by Elijah for CAN retransmission 5/6/23


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Flash Registers */
float __float_reg[64];
int __int_reg[256];
PreferenceWriter prefs;

int count = 0;

/* Structs for control, etc */

ControllerStruct controller;
ObserverStruct observer;
COMStruct com;
FSMStruct state;
EncoderStruct comm_encoder;
DRVStruct drv;
CalStruct comm_encoder_cal;
CANTxMessage can_tx;
CANRxMessage can_rx;

//CAN ACTIAVTION FLAG
int CAN_ACTIVE = 0;

/* init but don't allocate calibration arrays */
int *error_array = NULL;
int *lut_array = NULL;

uint8_t Serial2RxBuffer[1];

int loop_time;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_Delay(500);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2); // enable this to use delay_us() function
  HAL_TIM_Base_Start(&htim3); // enable this to time functions in us

  /* Load settings from flash */
  preference_writer_init(&prefs, 6);
  preference_writer_load(prefs);

  /* Sanitize configs in case flash is empty*/
  if(PHASE_ORDER==-1){PHASE_ORDER = 0;};
  if(CAN_ID==-1){CAN_ID = 1;}
  if(CAN_MASTER==-1){CAN_MASTER = 0;}
  if(CAN_TIMEOUT==-1){CAN_TIMEOUT = 1000;}
  if(EN_ENC_FILTER ==-1){EN_ENC_FILTER = 0;}
  if(EN_ENC_LINEARIZE ==-1){EN_ENC_LINEARIZE = 0;}
  if(E_ZERO==-1){E_ZERO = 0;}
  if(M_ZERO==-1){M_ZERO = 0;}

  if(isnan(I_BW) || I_BW==-1){I_BW = 1000;}
  if(isnan(I_MAX) || I_MAX ==-1){I_MAX=40;}
  if(isnan(I_MAX_CONT) || I_MAX_CONT==-1){I_MAX_CONT = 14.0f;}
  if(isnan(I_CAL)||I_CAL==-1){I_CAL = 5.0f;}
  if(isnan(I_FW_MAX) || I_FW_MAX ==-1){I_FW_MAX=0;}

//  if(isnan(K_SCALE) || K_SCALE==-1){K_SCALE = 0.000133f;}
//  if(isnan(KI_D) || KI_D==-1){KI_D = 0.0373f;}
//  if(isnan(KI_Q) || KI_Q ==-1){KI_Q = 0.0373f;}

  if(isnan(PPAIRS) || PPAIRS==-1){PPAIRS = 21.0f;}
  if(isnan(GR) || GR==-1){GR = 6.0f;}
  if(isnan(KT_OUT) || KT_OUT==-1){KT_OUT = 1.0f;}
  if(isnan(L_D) || L_D==-1){L_D = 0.000003f;}
  if(isnan(L_Q) || L_Q==-1){L_Q = 0.000003f;}
  if(isnan(R_PHASE) || R_PHASE==-1){R_PHASE = 0.433f;}
  if(isnan(R_NOMINAL) || R_NOMINAL==-1){R_NOMINAL = 0.0f;}

  if(isnan(R_TH) || R_TH==-1){R_TH = 1.25f;}
  if(isnan(C_TH) || C_TH==-1){C_TH = 0.0f;}
  if(isnan(INV_M_TH) || INV_M_TH==-1){INV_M_TH = 0.02825f;}
  if(isnan(T_AMBIENT) || T_AMBIENT==-1){T_AMBIENT = 25.0f;}
  if(isnan(TEMP_MAX) || TEMP_MAX==-1){TEMP_MAX = 125.0f;}

  if(isnan(P_MIN) || P_MIN==-1){P_MIN = -12.5f;}
  if(isnan(P_MAX) || P_MAX==-1){P_MAX = 12.5f;}
  if(isnan(V_MIN) || V_MIN==-1){V_MIN = -65.0f;}
  if(isnan(V_MAX) || V_MAX==-1){V_MAX = 65.0f;}
  if(isnan(KP_MAX) || KP_MAX==-1){KP_MAX = 500.0f;}
  if(isnan(KD_MAX) || KD_MAX==-1){KD_MAX = 10.0f;}
  if(isnan(T_MIN) || T_MIN==-1){T_MIN = -72.0f;}
  if(isnan(T_MAX) || T_MAX==-1){T_MAX = 72.0f;}

  printf("\r\nFirmware Version Number: %.3f\r\n", VERSION_NUM);

  /* Controller Setup */
  if(PHASE_ORDER){							// Timer channel to phase mapping

  }
  else{

  }

  init_controller_params(&controller);

  /* calibration "encoder" zeroing */
  memset(&comm_encoder_cal.cal_position, 0, sizeof(EncoderStruct));

  /* commutation encoder setup */
  comm_encoder.m_zero = M_ZERO;
  comm_encoder.e_zero = E_ZERO;
  comm_encoder.ppairs = PPAIRS;
  ps_warmup(&comm_encoder, 100);			// clear the noisy data when the encoder first turns on

  memcpy(&comm_encoder.offset_lut, &ENCODER_LUT, sizeof(comm_encoder.offset_lut));	// Copy the linearization lookup table
  //for(int i = 0; i<128; i++){printf("%d\r\n", comm_encoder.offset_lut[i]);}

  /* Turn on ADCs */
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc3);

  /* DRV8323 setup */
  HAL_GPIO_WritePin(DRV_CS, GPIO_PIN_SET ); 	// CS high
  HAL_GPIO_WritePin(ENABLE_PIN, GPIO_PIN_SET );
  HAL_Delay(1);
  drv_write_DCR(drv, 0x0, DIS_GDF_DIS, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1); //  TODO: enable gate drive fault?
  HAL_Delay(1);
  drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN_40, DIS_SEN_EN, 0x1, 0x1, 0x1, SEN_LVL_1_0); // calibrate shunt amplifiers
  HAL_Delay(1);
  zero_current(&controller); // moved this between the two drv_write_CSACR calls to match mbed fw version
  HAL_Delay(1);
  drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN_40, DIS_SEN_DIS, 0x0, 0x0, 0x0, SEN_LVL_1_0); // TODO: enable sensing of overcurrent fault?
  HAL_Delay(1);
  drv_write_OCPCR(drv, TRETRY_50US, DEADTIME_50NS, OCP_NONE, OCP_DEG_8US, VDS_LVL_1_88); // TODO: reduce VDS level and add OCP_RETRY?
  HAL_Delay(1);
  drv_disable_gd(drv);
  HAL_Delay(1);
  //drv_enable_gd(drv);
  printf("ADC A OFFSET: %d     ADC B OFFSET: %d\r\n", controller.adc_a_offset, controller.adc_b_offset);

  /* Turn on PWM */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  /* CAN setup */
  can_rx_init(&can_rx);
  can_tx_init(&can_tx);
  HAL_CAN_Start(&CAN_H); //start CAN
  //__HAL_CAN_ENABLE_IT(&CAN_H, CAN_IT_RX_FIFO0_MSG_PENDING); // Start can interrupt

  /* Set Interrupt Priorities */
  HAL_NVIC_SetPriority(PWM_ISR, 0x01,0x01); // commutation > communication
//  HAL_NVIC_SetPriority(CAN_ISR, 0x02, 0x02);

  /* Turn on interrupts */
  HAL_UART_Receive_IT(&huart2, (uint8_t *)Serial2RxBuffer, 1);
  HAL_TIM_Base_Start_IT(&htim1); // start main control interrupt

  // Check encoder initialization here
  int new_offset = 0;
  ps_sample(&comm_encoder, 0.001);
  HAL_GPIO_WritePin(LED, GPIO_PIN_SET );
  drv_enable_gd(drv);
  new_offset = check_encoder_init(&comm_encoder, &controller, &comm_encoder_cal);             // status = 1 is good
  HAL_Delay(100);
  drv_disable_gd(drv);
  HAL_GPIO_WritePin(LED, GPIO_PIN_RESET );

  E_ZERO = new_offset;
  comm_encoder.e_zero = E_ZERO;
  printf(" Position Sensor Electrical Offset: %d\n\r", E_ZERO);

  // initialize filter here for position sensor
  HAL_Delay(100);
//  ps_filter_init(&comm_encoder);

  // average 100 samples for filter init values
  int num_filt_init = 100;
  float filt_prev_mech_temp = 0.0f;
  float filt_prev_elec_temp = 0.0f;
  for (int i=0; i<num_filt_init; i++){
	  filt_prev_mech_temp += (1.0/(float)num_filt_init)*comm_encoder.angle_multiturn[0];
	  filt_prev_elec_temp += (1.0/(float)num_filt_init)*comm_encoder.elec_angle;
	  HAL_Delay(1); // need to wait for some time to get a new position sample
  }
  comm_encoder.filt_prev_mech = filt_prev_mech_temp;
  comm_encoder.filt_prev_elec = filt_prev_elec_temp;

  if (EN_ENC_FILTER == 1){
	  comm_encoder.filt_enable = 1;
  }
  // reset encoder sample count
  comm_encoder.first_sample = 0;

  /* Start the FSM */
  state.state = MENU_MODE;
  state.next_state = MENU_MODE;
  state.ready = 1;
  fsm_enter_state(&state);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_Delay(100);
//	  printf("%d\n\r", loop_time);
	  drv_check_faults(drv, &state);
	 // if(state.state==MOTOR_MODE){


	  	  //printf("%.2f %.2f %.2f %.2f %.2f\r\n", controller.p_des, controller.v_des, controller.kp, controller.kd, controller.t_ff);
	  //}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
