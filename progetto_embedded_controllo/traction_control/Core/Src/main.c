/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * @author         : Sviluppato per architettura Hard Real-Time STM32F4
 ******************************************************************************
 */

//Ruota 2 = Sinistra
//Ruota 1 = Destra
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bts7960.h"
#include "amt102.h"
#include "mpu6050.h"
#include "pi_controller.h"
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOT_MAX_PWM         4499
//Configurazione Motore 1 (Destro)
#define M1_PWM_TIM           &htim12          //Timer associato a PB15
#define M1_PWM_CH            TIM_CHANNEL_2    //Canale associato a PB15

//Configurazione Motore 2 (Sinistro)
#define M2_PWM_TIM           &htim12          //Timer associato a PB14
#define M2_PWM_CH            TIM_CHANNEL_1    //Canale associato a PB14

#define ENC_PPR_X4           8192.0f
#define ENC1_TIM             &htim3
#define ENC2_TIM             &htim4

#define WHEEL_RADIUS         0.05f   //Raggio della ruota in metri
#define TRACK_WIDTH          0.40f   //Carreggiata in metri
#define SLIP_MAX             0.15f   //Slittamento massimo ammesso (15%)

//Guadagni PI Velocità (Middle Loop)
#define KP_VEL                  2.5f
#define KI_VEL                  15.0f
//Guadagni PI Corrente (Inner Loop)
#define MAX_CURRENT_LIMIT       4.0f   //Corrente massima (A)
#define KP_CURR                 0.0729f
#define KI_CURR                 697.92f
#define MAX_DUTY_CYCLE          100.0f  //Limite massimo PWM (%)
#define ADC_BUFFER_DEPTH        10

#define V_MIN_SLIP           0.1f   //Parametro per evitare singolarità (divisione per zero)
#define MAX_CURRENT_CUT      4.0f    //Taglio massimo di corrente (A) permesso al TC
#define KP_TC                2.5f
#define KI_TC                8.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

BTS7960_Motor_t motor1;
BTS7960_Motor_t motor2;
AMT102_Encoder_t encoder1;
AMT102_Encoder_t encoder2;
mpu6050_t imu1;

//Buffer (Gestiti da DMA e Interrupt)
volatile uint8_t buffer_imu[14];
volatile uint16_t adc_buffer[ADC_BUFFER_DEPTH * 2];
volatile uint8_t rx_byte;
volatile float v_pred = 0.0f;
uint8_t payload_buffer[8];
uint8_t sync_state = 0;
uint8_t payload_idx = 0;

//Struttura Kalman filter
typedef struct {
	float v;
	float P;
	float R;
	float Q;
} Kalman_Velocity_t;

Kalman_Velocity_t kf_vel = { .v = 0.0f, .P = 1.0f, .Q = 0.005f, .R = 0.01 };

//Controllori PI
PI_Controller_t pi_vel_m1;
PI_Controller_t pi_vel_m2;
PI_Controller_t pi_curr_m1;
PI_Controller_t pi_curr_m2;
PI_Controller_t pi_tc_m1;
PI_Controller_t pi_tc_m2;

//Variabili di controllo
volatile float target_velocita_condivisa = 0.0f; //Target da UART
volatile float target_omega_condivisa = 0.0f;	//Target angolare omega da UART
volatile float target_corrente_m1 = 0.0f; //Target dal Middle Loop all'Inner Loop
volatile float target_corrente_m2 = 0.0f;
volatile float target_velocita_m1 = 0.0f;
volatile float target_velocita_m2 = 0.0f;

volatile float accel_x_condivisa = 0.0f;
volatile float accel_z_condivisa = 0.0f;
volatile float gyro_y_condivisa = 0.0f;
volatile float gyro_z_condivisa = 0.0f;

volatile float corrente_m1 = 0.0f;
volatile float corrente_m2 = 0.0f;

volatile uint8_t flag_dma_imu_pronto = 0;
volatile uint8_t flag_esegui_outer_loop = 0;
volatile uint8_t i2c_need_reset = 0;
volatile uint8_t system_safe_state = 0;          //0 = OK, 1 = AVARIA IMU
volatile uint8_t imu_timeout_counter = 0;
volatile uint8_t i2c_is_resetting = 0;
volatile float v_misurata_media = 0.0f;
volatile float duty_m1 = 0.0f;
volatile float duty_m2 = 0.0f;
volatile float err_vel_m1 = 0.0f;
volatile float err_vel_m2 = 0.0f;
volatile float err_curr_m1 = 0.0f;
volatile float err_curr_m2 = 0.0f;
volatile float sum_accel_x = 0.0f;
volatile float sum_accel_z = 0.0f;
volatile float sum_gyro_y = 0.0f;
volatile float sum_gyro_z = 0.0f;
volatile uint8_t imu_read_count = 0;
volatile float v_ruota1 = 0.0f;
volatile float v_ruota2 = 0.0f;

volatile float slip_m1 = 0.0f;
volatile float slip_m2 = 0.0f;
volatile float target_cinematico_m1 = 0.0f;
volatile float target_cinematico_m2 = 0.0f;

volatile float error_m1 = 0.0f;
volatile float error_m2 = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t is_first_run = 1;
void CurrentSensing_Update(float *corrente_m1_out, float *corrente_m2_out);
void StateEstimation_Update(void);
void TractionControl_Apply(float v_telaio, float gyro_z, float *i_cut_m1,
		float *i_cut_m2);
void System_Enter_SafeMode(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_ADC1_Init();
	MX_TIM6_Init();
	MX_TIM12_Init();
	MX_I2C3_Init();
	MX_TIM5_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	//Setup motori
	motor1.htim = M1_PWM_TIM;
	motor1.pwm_channel = M1_PWM_CH;
	motor1.dir_port = DIR_L_GPIO_Port;
	motor1.dir_pin = DIR_L_Pin;
	motor1.max_pwm_value = MOT_MAX_PWM;

	motor2.htim = M2_PWM_TIM;
	motor2.pwm_channel = M2_PWM_CH;
	motor2.dir_port = DIR_R_GPIO_Port;
	motor2.dir_pin = DIR_R_Pin;
	motor2.max_pwm_value = MOT_MAX_PWM;

	BTS7960_Init(&motor1);
	BTS7960_Init(&motor2);

	//Setup encoder
	encoder1.htim = ENC1_TIM;
	encoder1.ppr_x4 = ENC_PPR_X4;
	AMT102_Init(&encoder1);

	encoder2.htim = ENC2_TIM;
	encoder2.ppr_x4 = ENC_PPR_X4;
	AMT102_Init(&encoder2);
	//Inizializzazione PI velocità
	PI_Init(&pi_vel_m1, KP_VEL, KI_VEL, 0.010f, MAX_CURRENT_LIMIT);
	PI_Init(&pi_vel_m2, KP_VEL, KI_VEL, 0.010f, MAX_CURRENT_LIMIT);

	//Inizializzazione PI corrente
	PI_Init(&pi_curr_m1, KP_CURR, KI_CURR, 0.001f, MAX_DUTY_CYCLE);
	PI_Init(&pi_curr_m2, KP_CURR, KI_CURR, 0.001f, MAX_DUTY_CYCLE);

	PI_Init(&pi_tc_m1, KP_TC, KI_TC, 0.010f, MAX_CURRENT_CUT);
	PI_Init(&pi_tc_m2, KP_TC, KI_TC, 0.010f, MAX_CURRENT_CUT);

	//Setup IMU
	if (imu_init(&imu1, &hi2c3, IMU_ADDR) != HAL_OK) {
		Error_Handler();
	}
	imu_calibrate(&imu1);

	//Avvio periferiche e DMA
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, ADC_BUFFER_DEPTH * 2);
	HAL_Delay(20);
	HAL_TIM_Base_Start_IT(&htim5);
	__HAL_TIM_SET_COUNTER(&htim5, 0);

	HAL_TIM_Base_Start_IT(&htim6);
	__HAL_TIM_SET_COUNTER(&htim6, 0);

	HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx_byte, 1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		//Gestione I2C
		if (i2c_need_reset == 1) {
			i2c_is_resetting = 1;
			HAL_I2C_DeInit(&hi2c3);
			MX_I2C3_Init();
			imu_init(&imu1, &hi2c3, IMU_ADDR);
			i2c_need_reset = 0;
			i2c_is_resetting = 0;
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

//Callback ricezione dati UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		switch (sync_state) {
		case 0: //Attesa header 1 (0xAA)
			if (rx_byte == 0xAA)
				sync_state = 1;
			break;
		case 1: //Attesa header 2 (0x55)
			if (rx_byte == 0x55) {
				sync_state = 2;
				payload_idx = 0;
			} else if (rx_byte != 0xAA) {
				sync_state = 0;
			}
			break;
		case 2: //Lettura payload (8 byte)
			payload_buffer[payload_idx++] = rx_byte;
			if (payload_idx >= 8)
				sync_state = 3;
			break;
		case 3: //Lettura checksum e validazione
		{
			uint8_t calc_chk = 0;
			for (int i = 0; i < 8; i++) {
				calc_chk ^= payload_buffer[i];
			}
			//Applica i target solo se il pacchetto è integro
			if (calc_chk == rx_byte) {
				float nuova_velocita, nuova_omega;
				memcpy(&nuova_velocita, &payload_buffer[0], 4);
				memcpy(&nuova_omega, &payload_buffer[4], 4);
				target_velocita_condivisa = nuova_velocita;
				target_omega_condivisa = nuova_omega;
			}
			sync_state = 0; //Attesa prossimo pacchetto
		}
			break;
		}
		//Riattiva l'ascolto per il byte successivo
		HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx_byte, 1);
	}
}

//Callback ricezione dati I2C
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == I2C3) {
		int16_t raw_accel_x = (int16_t) (buffer_imu[0] << 8 | buffer_imu[1]);
		int16_t raw_accel_z = (int16_t) (buffer_imu[4] << 8 | buffer_imu[5]);
		int16_t raw_gyro_y = (int16_t) (buffer_imu[10] << 8 | buffer_imu[11]);
		int16_t raw_gyro_z = (int16_t) (buffer_imu[12] << 8 | buffer_imu[13]);
		//Conversione in m/s^2 e rad/s
		float accel_x_depurata = ((float) (raw_accel_x - imu1.imu_bias.accel_x)
				/ 16384.0f) * 9.81f;
		float accel_z_depurata = ((float) (raw_accel_z - imu1.imu_bias.accel_z)
				/ 16384.0f) * 9.81f;
		float gyro_y_depurato = ((float) (raw_gyro_y - imu1.imu_bias.gyro_y)
				/ 131.0f) * (3.14159265f / 180.0f);
		float gyro_z_depurato = ((float) (raw_gyro_z - imu1.imu_bias.gyro_z)
				/ 131.0f) * (3.14159265f / 180.0f);

		sum_accel_x += accel_x_depurata;
		sum_accel_z += accel_z_depurata;
		sum_gyro_y += gyro_y_depurato;
		sum_gyro_z += gyro_z_depurato;
		imu_read_count++;
		flag_dma_imu_pronto = 1;
	}
}

void CurrentSensing_Update(float *corrente_m1_out, float *corrente_m2_out) {
	uint32_t somma_m1 = 0;
	uint32_t somma_m2 = 0;

	for (uint16_t i = 0; i < ADC_BUFFER_DEPTH; i++) {
		somma_m1 += adc_buffer[i * 2];
		somma_m2 += adc_buffer[(i * 2) + 1];
	}

	float raw_adc_m1 = (float) somma_m1 / (float) ADC_BUFFER_DEPTH;
	float raw_adc_m2 = (float) somma_m2 / (float) ADC_BUFFER_DEPTH;

	float magnitudine_m1 = ((raw_adc_m1 / 4095.0f) * 3.3f) * 10.0f;
	float magnitudine_m2 = ((raw_adc_m2 / 4095.0f) * 3.3f) * 10.0f;

	*corrente_m1_out = (duty_m1 < 0.0f) ? -magnitudine_m1 : magnitudine_m1;
	*corrente_m2_out = (duty_m2 < 0.0f) ? -magnitudine_m2 : magnitudine_m2;
}

void StateEstimation_Update(void) {
	//Prelievo dati e gestione concorrenza
	__disable_irq();
	uint8_t count = imu_read_count;
	float ax_sum = sum_accel_x;
	float az_sum = sum_accel_z;
	float gy_sum = sum_gyro_y;
	float gz_sum = sum_gyro_z;
	//Azzeramento accumulatori per la prossima finestra da 10ms
	sum_accel_x = 0.0f;
	sum_accel_z = 0.0f;
	sum_gyro_y = 0.0f;
	sum_gyro_z = 0.0f;
	imu_read_count = 0;
	__enable_irq();
	//Calcolo media e Kalman
	if (count > 0) {
		flag_dma_imu_pronto = 0;
		//Calcolo della media
		accel_x_condivisa = ax_sum / count;
		accel_z_condivisa = az_sum / count;
		gyro_y_condivisa = gy_sum / count;
		gyro_z_condivisa = gz_sum / count;
		//Predizione e correzione Kalman con il segnale mediato
		float v_pred = kf_vel.v + accel_x_condivisa * 0.010f;
		float P_pred = kf_vel.P + kf_vel.Q;
		float K = P_pred / (P_pred + kf_vel.R);
		kf_vel.v = v_pred + K * (v_misurata_media - v_pred);
		kf_vel.P = (1.0f - K) * P_pred;
	} else {
		//Nessun dato letto in questa finestra (accelerazione nulla)
		float v_pred = kf_vel.v;
		float P_pred = kf_vel.P + kf_vel.Q;
		float K = P_pred / (P_pred + kf_vel.R);
		kf_vel.v = v_pred + K * (v_misurata_media - v_pred);
		kf_vel.P = (1.0f - K) * P_pred;
	}
}

void TractionControl_Apply(float v_telaio, float gyro_z, float *i_cut_m1,
		float *i_cut_m2) {
	float v_telaio_m1 = v_telaio - (gyro_z * TRACK_WIDTH * 0.5f);
	float v_telaio_m2 = v_telaio + (gyro_z * TRACK_WIDTH * 0.5f);

	//Elaborazione ruota destra
	float max_vel_m1 = fmaxf(fabsf(v_telaio_m1), fabsf(v_ruota1));
	float den_m1 = fmaxf(max_vel_m1, V_MIN_SLIP);
	float slip_ratio_m1 = fabsf(v_ruota1 - v_telaio_m1) / den_m1;

	uint8_t in_deadband_m1 = (fabsf(target_cinematico_m1) < 0.05f)
			&& (fabsf(v_telaio_m1) < 0.05f);
	if (slip_ratio_m1 == 0.0f || in_deadband_m1) {
		if (pi_tc_m1.integral_error > 0.0f) {
			pi_tc_m1.integral_error = 0.0f;
		}
		slip_ratio_m1 = 0.0f;
	}
	//Errore positivo = stiamo slittando oltre il limite (attivazione flag)
	error_m1 = slip_ratio_m1 - SLIP_MAX;
	slip_m1 = (error_m1 > 0.0f) ? 1.0f : 0.0f;

	//Hold dell'integratore (blocca se non slittiamo e la memoria è vuota)
	uint8_t hold_m1 =
			(error_m1 < 0.0f && pi_tc_m1.integral_error <= 0.0f) ? 1 : 0;

	//Calcolo taglio di corrente
	float cut_1 = PI_Compute(&pi_tc_m1, 0.0f, -error_m1, hold_m1);
	*i_cut_m1 = (cut_1 < 0.0f) ? 0.0f : cut_1;

	//Elaborazione ruota sinistra
	float max_vel_m2 = fmaxf(fabsf(v_telaio_m2), fabsf(v_ruota2));
	float den_m2 = fmaxf(max_vel_m2, V_MIN_SLIP);
	float slip_ratio_m2 = fabsf(v_ruota2 - v_telaio_m2) / den_m2;

	uint8_t in_deadband_m2 = (fabsf(target_cinematico_m2) < 0.05f)
			&& (fabsf(v_telaio_m2) < 0.05f);
	if (slip_ratio_m2 == 0.0f || in_deadband_m2) {
		if (pi_tc_m2.integral_error > 0.0f) {
			pi_tc_m2.integral_error = 0.0f;
		}
		slip_ratio_m2 = 0.0f;
	}

	error_m2 = slip_ratio_m2 - SLIP_MAX;
	slip_m2 = (error_m2 > 0.0f) ? 1.0f : 0.0f;

	uint8_t hold_m2 =
			(error_m2 < 0.0f && pi_tc_m2.integral_error <= 0.0f) ? 1 : 0;

	float cut_2 = PI_Compute(&pi_tc_m2, 0.0f, -error_m2, hold_m2);
	*i_cut_m2 = (cut_2 < 0.0f) ? 0.0f : cut_2;
}

void System_Enter_SafeMode(void) {
	target_corrente_m1 = 0.0f;
	target_corrente_m2 = 0.0f;
	target_velocita_m1 = 0.0f;
	target_velocita_m2 = 0.0f;
	kf_vel.v = 0.0f;
	is_first_run = 1;
	//Reset degli integratori PI
	PI_Reset(&pi_vel_m1);
	PI_Reset(&pi_vel_m2);
	PI_Reset(&pi_curr_m1);
	PI_Reset(&pi_curr_m2);
	PI_Reset(&pi_tc_m1);
	PI_Reset(&pi_tc_m2);
	slip_m1 = 0.0f;
	slip_m2 = 0.0f;
	__disable_irq();
	sum_accel_x = 0.0f;
	sum_accel_z = 0.0f;
	sum_gyro_y = 0.0f;
	sum_gyro_z = 0.0f;
	imu_read_count = 0;
	__enable_irq();
}

//Callback timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	//Controllo corrente a 1000 Hz
	if (htim->Instance == TIM5) {

		//Lettura sensori
		CurrentSensing_Update((float*) &corrente_m1, (float*) &corrente_m2);

		err_curr_m1 = target_corrente_m1 - corrente_m1;
		err_curr_m2 = target_corrente_m2 - corrente_m2;

		//Calcolo Duty Cycle tramite i PI
		duty_m1 = PI_Compute(&pi_curr_m1, target_corrente_m1, corrente_m1, 0);
		duty_m2 = PI_Compute(&pi_curr_m2, target_corrente_m2, corrente_m2, 0);

		//Attuazione hardware
		BTS7960_SetDutyCycle(&motor1, duty_m1);
		BTS7960_SetDutyCycle(&motor2, duty_m2);

		//Gestione IMU in DMA
		if (i2c_is_resetting == 0) {
			HAL_I2C_Mem_Read_DMA(&hi2c3, IMU_ADDR, ACCEL_XOUT_H_REG,
			I2C_MEMADD_SIZE_8BIT, buffer_imu, 14);
		}
	}

	//Controllo velocità a 100 Hz
	else if (htim->Instance == TIM6) {
		if (flag_dma_imu_pronto == 1) {
			imu_timeout_counter = 0;
			system_safe_state = 0;
		} else {
			imu_timeout_counter++;
			if (imu_timeout_counter > 5) {
				system_safe_state = 1;
				i2c_need_reset = 1;
			}
		}
		//Aggiornamento encoder
		AMT102_Update(&encoder1, 0.010f);
		AMT102_Update(&encoder2, 0.010f);
		v_ruota1 = -1.0f * encoder1.velocity_rad_s * WHEEL_RADIUS;
		v_ruota2 = encoder2.velocity_rad_s * WHEEL_RADIUS;
		v_misurata_media = (v_ruota1 + v_ruota2) / 2.0f;

		//Inizializzazione stato al primo avvio
		if (is_first_run == 1) {
			kf_vel.v = (v_ruota1 + v_ruota2) / 2.0f;
			is_first_run = 0;
		}

		if (system_safe_state == 0) {
			target_cinematico_m1 = target_velocita_condivisa
					- (target_omega_condivisa * TRACK_WIDTH * 0.5f);
			target_cinematico_m2 = target_velocita_condivisa
					+ (target_omega_condivisa * TRACK_WIDTH * 0.5f);
			target_velocita_m1 = target_cinematico_m1;
			target_velocita_m2 = target_cinematico_m2;

			//Stima di stato
			StateEstimation_Update();

			//Traction Control
			float i_cut_m1 = 0.0f;
			float i_cut_m2 = 0.0f;
			TractionControl_Apply(kf_vel.v, gyro_z_condivisa, &i_cut_m1,
					&i_cut_m2);

			err_vel_m1 = target_velocita_m1 - v_ruota1;
			err_vel_m2 = target_velocita_m2 - v_ruota2;

			float deadband_threshold = 0.05f;

			if (target_velocita_condivisa == 0.0f
					&& target_omega_condivisa == 0.0f
					&& fabsf(v_ruota1) < deadband_threshold
					&& fabsf(v_ruota2) < deadband_threshold) {

				target_corrente_m1 = 0.0f;
				target_corrente_m2 = 0.0f;
				PI_Reset(&pi_vel_m1);
				PI_Reset(&pi_vel_m2);
			} else {
				//Calcolo corrente nominale
				float nominal_curr_m1 = PI_Compute(&pi_vel_m1,
						target_velocita_m1, v_ruota1, (i_cut_m1 > 0.01f));
				float nominal_curr_m2 = PI_Compute(&pi_vel_m2,
						target_velocita_m2, v_ruota2, (i_cut_m2 > 0.01f));
				//Sottrazione del taglio in base al verso di marcia
				if (nominal_curr_m1 > 0.0f) {
					target_corrente_m1 = fmaxf(0.0f,
							nominal_curr_m1 - i_cut_m1);
				} else {
					target_corrente_m1 = fminf(0.0f,
							nominal_curr_m1 + i_cut_m1);
				}

				if (nominal_curr_m2 > 0.0f) {
					target_corrente_m2 = fmaxf(0.0f,
							nominal_curr_m2 - i_cut_m2);
				} else {
					target_corrente_m2 = fminf(0.0f,
							nominal_curr_m2 + i_cut_m2);
				}
			}

		} else {
			System_Enter_SafeMode();
		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
