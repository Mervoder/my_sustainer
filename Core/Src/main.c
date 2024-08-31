/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwgps/lwgps.h"
#include "LSM6DSLTR.h"
#include "./BME280/bme280.h"
#include "W25Qxx.h"
#include <math.h>
#include "stdio.h"
#include "string.h"
#include "kalman.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LORA_TX_BUFFER_SIZE 75
#define EGU_RX_BUFFER_SIZE 34
#define RX_BUFFER_SIZE 128
#define DEVICE_ID 2

#define GyroAlfa	0.2 //0.01
#define AccAlfa		0.4 // 0.1
#define HP_alpha 	0.92f // 0.8 en iyi
#define LP_alpha 	0.55f // 0.4 en iyi
#define beta		0.85

#define HEADER 		0x31

#define CMD_SET_REG 0xC0 // COMMAND FOR SETTING REGISTER
#define CMD_READ_REG 0xC1 // COMMAND FOR READING REGISTER
#define REG_ADD_H 0x0 // DEVICE ADDRESS HIGH BYTE
#define REG_ADD_L 0x1 // DEVICE ADDRESS LOW BYTE
#define REG0 0x2 // UART CONFIGURATION REGISTER
#define REG1 0x3 // RF CONFIGURATION REGISTER
#define REG2 0x4 // CHANNEL CONTROL
#define REG3 0x5 // TRANSMISSION PARAMETER CONTROL
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
uint8_t Lora_Tx_Buffer[LORA_TX_BUFFER_SIZE];

uint8_t EGU_RX_BUFFER[EGU_RX_BUFFER_SIZE],
		rx_buffer_gps[RX_BUFFER_SIZE]	;

uint8_t rx_data_EGU=0 ,
		rx_data_gps=0 ;

uint8_t rx_index_EGU=0,
		rx_index_gps=0;

uint8_t flag_lora ,
		flag_megu ,
		flag_sensor_imu ,
		flag_sensor_barometre,
		flag_counter,
		flag_median,
		flag_new_imu_data,
		flag_new_barometre_data,
		flag_adc,
		flag_adc_cnt ,
		flag_rampa_altitude,
		flag_flash,
		flag_flash_200ms,
		flag_kontrol_5x;

const uint8_t EGU_durum_sorgusu[5]={0x54,0x52,0x35,0x0D,0x0A};
const uint8_t EGU_motor_atesleme[5]={0x54,0x52,0x32,0x0D,0x0A};


uint8_t magnetic_switch , button_state , battery;

uint8_t flash_altitude[1024]={0},
		flash_accX[1024]={0},
		flash_accZ[1024]={0},
		flash_accY[1024]={0},
		flash_gyroX[1024]={0},
		flash_gyroY[1024]={0},
		flash_gyroZ[1024]={0};

float real_pitch, real_roll , toplam_pitch,toplam_roll , toplam_accX , toplam_accY , toplam_accZ, toplam_gX,toplam_gY,toplam_gZ , rampa_accel,
toplam_normal,real_normal, x_max;
float offset_x,offset_y,offset_z, gyroX_prev, gyroY_prev,gyroZ_prev,
accX_prev, accY_prev,accZ_prev;
float filtered_gyroX ,filtered_gyroY,filtered_gyroZ, filtered_accX,
filtered_accY,filtered_accZ;
float filtered_gyro[3] , filtered_gyro_HP[3];
float filtered_acc_LP[3] , filtered_acc[3];
float gravity_normal_angle , prev_time1;

float gyroX_HP_prev = 0.0f, gyroY_HP_prev = 0.0f, gyroZ_HP_prev = 0.0f;
float filtered_gyro_HP_X = 0.0f, filtered_gyro_HP_Y = 0.0f, filtered_gyro_HP_Z = 0.0f;

float gyroX_LP_prev = 0.0f, gyroY_LP_prev = 0.0f, gyroZ_LP_prev = 0.0f , filtered_gyro_LP[3];

float pressure , temperature , humidity , altitude, altitude_kalman ,altitude_max, prev_alt , speed,speed_max, offset_altitude;
float adc_pil_val , adc;
int time,i;
uint32_t timer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void imu_offset(void);
void Altitude_Offset();
void user_delay_ms(uint32_t period);
void imu_filter_calculate();
void union_converter();
void EGU_Buff_Load(void);
void Flash_buff_fill();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t E220_read_register(uint8_t reg);
int8_t E220_write_register(uint8_t reg,uint8_t parameter);

float BME280_Get_Altitude(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef union{
  float fVal;
  unsigned char array[4];
}float2unit8;
float2unit8 conv;

enum ZORLU2024
{
	RAMPA,UCUS_BASLADI,KADEMEAYRILDIMI,AYRILDI,APOGEE,SUSTAINER_ANA,FINISH
};
enum ZORLU2024 SUSTAINER;



lwgps_t gps;

KalmanFilter kf , na , pa , ax,ay,az,gx,gy,gz;

LSM6DSLTR Lsm_Sensor;

struct bme280_dev dev;
struct bme280_data comp_data;

int8_t rslt=0,receive_data;



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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
  HAL_Delay(100);
  receive_data =E220_write_register(0x2, 0x62);
  HAL_Delay(100);
  HAL_Delay(100);
   receive_data =E220_write_register(0x2, 0x64);
   HAL_Delay(100);
  receive_data =E220_write_register(0x3, 0x40);
  HAL_Delay(100);
  receive_data =E220_write_register(0x4, 0x10); // ch
  HAL_Delay(100);
  receive_data =E220_write_register(0x5, 0x40);//40
  HAL_Delay(100);
  receive_data =E220_write_register(0x6, 0x00);
  HAL_Delay(100);
  receive_data =E220_write_register(0x7, 0x00);
  HAL_Delay(100);
  receive_data =E220_write_register(0, 0x02); // h 0x06
  HAL_Delay(100);

  receive_data =E220_write_register(0x1, 0x03); // low 0x03
  HAL_Delay(200);

  receive_data = E220_read_register(0);
  HAL_Delay(100);
  receive_data = E220_read_register(1);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);//m0
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET); //m1
  HAL_Delay(100);

  KalmanFilter_Init(&ax, 0.2, 2, toplam_accX);
  KalmanFilter_Init(&ay, 0.2, 2, toplam_accY);
  KalmanFilter_Init(&az, 0.2, 2, toplam_accZ);
  KalmanFilter_Init(&gx, 0.2, 2, toplam_gX);
  KalmanFilter_Init(&gy, 0.2, 2, toplam_gY);
  KalmanFilter_Init(&gz, 0.2, 2, toplam_gZ);
  KalmanFilter_Init(&kf, 0.2, 1, toplam_gZ);

  lwgps_init(&gps);

  LSM6DSLTR_Init();

  dev.dev_id = BME280_I2C_ADDR_PRIM;
  dev.intf = BME280_I2C_INTF;
  dev.read = user_i2c_read;
  dev.write = user_i2c_write;
  dev.delay_ms = user_delay_ms;
  rslt = bme280_init(&dev);

  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  dev.settings.osr_p = BME280_OVERSAMPLING_4X;
  dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  dev.settings.filter = BME280_FILTER_COEFF_16;
  rslt = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev);
  rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

  imu_offset();
  Altitude_Offset();

  Lora_Tx_Buffer[0]=0x06;
  Lora_Tx_Buffer[1]=0x03; // 2A
  Lora_Tx_Buffer[2]=0x10; // 10
  Lora_Tx_Buffer[3]=DEVICE_ID;

  Lora_Tx_Buffer[50]=HEADER;// v4mod
  Lora_Tx_Buffer[74]='\n';

 // Buzzer_PlayStartSound();
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  HAL_ADC_Start_IT(&hadc1);

  HAL_UART_Receive_IT(&huart2, &rx_data_gps, 1);
  HAL_UART_Receive_IT(&huart6, &rx_data_EGU, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
/*************************************************************************************/
	  if(flag_sensor_imu ==1)
	  {
		  LSM6DSLTR_Read_Accel_Data(&Lsm_Sensor);
		  LSM6DSLTR_Read_Gyro_Data(&Lsm_Sensor);

		  toplam_accX += KalmanFilter_Update(&ax,Lsm_Sensor.Accel_X);
		  toplam_accY += KalmanFilter_Update(&ay,Lsm_Sensor.Accel_Y );
		  toplam_accZ += KalmanFilter_Update(&az,Lsm_Sensor.Accel_Z );
		  toplam_gX +=  KalmanFilter_Update(&gx,Lsm_Sensor.Gyro_X-offset_x  );
		  toplam_gY +=  KalmanFilter_Update(&gy,Lsm_Sensor.Gyro_Y -offset_y );
		  toplam_gZ +=  KalmanFilter_Update(&gz,Lsm_Sensor.Gyro_Z -offset_z );

		  flag_median++;

		  if(flag_median == 10)
		  {
			  rampa_accel = toplam_accX/10.0;
			  imu_filter_calculate();
			  flag_median=0;
			  flag_new_imu_data =1;
		  }

		  flag_sensor_imu = 0;
	  }

/*************************************************************************************/
	  if(flag_sensor_barometre == 1){

		  prev_alt = altitude;
		  rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
		  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
		  if(rslt == BME280_OK )
		  {
			  temperature = comp_data.temperature / 100.00;
			  humidity = comp_data.humidity;
			  pressure = comp_data.pressure;
			  altitude = BME280_Get_Altitude() - offset_altitude;
			  altitude_kalman = KalmanFilter_Update(&kf, altitude);
			 // speed_time = (HAL_GetTick()-speed_time_prev)/1000.0f;

			  speed = (altitude - prev_alt) * 3.37;
			 // speed_time_prev = speed_time;
			  flag_new_barometre_data=1;
		  }
		  flag_sensor_barometre =0;
	  }

/*************************************************************************************/
	  if((gps.seconds %4) ==2 && flag_lora ==1)
	  {
		 // HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_4);
		  time = HAL_GetTick();
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		  union_converter();
		  EGU_Buff_Load();
		 // HAL_UART_Transmit(&huart3, Lora_Tx_Buffer, LORA_TX_BUFFER_SIZE, HAL_MAX_DELAY);
		 // HAL_UART_Transmit_IT(&huart3, Lora_Tx_Buffer, LORA_TX_BUFFER_SIZE);
		  HAL_UART_Transmit_DMA(&huart3, Lora_Tx_Buffer, LORA_TX_BUFFER_SIZE);
		  flag_lora=0;
	  }

/*************************************************************************************/

		if(flag_megu==1)
		{

			HAL_UART_Transmit(&huart6, EGU_durum_sorgusu, 5, 1000);
			flag_megu=0;
		}


/*************************************************************************************/
	  if(altitude>altitude_max) altitude_max = altitude;

	  if(speed>speed_max) speed_max = speed;

	  if( Lsm_Sensor.Accel_X> x_max) x_max =  Lsm_Sensor.Accel_X;

	  if(flag_adc >=20 && flag_adc_cnt ==1)
	  {
		  if(adc > 2476) adc = 2234;
		  if(adc < 1755) adc = 1755;
		  // 6V = 1755 adc val 1,41V
		  // 8.4V = 2476 adc val 1,99V 0,58V
		 adc_pil_val=(float)( ( ( (adc/4095)*3.3)-1.41) / (1.99-1.41) ) *100 ; // pil conv
		 // adc_pil_val = (adc-1755)/(2746-1755)*100;
		 battery = adc_pil_val;
		 flag_adc_cnt =0;
		 flag_adc=0;
		 HAL_ADC_Start_IT(&hadc1);
	  }

/*************************************************************************************/

      magnetic_switch = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14); // pinler kontrol edilecek.
      button_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);

/*************************************************************************************/
      if(altitude_kalman > 100 ) flag_rampa_altitude =1;

      switch(SUSTAINER)
	{
      case RAMPA :

    	  if(rampa_accel >0/* && flag_new_imu_data ==1*/)
    	  {
    		  flag_new_imu_data =0;
    		  flag_kontrol_5x++;
    	  }

    	  if(flag_kontrol_5x >=2)
    	  {
    		  flag_kontrol_5x =0;
    		  SUSTAINER=UCUS_BASLADI;
    		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
    	  }

    	  break;

      case UCUS_BASLADI :
    	  timer=HAL_GetTick();
    	  SUSTAINER=KADEMEAYRILDIMI;


		  break;

      case KADEMEAYRILDIMI :

    	  if( ( (magnetic_switch ==0 && (HAL_GetTick()-timer)>=4500) && flag_rampa_altitude ==1) || ((HAL_GetTick()-timer)==6500) )
    	  {
    		  flag_kontrol_5x++;
    	  }
    	  if(flag_kontrol_5x >=5)
		  { HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
    		  SUSTAINER=AYRILDI;
    		  flag_kontrol_5x=0;
		  }
		  break;

      case AYRILDI :

			HAL_UART_Transmit(&huart6, EGU_motor_atesleme, 5, 1000);
			HAL_UART_Transmit(&huart6, EGU_motor_atesleme, 5, 1000);
			HAL_UART_Transmit(&huart6, EGU_motor_atesleme, 5, 1000);
			HAL_UART_Transmit(&huart6, EGU_motor_atesleme, 5, 1000);
			HAL_UART_Transmit(&huart6, EGU_motor_atesleme, 5, 1000);
			SUSTAINER=APOGEE;
		  break;

      case APOGEE :

    	  if(altitude < altitude_max  && flag_new_barometre_data ==1  && flag_rampa_altitude == 1 && speed <= -1 )
		  {
    		  flag_new_barometre_data =0;
			  flag_kontrol_5x++;
		  }
    	  if(flag_kontrol_5x >=5)
    	  {
    		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
			flag_kontrol_5x=0;
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
			SUSTAINER=SUSTAINER_ANA;
    	  }


		  break;

      case SUSTAINER_ANA :
    	  if(altitude <= 500  && flag_new_barometre_data ==1)
		  {
			  flag_new_barometre_data =0;
			  flag_kontrol_5x++;
		  }
    	  if(flag_kontrol_5x >=5)
		  {
    		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);
			flag_kontrol_5x=0;
			SUSTAINER=FINISH;
		  }

		  break;

      case FINISH :
		  break;
	}

/*************************************************************************************/
  //    Flash_buff_fill();

/*************************************************************************************/

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 16000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8400;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 300-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8400;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 19200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|CS_Pin|Buzzer_Pin|GATE_D_Pin
                          |GATE_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M0_Pin|M1_Pin|FN_Pin|Led2_Pin
                          |Led1_Pin|GATE_B_Pin|GATE_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 CS_Pin Buzzer_Pin GATE_D_Pin
                           GATE_C_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|CS_Pin|Buzzer_Pin|GATE_D_Pin
                          |GATE_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 Button_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_14|Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M0_Pin M1_Pin FN_Pin Led2_Pin
                           Led1_Pin GATE_B_Pin GATE_A_Pin */
  GPIO_InitStruct.Pin = M0_Pin|M1_Pin|FN_Pin|Led2_Pin
                          |Led1_Pin|GATE_B_Pin|GATE_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim==&htim2)// Lora timer
	{
		flag_lora=1;
	}

	if(htim==&htim3)// sensor timer 30ms
	{
		flag_sensor_imu=1;

		flag_counter++;
		if(flag_counter == 10)
		{
			flag_sensor_barometre =1;
			flag_counter=0;
		}
	}

	if(htim==&htim4)// megü timer
	{
		flag_megu=1;
		flag_adc++;
		flag_flash_200ms=1;

	}

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart2){
	if(rx_data_gps != '\n' && rx_index_gps < RX_BUFFER_SIZE) {
		rx_buffer_gps[rx_index_gps++] = rx_data_gps;
	} else {
		lwgps_process(&gps, rx_buffer_gps, rx_index_gps+1);
		rx_index_gps = 0;
		rx_data_gps = 0;
	}
	HAL_UART_Receive_IT(&huart2, &rx_data_gps, 1);
	}

	if(huart == &huart6){
		if(rx_data_EGU != '\n' && rx_index_EGU <34){//sol taraf silinebilir
			EGU_RX_BUFFER[rx_index_EGU++]=rx_data_EGU;

		}
		else
		{
			rx_data_EGU=0;
			rx_index_EGU=0;

		}
	HAL_UART_Receive_IT(&huart6, &rx_data_EGU, 1);
		}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1 )
	{
		adc= HAL_ADC_GetValue(&hadc1);
		flag_adc_cnt = 1;

	}
}

int8_t E220_write_register(uint8_t reg,uint8_t parameter)
{

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);

	HAL_Delay(3);

	uint8_t send_data[4]={CMD_SET_REG,reg,1,parameter};
	uint8_t receive_data[4]={0};

	HAL_UART_Transmit(&huart3,send_data ,4, 100);
	HAL_UART_Receive(&huart3, receive_data, 4, 100);


	if(receive_data[0]==CMD_READ_REG && receive_data[1]==reg && receive_data[2]==1 && receive_data[3] == parameter)
		return receive_data[3];
	else
		return -1;

}
int8_t E220_read_register(uint8_t reg)
{


	uint8_t send_data[3]={CMD_READ_REG,reg,1};
	uint8_t receive_data[4]={0};
	HAL_UART_Transmit(&huart3,send_data ,3, 100);



	HAL_UART_Receive(&huart3, receive_data, 4, 100);

	if(receive_data[0]==CMD_READ_REG && receive_data[1]==reg && receive_data[2]==1)
		return receive_data[3];
	else
		return -1;
}



void imu_offset(void)

{
	  for(int i = 0; i<5;i++)
	  {

		      LSM6DSLTR_Read_Gyro_Data(&Lsm_Sensor);

		      offset_x += Lsm_Sensor.Gyro_X;
			  offset_y += Lsm_Sensor.Gyro_Y;
			  offset_z += Lsm_Sensor.Gyro_Z;
			  HAL_Delay(10);
	  }
	  offset_x=offset_x/5;
	  offset_y=offset_y/5;
	  offset_z=offset_z/5;
}

void imu_filter_calculate()
{
	  filtered_gyro_LP[0] = LP_alpha * toplam_gX/10.0f + (1.0 - LP_alpha) * gyroX_LP_prev;
	  filtered_gyro_LP[1] = LP_alpha * toplam_gY/10.0f + (1.0 - LP_alpha) * gyroY_LP_prev ;
	  filtered_gyro_LP[2] = LP_alpha * toplam_gZ/10.0f + (1.0 - LP_alpha) * gyroZ_LP_prev;

	/*************************** ********************************/
	  filtered_gyro_HP_X = beta * (gyroX_HP_prev +  filtered_gyro_LP[0] - gyroX_LP_prev);
	  filtered_gyro_HP_Y = beta * (gyroY_HP_prev + filtered_gyro_LP[1] - gyroY_LP_prev);
	  filtered_gyro_HP_Z = beta * (gyroZ_HP_prev +  filtered_gyro_LP[2] - gyroZ_LP_prev);


	  gyroX_LP_prev =  filtered_gyro_LP[0];
	  gyroY_LP_prev =  filtered_gyro_LP[1];
	  gyroZ_LP_prev =  filtered_gyro_LP[2];

	  // Low pass filter for accelerometer data
	  filtered_acc_LP[0] = LP_alpha * filtered_acc_LP[0] + (1 - LP_alpha) *  toplam_accX / 10.0;
	  filtered_acc_LP[1] = LP_alpha * filtered_acc_LP[1] + (1 - LP_alpha) *  toplam_accY / 10.0;
	  filtered_acc_LP[2] = LP_alpha * filtered_acc_LP[2] + (1 - LP_alpha) *  toplam_accZ / 10.0;

	  gyroX_HP_prev = filtered_gyro_HP_X;
	  gyroY_HP_prev = filtered_gyro_HP_Y;
	  gyroZ_HP_prev = filtered_gyro_HP_Z;


	  real_roll = atan2f(filtered_acc_LP[1], sqrtf(filtered_acc_LP[0] * filtered_acc_LP[0] + filtered_acc_LP[2] * filtered_acc_LP[2] +  1e-10)) * 180.0f / 3.14;
	  real_pitch = atan2f(-filtered_acc_LP[0], sqrtf(filtered_acc_LP[1] * filtered_acc_LP[1] + filtered_acc_LP[2] * filtered_acc_LP[2]+ 1e-10)) * 180.0f / 3.14;

	  uint32_t current_time = HAL_GetTick(); // current time
	  float dt = (current_time - prev_time1) / 1000.0f;

	  real_roll = ALPHA * (real_roll + filtered_gyro_HP_X * dt) + (1 - ALPHA) * real_roll;
	  real_pitch = ALPHA * (real_pitch + filtered_gyro_HP_Y * dt) + (1 - ALPHA) * real_pitch;

	  prev_time1 = current_time;
	  gravity_normal_angle = sqrtf(real_roll * real_roll + real_pitch * real_pitch);

	  toplam_roll = 0;
	  toplam_pitch = 0;
	  toplam_accX = 0;
	  toplam_accY = 0;
	  toplam_accZ = 0;
	  toplam_gX = 0;
	  toplam_gY = 0;
	  toplam_gZ = 0;
}

void Altitude_Offset()
{
	for(uint8_t i=0;i<10;i++)
	{
		HAL_Delay(40);
	  rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
	  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
	  if(rslt == BME280_OK)
	  { pressure = comp_data.pressure;
	    offset_altitude=BME280_Get_Altitude();
	  }
	}
}

void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
}

void union_converter(void)
{

	  Lora_Tx_Buffer[4]= gps.sats_in_view;
	 float2unit8 f2u8_gpsalt;
    f2u8_gpsalt.fVal=gps.altitude;
		 for(uint8_t i=0;i<4;i++)
		 {
			 Lora_Tx_Buffer[i+5]=f2u8_gpsalt.array[i];
		 }

	 float2unit8 f2u8_latitude;
	 f2u8_latitude.fVal=gps.latitude;
		 for(uint8_t i=0;i<4;i++)
		 {
			 Lora_Tx_Buffer[i+9]=f2u8_latitude.array[i];
		 }

	 float2unit8 f2u8_longitude;
	 f2u8_longitude.fVal=gps.longitude;
		 for(uint8_t i=0;i<4;i++)
		 {
			 Lora_Tx_Buffer[i+13]=f2u8_longitude.array[i];
		 }

	 float2unit8 f2u8_altitude;
	 f2u8_altitude.fVal=altitude;
		 for(uint8_t i=0;i<4;i++)
		 {
			 Lora_Tx_Buffer[i+17]=f2u8_altitude.array[i];
		 }
	 float2unit8 f2u8_speed;
	 f2u8_speed.fVal=speed;
		 for(uint8_t i=0;i<4;i++)
		 {
			 Lora_Tx_Buffer[i+21]=f2u8_speed.array[i];
		 }

	 float2unit8 f2u8_temp;
	 f2u8_temp.fVal=temperature;
		 for(uint8_t i=0;i<4;i++)
		 {
			 Lora_Tx_Buffer[i+25]=f2u8_temp.array[i];
		 }

	 float2unit8 f2u8_accx;
	 f2u8_accx.fVal=Lsm_Sensor.Accel_X;
		 for(uint8_t i=0;i<4;i++)
		 {
			 Lora_Tx_Buffer[i+29]=f2u8_accx.array[i];
		 }

	 float2unit8 f2u8_accy;
	 f2u8_accy.fVal=Lsm_Sensor.Accel_Y;
	 	 for(uint8_t i=0;i<4;i++)
		 {
	 		Lora_Tx_Buffer[i+33]=f2u8_accy.array[i];
		 }

	 float2unit8 f2u8_accz;
	 f2u8_accz.fVal=Lsm_Sensor.Accel_Z;
	 	 for(uint8_t i=0;i<4;i++)
		 {
	 		Lora_Tx_Buffer[i+37]=f2u8_accz.array[i];
		 }

	 float2unit8 f2u8_roll;
	 f2u8_roll.fVal=gravity_normal_angle;// real roll
		 for(uint8_t i=0;i<4;i++)
		 {
			 Lora_Tx_Buffer[i+41]=f2u8_roll.array[i];
		 }
	 float2unit8 f2u8_pitch;
	 f2u8_pitch.fVal=real_pitch;
		 for(uint8_t i=0;i<4;i++)
		 {
			 Lora_Tx_Buffer[i+45]=f2u8_pitch.array[i];
		 }

		 f2u8_altitude.fVal=altitude_max;
		 	Lora_Tx_Buffer[49]=battery;
			Lora_Tx_Buffer[69] = f2u8_altitude.array[0];
			Lora_Tx_Buffer[70] = f2u8_altitude.array[1];
			Lora_Tx_Buffer[71] = f2u8_altitude.array[2];
			Lora_Tx_Buffer[72] = f2u8_altitude.array[3];


}


void EGU_Buff_Load(void)
{
	Lora_Tx_Buffer[52]=EGU_RX_BUFFER[29];//EGU HATA
	Lora_Tx_Buffer[53]=EGU_RX_BUFFER[30];//Fitil kontrol 0 ise fitil bağlı değil 1 ise fitil bağlı
	Lora_Tx_Buffer[54]=EGU_RX_BUFFER[6];//EGU BATARYA-F
	Lora_Tx_Buffer[55]=EGU_RX_BUFFER[7];
	Lora_Tx_Buffer[56]=EGU_RX_BUFFER[8];
	Lora_Tx_Buffer[57]=EGU_RX_BUFFER[9];
	Lora_Tx_Buffer[58]=EGU_RX_BUFFER[22];//EGU ANGLE-F
	Lora_Tx_Buffer[59]=EGU_RX_BUFFER[23];
	Lora_Tx_Buffer[60]=EGU_RX_BUFFER[24];
	Lora_Tx_Buffer[61]=EGU_RX_BUFFER[25];
	Lora_Tx_Buffer[62]=EGU_RX_BUFFER[10];//EGU IRTIFA-F
	Lora_Tx_Buffer[63]=EGU_RX_BUFFER[11];
	Lora_Tx_Buffer[64]=EGU_RX_BUFFER[12];
	Lora_Tx_Buffer[65]=EGU_RX_BUFFER[13];
	Lora_Tx_Buffer[66]=EGU_RX_BUFFER[26];//EGU UCUS BASLADIMI?
	Lora_Tx_Buffer[67]=EGU_RX_BUFFER[28];//manyetik switch 1 ise kopmadı 0 ise koptu
	Lora_Tx_Buffer[68]=EGU_RX_BUFFER[27];//MOTOR ATESLEME TALEBİ GELDİ Mİ?
	Lora_Tx_Buffer[74]='\n';

}

void Flash_buff_fill()
{
	if( flag_flash_200ms == 1 && flag_flash ==0 /*&& SUSTAINER >=1*/)
	  	{
	  		if(i >= 252) {
	  			flag_flash=1;
	  		}

	  		conv.fVal=Lsm_Sensor.Accel_X;

	  		flash_accX[i] = conv.array[0];
	  		flash_accX[i+1] = conv.array[1];
	  		flash_accX[i+2] = conv.array[2];
	  		flash_accX[i+3] = conv.array[3];

	  		conv.fVal=Lsm_Sensor.Accel_Y;
	  		flash_accY[i] = conv.array[0];
	  		flash_accY[i+1] = conv.array[1];
	  		flash_accY[i+2] = conv.array[2];
	  		flash_accY[i+3] = conv.array[3];

	  		conv.fVal=Lsm_Sensor.Accel_Z;
	  		flash_accZ[i] = conv.array[0];
	  		flash_accZ[i+1] = conv.array[1];
	  		flash_accZ[i+2] = conv.array[2];
	  		flash_accZ[i+3] = conv.array[3];

	  		conv.fVal=Lsm_Sensor.Gyro_X;
	  		flash_gyroX[i] = conv.array[0];
	  		flash_gyroX[i+1] = conv.array[1];
	  		flash_gyroX[i+2] = conv.array[2];
	  		flash_gyroX[i+3] = conv.array[3];

	  		conv.fVal=Lsm_Sensor.Gyro_Y;
	  		flash_gyroY[i] = conv.array[0];
	  		flash_gyroY[i+1] = conv.array[1];
	  		flash_gyroY[i+2] = conv.array[2];
	  		flash_gyroY[i+3] = conv.array[3];

	  		conv.fVal=Lsm_Sensor.Gyro_Z;
	  		flash_gyroZ[i] = conv.array[0];
	  		flash_gyroZ[i+1] = conv.array[1];
	  		flash_gyroZ[i+2] = conv.array[2];
	  		flash_gyroZ[i+3] = conv.array[3];

	  		conv.fVal=altitude_kalman;
	  		flash_altitude[i] = conv.array[0];
	  		flash_altitude[i+1] = conv.array[1];
	  		flash_altitude[i+2] = conv.array[2];
	  		flash_altitude[i+3] = conv.array[3];



	  		flag_flash =0;

	  		i=i+4;

	  	}
}

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK) return -1;
  if(HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10) != HAL_OK) return -1;

  return 0;
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int8_t *buf;
  buf = malloc(len +1);
  buf[0] = reg_addr;
  memcpy(buf +1, data, len);

  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t*)buf, len + 1, HAL_MAX_DELAY) != HAL_OK) return -1;

  free(buf);
  return 0;
}



float BME280_Get_Altitude(void)
{
	float press = comp_data.pressure / 10000.0;
	float temp = comp_data.temperature / 100.0;
	float alt = 44330 * (1 - pow((press / 1013.25),(1/5.255)));
	//alt = ((pow((P0/press), (1/5.257))-1) * (temp + 273.15)) / 0.0065;

	return (alt);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
