/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define G_acc (0.004) // accelerometer gain in normal mode
#define G_mag (1.5) // magnetometer gain in normal mode

#define LSM303AGR_ACC_ADDR (0x19<<1) // Accelerometer address (notre adresse est sur 7 bits, on ne perd pas l'information en faisant le décalage)
#define LSM303AGR_WHO_AM_I_A (0x0F) // Accelerometer Who am I Register

#define LSM303AGR_CTRL_REG1_A (0x20) // CTRL register 1
#define LSM303AGR_CTRL_REG5_A (0x24) // CTRL register 5
#define LSM303AGR_OUT_X_L_A (0x28) // Acceleration Registers


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char acc_data_write[2];
char acc_data_read[6];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void powerup_sensor(void); // power-up sensor and set basics parameters

void get_data(void); // read acceleration in acc_data_read[0:5] and magnetic field in mag_data_read[0:5]

void soft_reset(void); // reset sensor internal parameters ("factory reset")

void read_registers(void);// read all accelerometer and magnetometer registers and print it on serial

int16_t fusionbinaire(uint8_t LSB, uint8_t MSB);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void __io_putchar(uint8_t ch){
	HAL_UART_Transmit(&huart2, &ch, 1, 1);
}

int16_t fusionbinaire(uint8_t LSB, uint8_t MSB){
    int16_t res=0x0000;
    if(MSB & (1<<7)){
        MSB = (uint8_t)((~MSB)+1);
        res |= LSB>>6;
        res |= (MSB<<2);
        res = (int16_t)((~res)+1);
    }else{
        res |= LSB>>6;
        res |= (MSB<<2);
    }
    return res;
}

void get_data(void){
	 acc_data_write[0] = LSM303AGR_OUT_X_L_A | (1<<7);
	 HAL_I2C_Master_Transmit(&hi2c1,LSM303AGR_ACC_ADDR,(uint8_t*)acc_data_write,1,6000);
	 HAL_I2C_Master_Receive(&hi2c1,LSM303AGR_ACC_ADDR,(uint8_t*)acc_data_read,6,6000);

}
void powerup_sensor(void){
	// Turn on accelerometer
	HAL_StatusTypeDef ret;
	// Accelerometer test
	acc_data_write[0] = LSM303AGR_WHO_AM_I_A;

	ret = HAL_I2C_Master_Transmit(&hi2c1,LSM303AGR_ACC_ADDR,(uint8_t*)acc_data_write,1,2000);
	if (ret != HAL_OK) {printf("FAILED TX1: RETURN ERROR: %d",ret);}

	HAL_I2C_Master_Receive(&hi2c1,LSM303AGR_ACC_ADDR,(uint8_t*)acc_data_read,1,1000);
	printf("RECEIVED ACCELEROMETER 'WHO AM I' WITH ID : (0x%.2x)\n\r",acc_data_read[0]);

}

void soft_reset(void){
	acc_data_write[0]=LSM303AGR_CTRL_REG5_A | (1<<7);
	acc_data_write[1] = 0x40; //Reboot memory content -> en binaire 0b10000000
	HAL_I2C_Master_Transmit(&hi2c1,LSM303AGR_ACC_ADDR,(uint8_t*)acc_data_write,2,1000);
	HAL_Delay(100);
	acc_data_write[0] = LSM303AGR_CTRL_REG1_A;
	acc_data_write[1] = 0x27; //10 Hz, ENABLE X-AXIS,Y-AXIS,Z-AXIS
	HAL_I2C_Master_Transmit(&hi2c1,LSM303AGR_ACC_ADDR,(uint8_t*)acc_data_write,2,1000);
	printf("SOFT RESET [OK]\n\r");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 int16_t aXm,aYm,aZm; //valeurs brutes accéléromètre
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  printf("\033[2J\033[33m\r---- POSTURE ANALYZER) ----\n\r\033[0m");
  soft_reset();
  powerup_sensor();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(2000);
  uint16_t buff_count=0;
  while (1)
  {
	HAL_Delay(125);
	get_data();
	aXm=fusionbinaire(acc_data_read[0],acc_data_read[1]);
	aYm=fusionbinaire(acc_data_read[2],acc_data_read[3]);
	aZm=fusionbinaire(acc_data_read[4],acc_data_read[5]);


	// PRINTING VALUES
	buff_count++;
	printf("%03d %03d %03d",aXm,aYm,aZm);
	if (buff_count % 32 == 0) {
		printf("\n\r");
		buff_count=0;
	}
	else printf(" ");


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
