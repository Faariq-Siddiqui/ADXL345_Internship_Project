/* Including  Header Files */
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Defining Device Defaults*/
#define ADXL345_DEV_ADDRESS 0x53 << 1
#define ADXL345_REG_BW_RATE 0x2C
#define ADXL345_REG_DEVID 0x00

/* Setting the registers which prompt ADXL345 for reading vibrations*/
#define ADXL345_REG_POWER_CTL 0x2D
#define ADXL345_MEASURE_MODE 0x08
#define ADXL345_REG_THRESHOLD_ACTIVE 0x24
#define ADXL345_REG_THRESHOLD_INACTIVE 0x25
#define ADXL345_REG_TAP_AXES 0x2A
#define ADXL345_REG_DATA_FORMAT 0x31

/* Registers which hold the readings */
#define ADXL345_REG_X0 0x32
#define ADXL345_REG_X1 0x33
#define ADXL345_REG_Y0 0x34
#define ADXL345_REG_Y1 0x35
#define ADXL345_REG_Z0 0x36
#define ADXL345_REG_Z1 0x37

/*miscellaneous macros for use */
#define ADXL345_HIGH_REG_VALUE 0xff
#define ACCELERATION_DUE_TO_GRAVITY 9.8
#define SCALE_FACTOR 4.3
#define OUTPUT_AS_TXT 1
#define OUTPUT_AS_CSV 2
#define DELAY_PER_CYCLE 500

// Initialises Variables for value in ' G ' or ' m/s^2 '
float total_value_in_G = 0;
float total_value_in_MeterPerSecondSquare = 0;
int timeInMilliSeconds = 0;

/* creates structures to store values in registers */
struct acc_reg_data{
	int16_t reg_x;
	int16_t reg_y;
	int16_t reg_z;
} sensorData;

struct output_reg_data{
	float reg_x;
	float reg_y;
	float reg_z;
} output_in_G, output_in_MetersPerSecondSquared;

/*initialisations...*/
uint8_t dataStoreBuffer[1];
uint8_t temp;


I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* Private user code ---------------------------------------------------------*/

/**************************************************
 * @brief:  This function writes to a single register of slave device whose address is supplied as a parameter under I2C Protocol.
 * @parameter:  MemAddress -> ( unsigned 16 bit address ) usually supplied as a hex value. This is the address of the register of the slave device to which we are writing.
 * @parameter:  Data -> (unsigned 8 bit value) usually supplied as a hex value. This is the value that one wants to set in that register
 * @return value: Null
**************************************************/
void ADXL345_Write_To_Reg(uint16_t MemAddress, uint8_t Data){
	if (HAL_I2C_Mem_Write(&hi2c1, ADXL345_DEV_ADDRESS, MemAddress, 1, &Data, 1, 100) == HAL_OK )
	{
		temp = 23;
	}
	else
	{
		temp = 1;

	}
}

// Uses ADXL345_Write_To_Reg() to write to registers of ADXL345, thus initialising it for use.
void ADXL345_Init(void){

	/****************************************************************
	 * Guide for setting Range in ADXL345: [Use these values in ADXL345_REG_DATA_FORMAT]
	 *
	 * 02 G ----------> 0x00 <----------- for 10 Bit - Resolution
	 * 04 G ----------> 0x01 <----------- for 10 Bit - Resolution
	 * 08 G ----------> 0x02 <----------- for 10 Bit - Resolution
	 * 16 G ----------> 0x00 <----------- for 10 Bit - Resolution
	 *
	 * 02 G ----------> 0x08 <----------- for 10 Bit - Full Resolution
	 * 04 G ----------> 0x09 <----------- for 11 Bit - Full Resolution
	 * 08 G ----------> 0x0A <----------- for 12 Bit - Full Resolution
	 * 16 G ----------> 0x0B <----------- for 13 Bit - Full Resolution
	 *
	****************************************************************/

	ADXL345_Write_To_Reg(ADXL345_REG_DATA_FORMAT, 0x08); // This is where we change the values for 2G ,4G,8G,16G;

	ADXL345_Write_To_Reg(ADXL345_REG_POWER_CTL, 0x3B); // Power Control Register
	ADXL345_Write_To_Reg(ADXL345_REG_BW_RATE, 0x0A); // Set 0x0A for 100Hz which is for I2C Protocol

}

/**************************************************
* @brief:  This function reads from a single register of slave device whose address is supplied as a parameter under I2C Protocol.
* @parameter:  MemAddress -> ( unsigned 16 bit address ) usually supplied as a hex value. This is the address of the register of the slave device being read from.
* @return value: dataStoreBuffer[0], returns the 1 and only byte stored in dataStoreBuffer. This has the data read from the register
* 				 which maybe used directly or maybe stored in a variable for later use.
**************************************************/

uint8_t ADXL345_Read_From_Reg(uint16_t MemAddress){
	if (HAL_I2C_Mem_Read(&hi2c1, ADXL345_DEV_ADDRESS, MemAddress, 1, dataStoreBuffer, 1, 100) == HAL_OK )
	{
		temp = 23;
	}
	else
	{
		temp = 1;

	}
	return dataStoreBuffer[0];
}

// Uses ADXL345_Read_From_Reg() function to read from registers.
void read_RegistersXYZ(void){

	uint8_t x0, y0, z0, x1, y1, z1;

	x0 = ADXL345_Read_From_Reg(ADXL345_REG_X0);
	x1 = ADXL345_Read_From_Reg(ADXL345_REG_X1);

	y0 = ADXL345_Read_From_Reg(ADXL345_REG_Y0);
	y1 = ADXL345_Read_From_Reg(ADXL345_REG_Y1);

	z0 = ADXL345_Read_From_Reg(ADXL345_REG_Z0);
	z1 = ADXL345_Read_From_Reg(ADXL345_REG_Z1);

	sensorData.reg_x = (int)(x1 << 8 | x0);
	sensorData.reg_y = (int)(y1 << 8 | y0);
	sensorData.reg_z = (int)(z1 << 8 | z0);

}


float  convert_output_to_G(uint16_t registerValue){
	float register_value_in_G = (float)((registerValue * SCALE_FACTOR)/ 1000 );
	return register_value_in_G;
}

float convert_output_to_MeterPerSecondSqaure(uint16_t registerValue){
	float  register_value_in_MeterPerSecondSqaure = (float)((registerValue * SCALE_FACTOR)/ 1000 )* ACCELERATION_DUE_TO_GRAVITY;
	return register_value_in_MeterPerSecondSqaure;
}

// This helps provide printf output to the console shell in STM32CubeIDE via VirtualCOM through USART2.
int _write(int file, char *ptr, int len){
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

// This helps provide scanf output to the console shell in STM32CubeIDE via VirtualCOM through USART2.
int _read(int file, char *ptr, int len) {
	HAL_UART_Receive(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

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


  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  ADXL345_Init();

  // This is where we decide the output format
  int outputFormat = 2;
  if (outputFormat == 2){
	  printf("Reading Number, Time (in ms), X:, Y:, Z:, X (in G):, Y (in G):, Z (in G):, X (in m/s^2): , Y (in m/s^2):, Z (in m/s^2):, Total Value in G, Total Value in m/s^2\r\n");
  }

  while (1)
    {

  	  for ( int numberOfReadings = 1; numberOfReadings <= 25; numberOfReadings++ ){

  			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  			read_RegistersXYZ();

  			output_in_G.reg_x = convert_output_to_G(sensorData.reg_x);
  			output_in_G.reg_y = convert_output_to_G(sensorData.reg_y);
  			output_in_G.reg_z = convert_output_to_G(sensorData.reg_z);

  			output_in_MetersPerSecondSquared.reg_x = convert_output_to_MeterPerSecondSqaure(sensorData.reg_x);
  			output_in_MetersPerSecondSquared.reg_y = convert_output_to_MeterPerSecondSqaure(sensorData.reg_y);
  			output_in_MetersPerSecondSquared.reg_z = convert_output_to_MeterPerSecondSqaure(sensorData.reg_z);

  			total_value_in_G = sqrt(pow(output_in_G.reg_x, 2) + pow(output_in_G.reg_y, 2) + pow(output_in_G.reg_z, 2));
  			total_value_in_MeterPerSecondSquare = sqrt(pow(output_in_MetersPerSecondSquared.reg_x,2) + pow(output_in_MetersPerSecondSquared.reg_y,2) + pow(output_in_MetersPerSecondSquared.reg_z,2));
  			// This flag determines whether the output shown would be in TXT Format or CSV Format;

  			timeInMilliSeconds = (numberOfReadings-1) * DELAY_PER_CYCLE;

  			switch (outputFormat){

  			case OUTPUT_AS_TXT:
  				printf("-----------------------------------------------------------------------------------------------------------\r\n");
  				printf("%d. x: %d, y: %d, z: %d RAW OUTPUT\r\n", numberOfReadings, sensorData.reg_x, sensorData.reg_y, sensorData.reg_z);
  	  			printf("\r\n");
				printf("%d. x: %f, y: %f, z: %f G\r\n", numberOfReadings, output_in_G.reg_x, output_in_G.reg_y, output_in_G.reg_z);
  				printf("\r\n");
  				printf("%d. x: %f, y: %f, z: %f m/s^2\r\n", numberOfReadings, output_in_MetersPerSecondSquared.reg_x, output_in_MetersPerSecondSquared.reg_y, output_in_MetersPerSecondSquared.reg_z);
  				printf("\r\n");
  				printf("Total Value: %f G\r\n", total_value_in_G);
  				printf("\r\n");
  				printf("Total Value: %f m/s^2\r\n", total_value_in_MeterPerSecondSquare);
  				printf("\r\n");
  				printf("Time (in ms): %d", timeInMilliSeconds);
  				printf("-----------------------------------------------------------------------------------------------------------\r\n");

  				HAL_Delay(DELAY_PER_CYCLE);
  				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  				HAL_Delay(DELAY_PER_CYCLE);
  				break;

  			case OUTPUT_AS_CSV:
  				printf("%d, %d, %d, %d, %d, %f, %f, %f, %f, %f, %f, %f, %f\r\n", numberOfReadings, timeInMilliSeconds,
  				            sensorData.reg_x, sensorData.reg_y, sensorData.reg_z, output_in_G.reg_x, output_in_G.reg_y,output_in_G.reg_z,
  				            output_in_MetersPerSecondSquared.reg_x, output_in_MetersPerSecondSquared.reg_y, output_in_MetersPerSecondSquared.reg_z,
  				            total_value_in_G, total_value_in_MeterPerSecondSquare);

  				HAL_Delay(DELAY_PER_CYCLE);
  				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  				HAL_Delay(DELAY_PER_CYCLE);
  				break;
  			}



  	  	}

  	  while(1){

  	  }

    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 166;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

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
