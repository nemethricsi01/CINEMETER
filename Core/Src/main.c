/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include <stdbool.h>
#include "ssd1306.h"
#include "ssd1306_conf.h"
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "FLASH_SECTOR_F4.h"
#include "MY_NRF24.h"
#include "VL53L1X.h"
VL53L1X sensor1;
VL53L1X sensor2;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
#define USART_BUF_SIZE              9
#define LONG_PRESS_DURATION 100
#define LOW 0

// Constants

/*USART_BUFFER_SIZE*/
uint8_t usart_rx_buf[USART_BUF_SIZE];
// Constants

/*USART_BUFFER_SIZE*/
uint8_t usart_rx_buf[USART_BUF_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint8_t HEADER = 0x59;
//char buffer[40];
char strength_buffer[20];
char ofst_buffer[20];
char tx_buffer[100];
char RS_buffer[100];
char rs232_1[20];
char rs232_2[20];
char dist_buffer[100];
char twosens[100];

char DistTransmit[20];
uint64_t TxpipeAddrs = 0x11223344AA;

uint16_t temp_d;
uint16_t timer_val;
double dist_rx;
#define TRUE  1
#define FALSE 0
//char dist_string[20];

uint16_t dist, strength = 0;
uint8_t checksum, chip_temp = 0;
uint32_t last_loop_time = 0;
const uint32_t LOOP_INTERVAL = 100;
const uint32_t LOOP_INTERVAL1 = 1000;
int count = 0;
int offset = 0;

int laser_on = 0;

int button1_pressed = 0;
int button2_pressed = 0;
int button3_pressed = 0;

int integer_part = 0;
int fractional_part = 0;

GPIO_TypeDef *Button1_Port = GPIOC;
GPIO_TypeDef *Button2_Port = GPIOC;
GPIO_TypeDef *Button3_Port = GPIOC;

uint16_t Button1_Pin = GPIO_PIN_11;
uint16_t Button2_Pin = GPIO_PIN_10;
uint16_t Button3_Pin = GPIO_PIN_12;

#define BUTTON_PORT          GPIOC
#define DEBOUNCE_INC_DELAY       0
#define DEBOUNCE_DEC_DELAY       80

#define BUTTON_UP_PIN GPIO_PIN_11
#define BUTTON_DOWN_PIN GPIO_PIN_12
#define BUTTON_SELECT_PIN GPIO_PIN_10

#define XSHUT_GPIO_Pin          8
#define XSHUT_GPIO_Port		GPIOB

#define XSHUT2_GPIO_Pin          9
#define XSHUT2_GPIO_Port		GPIOB


#define DISTANCE_THRESHOLD 3000
#define DISPLAY_DURATION_MS 200
#define ONE_SECOND 1000

//#define FLASH_USER_ADDR 0x0803000// Start address of the last sector (Sector 7, 16 Kbytes)
//#define FLASH_USER_END_ADDR 0x0803FFFF
//#define FLASH_USER_START_ADDR
// Define the structure to hold device settings

uint32_t lastButtonPressTime = 0;
#define BUTTON_DEBOUNCE_DELAY 400

GPIO_PinState button_state = GPIO_PIN_RESET;
GPIO_PinState prev_button_state = GPIO_PIN_RESET;
uint8_t selected_item = 0;
uint8_t button_press_count = 0;

uint32_t RX_Flash_Data[6];
char string[30];

uint8_t switch_counter = 0;
uint8_t max_counter = 25; // Anzahl der Messungen, bevor der Sensor gewechselt wird

// Functions for reading the button states.
// Note: These should be replaced by the actual functions you use to read the button states.

#define DEBOUNCE_TIME 50
#define LONG_PRESS_TIME 1000

typedef enum {
  STATE_IDLE,
  STATE_MENU,
  STATE_SETTING,
} state_t;

typedef enum {
  EVENT_NONE,
  EVENT_BUTTON_SHORT_PRESS,
  EVENT_BUTTON_LONG_PRESS,
} event_t;

state_t state = STATE_IDLE;
event_t event = EVENT_NONE;
int setting_index = 0;
char *settings[] = {"Setting 1", "Setting 2", "Setting 3"};
int settings_values[] = {0, 0, 0};

void handle_button() {
  static uint32_t button_press_timestamp = 0;
  static uint32_t button_release_timestamp = 0;
  static uint32_t last_button_press_timestamp = 0;
  static int button_released = 1;

  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_SET) {
    if (button_released) {
      button_released = 0;
      button_press_timestamp = HAL_GetTick();
      if (button_press_timestamp - last_button_press_timestamp <= DEBOUNCE_TIME) {
        event = EVENT_BUTTON_LONG_PRESS;
      }
      last_button_press_timestamp = button_press_timestamp;
    }
  } else {
    if (!button_released) {
      button_released = 1;
      button_release_timestamp = HAL_GetTick();
      if (button_release_timestamp - button_press_timestamp > LONG_PRESS_TIME) {
        event = EVENT_BUTTON_LONG_PRESS;
      } else {
        event = EVENT_BUTTON_SHORT_PRESS;
      }
    }
  }
}

void update_display() {
  ssd1306_Fill(Black);
  if (state == STATE_MENU) {
    for (int i = 0; i < sizeof(settings) / sizeof(settings[0]); i++) {
      ssd1306_SetCursor(2, 10 + 12 * i);
      if (i == setting_index) {
        ssd1306_WriteString(">", Font_7x10, White);
      }
      ssd1306_SetCursor(20, 10 + 12 * i);
      ssd1306_WriteString(settings[i], Font_7x10, White);
    }
  } else if (state == STATE_SETTING) {
    ssd1306_SetCursor(2, 10);
    ssd1306_WriteString(settings[setting_index], Font_7x10, White);
    ssd1306_SetCursor(2, 22);
    char buf[10];
    sprintf(buf, "%d", settings_values[setting_index]);
    ssd1306_WriteString(buf, Font_7x10, White);
  }
  ssd1306_UpdateScreen();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_11)
    {
        handle_button();
    }

    if (GPIO_Pin == GPIO_PIN_10)
    {
        if (state == STATE_MENU) {
            setting_index = (setting_index - 1 + sizeof(settings) / sizeof(settings[0])) % sizeof(settings) / sizeof(settings[0]);
            update_display();
        } else if (state == STATE_SETTING) {
            settings_values[setting_index]--;
            update_display();
        }
    }

    if (GPIO_Pin == GPIO_PIN_12)
    {
        if (state == STATE_MENU) {
            setting_index = (setting_index + 1) % sizeof(settings) / sizeof(settings[0]);
            update_display();
        } else if (state == STATE_SETTING) {
            settings_values[setting_index]++;
            update_display();
        }
    }
}


uint16_t read_distance() {
 // Initialize to a safe value
	static int last_valid_temp_d = 0;
	HAL_UART_Receive(&huart6, usart_rx_buf, 9, 1);
	if (usart_rx_buf[0] == HEADER && usart_rx_buf[1] == HEADER) {
		checksum = 0;               // Initialize the checksum variable to zero.
		for (int i = 0; i < 8; i++) {
			checksum += usart_rx_buf[i];
		}
		if (usart_rx_buf[8] == (checksum & 0xff)) {
			dist = usart_rx_buf[2] | (usart_rx_buf[3] << 8);
			strength = usart_rx_buf[4] | (usart_rx_buf[5] << 8);
			if (dist != temp_d) {
				int potential_temp_d = dist + offset;
				if (potential_temp_d <= 4500 && potential_temp_d >= 002
						&& potential_temp_d != 65535) {
					last_valid_temp_d = potential_temp_d; // Update the last valid temp_d
					temp_d = last_valid_temp_d;
				}

			}

		}
		static uint16_t prev_distance_mm = 0;
		uint16_t distance_mm = TOF_GetDistance(&sensor1);
		int diff_distance = abs((int) distance_mm - (int) prev_distance_mm); // Calculate the difference

	    // Check if the strength is above threshold, otherwise use temp_d

		if ((strength / 655) >= 9 && distance_mm / 1000 < 2) {
		    // For 'distance_mm', make sure it's less than 2 meters
		    prev_distance_mm = distance_mm; // Update the previous distance
		    int integer_part = distance_mm / 1000;
		    int fractional_part = ((distance_mm % 1000) / 10);
		    sprintf(tx_buffer, "S%d.%02d\r\n", integer_part, fractional_part);
		   // show_Display();
			if (integer_part >= 10) {
				sprintf(rs232_2, "|1%d%02d\r\n", integer_part, fractional_part);
			} else {
				sprintf(rs232_2, "|1 %d%02d\r\n", integer_part, fractional_part);
			}

			if (fractional_part = 00) {
				sprintf(rs232_2, "|1 010\r\n");
			}

		} else {
		    // For 'temp_d'
		    prev_distance_mm = temp_d; // Use temp_d instead
		    int integer_part = temp_d / 100;
		    int fractional_part = (temp_d % 100);
		    sprintf(tx_buffer, "L%d.%02d\r\n", integer_part, fractional_part);
		  //  show_Display();
			if (integer_part >= 10) {
				sprintf(rs232_2, "|1%d%02d\r\n", integer_part, fractional_part);
			} else {
				sprintf(rs232_2, "|1 %d%02d\r\n", integer_part, fractional_part);
			}

			if (fractional_part = 00) {
				sprintf(rs232_2, "|1 010\r\n");
			}

		}
		 show_Display();


	}

}


void show_Display() {

	//uint16_t distance = get_distance();

	sprintf(ofst_buffer, "%+d cm", offset, 0);
	sprintf(strength_buffer, "SGNL: %d", (strength / 655));
	// Update the display

	//sprintf(tx_buffer, "%d.%02d", integer_part, fractional_part);
	//sprintf(strength_buffer, "SENS: AUTO", );

	//HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
	// hier hin das mit dem display
	ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 10);
	ssd1306_WriteString("OFST:", Font_7x10, White);
	ssd1306_SetCursor(50, 10);

	//
	ssd1306_WriteString(ofst_buffer, Font_7x10, White);

	ssd1306_SetCursor(0, 20);
	ssd1306_WriteString(strength_buffer, Font_7x10, White);

	ssd1306_SetCursor(0, 35);
	ssd1306_WriteString(tx_buffer, Font_16x24, White);
	ssd1306_SetCursor(90, 30);
	ssd1306_WriteString("M", Font_16x26, White);
	ssd1306_SetCursor(90, 20);
	ssd1306_WriteString("METER", Font_7x10, White);
	ssd1306_UpdateScreen();

}

void process_distance(){
	uint16_t distance1 = TOF_GetDistance(&sensor1);
	uint16_t distance2 = TOF_GetDistance(&sensor2);
	 sprintf(twosens, "SENS1:%d\r\n SENS2: %d\r\n", distance1, distance2);
	 HAL_UART_Transmit(&huart2, (uint8_t*) twosens, strlen(twosens),
	 		HAL_MAX_DELAY);
}

void RS232() {

	sprintf(rs232_1, "|0  60\r\n");

	if (__HAL_TIM_GET_COUNTER(&htim10) - timer_val >= 500) {
//		HAL_Delay(100);


		HAL_UART_Transmit(&huart2, (uint8_t*) rs232_1, strlen(rs232_1),
		HAL_MAX_DELAY);
//		HAL_Delay(100);
		HAL_UART_Transmit(&huart2, (uint8_t*) rs232_2, strlen(rs232_2),
		HAL_MAX_DELAY);
		timer_val = __HAL_TIM_GET_COUNTER(&htim10);
	}
}

// GET BUTTON STATUS

void modifyOffset(int *offset) {
	static uint8_t debounceFlag = 0;
	if (HAL_GPIO_ReadPin(BUTTON_PORT, GPIO_PIN_12) == GPIO_PIN_SET
			&& debounceFlag == 0) {
		(*offset)++;
		debounceFlag = 1;
		show_Display();
		HAL_Delay(DEBOUNCE_INC_DELAY);
	} else if (HAL_GPIO_ReadPin(BUTTON_PORT, GPIO_PIN_11) == GPIO_PIN_SET
			&& debounceFlag == 0) {
		(*offset)--;
		debounceFlag = 1;
		show_Display();
		HAL_Delay(DEBOUNCE_DEC_DELAY);
	}

	if (HAL_GPIO_ReadPin(BUTTON_PORT, GPIO_PIN_12) == GPIO_PIN_RESET
			&& HAL_GPIO_ReadPin(BUTTON_PORT, GPIO_PIN_11) == GPIO_PIN_RESET) {
		debounceFlag = 0;
	}

}

void laser(uint8_t status) {
	if (status) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
}

void read_laser() {

	if (HAL_GPIO_ReadPin(BUTTON_PORT, GPIO_PIN_10) == GPIO_PIN_SET) {

		laser(1);  // Turn ON the laser
	} else {
		laser(0);  // Turn OFF the laser
	}
}

// SAVE TO FLASH

// TRANSMIT DIST DATA

void TransmitData() {
	sprintf(DistTransmit, "%d", temp_d);
	if (__HAL_TIM_GET_COUNTER(&htim10) - timer_val >= 500) {

		NRF24_write(DistTransmit, 20);
		timer_val = __HAL_TIM_GET_COUNTER(&htim10);
	}

}
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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C3_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
//    uint32_t uint_offset = (uint32_t)offset;
//    int numofwords = (strlen(offset)/4)+((strlen(offset)%4)!=0);
	HAL_TIM_Base_Start(&htim10);
	/* USER CODE END 2 */
	timer_val = __HAL_TIM_GET_COUNTER(&htim10);

	sprintf(rs232_1, "|0  60\r\n");

	sprintf(rs232_2, "|1 187\r\n");
//	sprintf(rs232_2, "|1 021\r\n");

	HAL_Delay(100);
	HAL_UART_Transmit(&huart2, (uint8_t*) rs232_1, strlen(rs232_1),
	HAL_MAX_DELAY);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart2, (uint8_t*) rs232_2, strlen(rs232_2),
	HAL_MAX_DELAY);

	ssd1306_Init();
	ssd1306_WriteCommand(0xC0);
	ssd1306_WriteCommand(0xA0);
	LogoAnimation();

	TOF_InitStruct(&sensor1, &hi2c3, 0x32, XSHUT_GPIO_Port, XSHUT_GPIO_Pin);
	TOF_InitStruct(&sensor2, &hi2c3, 0x52, XSHUT2_GPIO_Port, XSHUT2_GPIO_Pin);

	VL53L1X* sensors[] = {&sensor1, &sensor2};
	TOF_BootMultipleSensors(sensors, 2);

	/* NRF BEGIN

	 NRF24_begin(GPIOB, GPIO_PIN_6, GPIO_PIN_7, hspi2);
	 nrf24_DebugUART_Init(huart2);

	 printRadioSettings();
	 NRF24_stopListening();
	 NRF24_openWritingPipe(TxpipeAddrs);
	 NRF24_setAutoAck(false);
	 NRF24_setChannel(1);
	 NRF24_setPayloadSize(20);
	 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	temp_d = 0;
	prev_button_state = button_state;


	while (1) {

//		 read_distance();

//		get_distance();
//		read_data_VL53L1();
//		 modifyOffset(&offset);
//		 RS232();
//	   	 HAL_GPIO_TogglePin(GPIOA, LED_Pin);
//		 read_laser();
//		process_distance();
//		print_distance();
//		read_data_VL53L1();
//		Offset_write_to_flash(offset);

		if (event == EVENT_BUTTON_SHORT_PRESS) {
		      if (state == STATE_IDLE) {
		        state = STATE_MENU;
		      } else if (state == STATE_MENU) {
		        state = STATE_SETTING;
		      } else if (state == STATE_SETTING) {
		        state = STATE_MENU;
		      }
		      update_display();
		      event = EVENT_NONE;
		    } else if (event == EVENT_BUTTON_LONG_PRESS) {
		      if (state == STATE_MENU) {
		        state = STATE_IDLE;
		      }
		      update_display();
		      event = EVENT_NONE;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8000-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart6.Init.BaudRate = 115200;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LASER_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SCK_Input_Pin_Pin|CSNPin_Pin|CEPin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LASER_Pin LED_Pin */
  GPIO_InitStruct.Pin = LASER_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_1_Pin BUTTON_2_Pin BUTTON_3_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin|BUTTON_2_Pin|BUTTON_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_Pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SCK_Input_Pin_Pin */
  GPIO_InitStruct.Pin = SCK_Input_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SCK_Input_Pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CSNPin_Pin CEPin_Pin */
  GPIO_InitStruct.Pin = CSNPin_Pin|CEPin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*


 void display_menu() {

 ssd1306_Fill(Black);
 ssd1306_SetCursor(0, 0);
 ssd1306_WriteString("MENU", Font_7x10, White);


 for (int i = 0; i < NUM_OPTIONS; i++) {
 if (i == selected_option) {
 ssd1306_WriteString("> ", Font_7x10, White);
 } else {
 ssd1306_WriteString("  ", Font_7x10, White);
 }
 ssd1306_WriteString(options[i], Font_7x10, White);
 ssd1306_SetCursor(28, (i+1)*12);
 }

 ssd1306_UpdateScreen();
 }


 */

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY); // If you want to direct printf to some other ports let say uart1 or 6 then change the first param.

	return ch;
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
	while (1) {
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
