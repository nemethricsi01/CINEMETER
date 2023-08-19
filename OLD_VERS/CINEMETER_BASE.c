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
#include "ssd1306.h"
#include "ssd1306_conf.h"
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "NRF24L01.h"

//#include "MY_NRF24.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define LONG_PRESS_DURATION 80
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
const uint8_t HEADER=0x59;
//char buffer[40];
char strength_buffer[20];
char ofst_buffer[20];
char tx_buffer[20];
char rs232_1[20];
char rs232_2[20];


char DistTransmit[20] = "TEST DU FICK";
uint64_t TxpipeAddrs = 0x11223344AA;

uint16_t  temp_d;
uint16_t  timer_val;



//char dist_string[20];

uint16_t dist, strength=0;
uint8_t checksum, chip_temp=0;
uint32_t last_loop_time = 0;
const uint32_t LOOP_INTERVAL = 100;
const uint32_t LOOP_INTERVAL1 = 1000;
int count = 0;
int offset = 0;

int laser_on = 0;
int button1_pressed = 0;
int button3_pressed = 0;

 GPIO_TypeDef* Button1_Port = GPIOC;
 GPIO_TypeDef* Button2_Port = GPIOC;
 GPIO_TypeDef* Button3_Port = GPIOC;

 uint16_t Button1_Pin = GPIO_PIN_10;
 uint16_t Button2_Pin = GPIO_PIN_11;
 uint16_t Button3_Pin = GPIO_PIN_12;

#define BUTTON_PIN GPIO_PIN_11
#define BUTTON_GPIO_PORT GPIOC

 static uint32_t button1_press_time = 0;
 static uint32_t button2_press_time = 0;
 static uint32_t button3_press_time = 0;


#define BUTTON_1_PIN GPIO_PIN_10
#define BUTTON_1_PORT GPIOC
#define BUTTON_2_PIN GPIO_PIN_11
#define BUTTON_2_PORT GPIOC
#define BUTTON_3_PIN GPIO_PIN_12
#define BUTTON_3_PORT GPIOC

#define MENU_ITEMS_COUNT 4


 char* menu_items[MENU_ITEMS_COUNT] = {
     "Option 1",
     "Option 2",
     "Option 3",
     "Option 4"
 };

 uint8_t selected_item = 0;
 uint8_t button_press_count = 0;

 GPIO_PinState button_state = GPIO_PIN_RESET;
 GPIO_PinState prev_button_state = GPIO_PIN_RESET;

 // Define the menu text
 char menu_text[] = "MENU";
 int is_menu_displayed = 0;

 void delay_ms(uint32_t ms)
 {
   uint32_t tickstart = HAL_GetTick();
   while((HAL_GetTick() - tickstart) < ms);
 }





 uint8_t data[50];

 uint8_t TxAddress[] = {0xEE,0xDD,0xCC,0xBB,0xAA};
 uint8_t TxData[] = "Hello World\n";




/**
 *
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
  MX_I2C3_Init();
  /* USER CODE BEGIN SysInit */

  ssd1306_Init();
  ssd1306_WriteCommand(0xC0);
  ssd1306_WriteCommand(0xA0);
  ssd1306_Fill(Black);
  ssd1306_SetCursor(10,10);
  ssd1306_WriteString("CINEMETER", Font_11x18, White);
  ssd1306_UpdateScreen();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();

  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start(&htim10);
  /* USER CODE END 2 */
timer_val = __HAL_TIM_GET_COUNTER(&htim10);



  /* NRF BEGIN
HAL_Delay(1200);
  	  NRF24_begin(GPIOB, CSNPin_Pin, CEPin_Pin, hspi2);

  	  NRF24_stopListening();
  	  NRF24_openWritingPipe(TxpipeAddrs);
  	  NRF24_setAutoAck(false);
  	  NRF24_setChannel(1);
  	  NRF24_setPayloadSize(20);


  	 nrf24_DebugUART_Init(huart2);
  	  printStatusReg();
  	  printRadioSettings();


  void display_menu() {
       ssd1306_Fill(Black);
       ssd1306_SetCursor(2, 0);
       ssd1306_WriteString("MENU", Font_11x18, White);
       ssd1306_SetCursor(0, 22);
       for (uint8_t i = 0; i < MENU_ITEMS_COUNT; i++) {
           if (i == selected_item) {
               ssd1306_WriteString("> ", Font_7x10, White);
           } else {
               ssd1306_WriteString("  ", Font_7x10, White);
           }
           ssd1306_WriteString(menu_items[i], Font_7x10, White);
           ssd1306_SetCursor(0, (i*8) + 12);
       }
       ssd1306_UpdateScreen();
   }


  uint8_t selected_item = 0;
  /* USER CODE END 2 */


NRF24_Init();

//  NRF24_RxMode(RxAddress, 10);

 NRF24_TxMode(TxAddress, 10);


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  temp_d = 0;
  prev_button_state = button_state;



  while (1)
  {
    // toggle the LED pin

	  if (NRF24_Transmit(TxData) == 1)
		  {
				 HAL_UART_Transmit(&huart2, (uint8_t*)"TX Succsess", strlen("TX Succsess"), HAL_MAX_DELAY);
					HAL_GPIO_TogglePin(GPIOA,LED_Pin);
		  }

	  HAL_Delay(1000);

		  HAL_UART_Transmit(&huart2, (uint8_t*)"No Succsess", strlen("No Succsess"), HAL_MAX_DELAY);
	  }
}
  /*
		HAL_GPIO_TogglePin(GPIOA,LED_Pin);



        button_state = HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_PIN);

          // Check for a button press
          if (button_state == GPIO_PIN_SET && prev_button_state == GPIO_PIN_RESET) {
            // Increment the button press count
            button_press_count++;

            // If this is the first press, display the menu text
            if (button_press_count == 1) {
            	display_menu();
              is_menu_displayed = 1;
            }
          }

          // Check for a double press
          if (button_state == GPIO_PIN_SET && prev_button_state == GPIO_PIN_SET && button_press_count) {
            // Clear the display and reset the button press count
            ssd1306_Fill(Black);
            ssd1306_UpdateScreen();
            button_press_count = 0;
            is_menu_displayed = 0;
          }

          // Update the previous button state
          prev_button_state = button_state;

          // Check if the menu is currently being displayed
          if (is_menu_displayed) {

            // Ignore other functions
          } else {

              read_data_uart1();


//                      writeData();
//                      TransmitData();
                      read_buttons();
//                      RS232();
//                      HAL_UART_Transmit(&huart2, (uint8_t*)"test", strlen("test"), HAL_MAX_DELAY);

          }



  }

}

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
  RCC_OscInitStruct.PLL.PLLN = 80;
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
  hi2c3.Init.ClockSpeed = 100000;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  HAL_GPIO_WritePin(GPIOB, CSNPin_Pin|CEPin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LASER_Pin LED_Pin */
  GPIO_InitStruct.Pin = LASER_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_1_Pin BUTTON_2_Pin BUTTON_3_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin|BUTTON_2_Pin|BUTTON_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IRQ_Pin SCK_Input_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin|SCK_Input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CSNPin_Pin CEPin_Pin */
  GPIO_InitStruct.Pin = CSNPin_Pin|CEPin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void read_data_uart1(){
    memset( usart_rx_buf, 0, sizeof( usart_rx_buf)); // Clear the data buffer.
    HAL_UART_Receive(&huart6, usart_rx_buf, 9, 1);

    if (usart_rx_buf[0]==HEADER && usart_rx_buf[1]==HEADER){
        checksum = 0; // Initialize the checksum variable to zero.
        for (int i=0;i<8;i++){
            checksum += usart_rx_buf[i];
        }
        if (usart_rx_buf[8] == (checksum&0xff)){

            dist = usart_rx_buf[2] | (usart_rx_buf[3]<<8);
            //            printf("temp_d value: %d \n", temp_d);
            //            printf("dist value: %d \n", dist);
            strength = usart_rx_buf[4] |(usart_rx_buf[5]<<8);
            if(dist != temp_d){



         	                                   temp_d = dist + offset;

         	                                   int integer_part = temp_d / 100;
         	                                   int fractional_part = (temp_d % 100);


         	                            sprintf(ofst_buffer, "%+d cm", offset, 0);

         	                            // Update the display




         	                            sprintf(tx_buffer, "%d.%02d", integer_part, fractional_part);
         	                            sprintf(strength_buffer, "SGNL: %d", (strength / 655));
         	                            //HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
         	                            // hier hin das mit dem display
         	                            ssd1306_Fill(Black);
         	                            ssd1306_SetCursor(0,10);
         	                            ssd1306_WriteString("OFST:", Font_7x10, White);
         	                            ssd1306_SetCursor(50, 10);

         	//
         	                            ssd1306_WriteString(ofst_buffer, Font_7x10, White);

         	                            ssd1306_SetCursor(0,20);
         	                            ssd1306_WriteString(strength_buffer, Font_7x10, White);

         	                            ssd1306_SetCursor(0,35);
         	                            ssd1306_WriteString(tx_buffer,  Font_16x24, White);
         	                            ssd1306_SetCursor(90,30);
         	                            ssd1306_WriteString("M", Font_16x26, White);
         	                            ssd1306_SetCursor(90,20);
         	                            ssd1306_WriteString("METER", Font_7x10, White);
         	                            ssd1306_UpdateScreen();
                    }
    }



}


void writeData() {



	                        }



}




void read_buttons() {





    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_SET) {
        if (!button1_pressed) {
            button1_press_time = HAL_GetTick(); // record button press time
            button1_pressed = 1;
            ssd1306_Fill(Black);
  	                            ssd1306_SetCursor(0,10);
  	                            ssd1306_WriteString("OFST:", Font_7x10, White);
  	                            ssd1306_SetCursor(50, 10);

  	//
  	                            ssd1306_WriteString(ofst_buffer, Font_7x10, White);

  	                            ssd1306_SetCursor(0,20);
  	                            ssd1306_WriteString(strength_buffer, Font_7x10, White);

  	                            ssd1306_SetCursor(0,35);
  	                            ssd1306_WriteString(tx_buffer,  Font_16x24, White);
  	                            ssd1306_SetCursor(90,30);
  	                            ssd1306_WriteString("M", Font_16x26, White);
  	                            ssd1306_SetCursor(90,20);
  	                            ssd1306_WriteString("METER", Font_7x10, White);
  	                            ssd1306_UpdateScreen();

        } else {
            uint32_t duration = HAL_GetTick() - button1_press_time;
            if (duration > LONG_PRESS_DURATION) { // check if button is pressed for a long time
                offset++; // increment offset value
                ssd1306_Fill(Black);
      	                            ssd1306_SetCursor(0,10);
      	                            ssd1306_WriteString("OFST:", Font_7x10, White);
      	                            ssd1306_SetCursor(50, 10);

      	//
      	                            ssd1306_WriteString(ofst_buffer, Font_7x10, White);

      	                            ssd1306_SetCursor(0,20);
      	                            ssd1306_WriteString(strength_buffer, Font_7x10, White);

      	                            ssd1306_SetCursor(0,35);
      	                            ssd1306_WriteString(tx_buffer,  Font_16x24, White);
      	                            ssd1306_SetCursor(90,30);
      	                            ssd1306_WriteString("M", Font_16x26, White);
      	                            ssd1306_SetCursor(90,20);
      	                            ssd1306_WriteString("METER", Font_7x10, White);
      	                            ssd1306_UpdateScreen();

        }
    }
    }else {
        button1_pressed = 0;
    }

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_SET) {
        if (!button3_pressed) {
            button3_press_time = HAL_GetTick(); // record button press time
            button3_pressed = 1;

        } else {
            uint32_t duration = HAL_GetTick() - button3_press_time;
            if (duration > LONG_PRESS_DURATION) { // check if button is pressed for a long time
                offset--; // decrement offset value
                ssd1306_Fill(Black);
      	                            ssd1306_SetCursor(0,10);
      	                            ssd1306_WriteString("OFST:", Font_7x10, White);
      	                            ssd1306_SetCursor(50, 10);

      	//
      	                            ssd1306_WriteString(ofst_buffer, Font_7x10, White);

      	                            ssd1306_SetCursor(0,20);
      	                            ssd1306_WriteString(strength_buffer, Font_7x10, White);

      	                            ssd1306_SetCursor(0,35);
      	                            ssd1306_WriteString(tx_buffer,  Font_16x24, White);
      	                            ssd1306_SetCursor(90,30);
      	                            ssd1306_WriteString("M", Font_16x26, White);
      	                            ssd1306_SetCursor(90,20);
      	                            ssd1306_WriteString("METER", Font_7x10, White);
      	                            ssd1306_UpdateScreen();

            }
        }
    } else {
        button3_pressed = 0;
    }

    // check if buttons 1 and 3 are pressed and activate the laser
    if (button1_pressed && button3_pressed && !laser_on) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        laser_on = 1;
        ssd1306_Fill(Black);
        ssd1306_Fill(Black);
        ssd1306_SetCursor(20,20);
        ssd1306_WriteString("LASER", Font_16x24, White);
        ssd1306_UpdateScreen();

        HAL_Delay(5000);

    }
    // check if either button 1 or 3 is released and deactivate the laser
    else if (!button1_pressed || !button3_pressed) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        laser_on = 0;
    }
}



void RS232() {
	   sprintf(rs232_1,"|0  60");
	   sprintf(rs232_2,"|1  %d", temp_d);

	if(__HAL_TIM_GET_COUNTER(&htim10) - timer_val >= 1500)
		{

		    HAL_UART_Transmit(&huart2, (uint8_t*)rs232_1, strlen(rs232_1), HAL_MAX_DELAY);
		    HAL_UART_Transmit(&huart2, (uint8_t*)rs232_2, strlen(rs232_2), HAL_MAX_DELAY);
	timer_val = __HAL_TIM_GET_COUNTER(&htim10);
		}
}






void save_option_to_flash(int option) {
  // TODO: Implement saving option to flash memory
}


void TransmitData() {
//	 sprintf(DistTransmit,"%d", temp_d);
	if(__HAL_TIM_GET_COUNTER(&htim10) - timer_val >= 500)
	{
		 if( NRF24_write(DistTransmit, 20))
		       {
		        ssd1306_Fill(Black);
		        ssd1306_SetCursor(100, 0);
		        ssd1306_WriteString("TX", Font_7x10, White);
		      ssd1306_UpdateScreen();

		       }
timer_val = __HAL_TIM_GET_COUNTER(&htim10);
	}

	HAL_Delay(100);


	/*

	if(__HAL_TIM_GET_COUNTER(&htim10) - timer_val >= 500)
		{
		// do something
	timer_val = __HAL_TIM_GET_COUNTER(&htim10);
		}

/*

       if( NRF24_write(DistTransmit, 20))
       {
        ssd1306_Fill(Black);
        ssd1306_SetCursor(100, 0);
        ssd1306_WriteString("TX", Font_7x10, White);
      ssd1306_UpdateScreen();

       }
*/

}



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

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY); // If you want to direct printf to some other ports let say uart1 or 6 then change the first param.

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
