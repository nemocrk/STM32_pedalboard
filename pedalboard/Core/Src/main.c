/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "memorymap.h"
#include "sai.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    // Filter coefficients
    float filter_coeffs[FILTER_TAP_NUM];
    // Delay line for FIR filter
    float delay_line[FILTER_TAP_NUM];
    uint32_t delay_line_index;
    // Volume control (0.0 to 1.0)
    float volume;
    // Sample rate conversion ratio
    float src_ratio;
} PCM3168A_DSP_Channel;

typedef struct {
    I2C_HandleTypeDef *hi2c1;
    PCM3168A_DSP_Channel dac_channels[PCM3168A_NUM_CHANNELS];
    PCM3168A_DSP_Channel adc_channels[PCM3168A_NUM_ADC];
    // Global configuration
    uint32_t sample_rate;
    uint8_t bits_per_sample;
    // Processing buffers
    int32_t input_buffer[BUFFER_SIZE];
    int32_t output_buffer[BUFFER_SIZE];
} PCM3168A_DSP;

// Structure to hold frequency detection state
typedef struct {
    int32_t prev_sample;
    uint32_t zero_crossings;
    uint32_t sample_count;
    float current_frequency;
} FrequencyDetector;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Use SIMD instructions where possible
#pragma GCC push_options
#pragma GCC target ("fpu=fpv5-d16")
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// DMA buffers
__attribute__((section(".dtcmram"))) 
static uint8_t DMA_TxBuffer[BUFFER_SIZE * 4]; // *4 because each 32-bit sample needs 4 bytes

__attribute__((section(".dtcmram"))) 
static uint8_t DMA_RxBuffer[BUFFER_SIZE * 4];

// Processing buffers
static int32_t ProcessTxBuffer[BUFFER_SIZE];
static int32_t ProcessRxBuffer[BUFFER_SIZE];

PCM3168A_DSP dsp;

FrequencyDetector freq_detector;

const int DC1_24_BIT_I2S_TDM = 0x06; // DAC_CONTROL_1: set 24-bit I²S mode, TDM format
const int AC1_24_BIT_I2S_TDM = 0x06; // ADC_CONTROL_1: set 24-bit I²S mode, TDM format

enum PCM3168reg {RESET_CONTROL = 0x40, DAC_CONTROL_1, DAC_CONTROL_2,
                  DAC_OUTPUT_PHASE, DAC_SOFT_MUTE_CONTROL, DAC_ZERO_FLAG,
                  DAC_CONTROL_3, DAC_ATTENUATION_ALL, DAC_ATTENUATION_BASE /* 8 registers */,
                  ADC_SAMPLING_MODE = 0x50, ADC_CONTROL_1, ADC_CONTROL_2,
                  ADC_INPUT_CONFIGURATION, ADC_INPUT_PHASE, ADC_SOFT_MUTE_CONTROL,
                  ADC_OVERFLOW_FLAG, ADC_CONTROL_3,
                  ADC_ATTENUATION_ALL, ADC_ATTENUATION_BASE /* 6 registers */
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void lcd_send_cmd (char cmd);
void lcd_send_data (char data);
void lcd_send_string (char *str);
void lcd_put_cur(int row, int col);
void lcd_init (void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef PCM3168A_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
    uint8_t data[2];
    data[0] = reg;
    data[1] = value;
    
    char buffer[16] = "";
    int i = 0;
    HAL_StatusTypeDef actualStatus = HAL_OK;
    for(i = 0; i<128; i++)
    {
        actualStatus = HAL_I2C_Master_Transmit(hi2c, i, data, 2, 100);
        if(actualStatus == HAL_OK)
        {
            snprintf(buffer,16,"%d [%d]",i,hi2c->ErrorCode);
            lcd_put_cur(0, 0);
            lcd_send_string("Maybe this:");
            lcd_put_cur(1, 0);
            lcd_send_string(buffer);
            break;
        }
        else
        {
            snprintf(buffer,16,"%d [%d]",i,hi2c->ErrorCode);
            lcd_put_cur(0, 0);
            lcd_send_string("Not this:");
            lcd_put_cur(1, 0);
            lcd_send_string(buffer);
        }
        HAL_Delay(200);
    }
    

    return HAL_I2C_Master_Transmit(hi2c, PCM3168A_I2C_ADDR, data, 2, 100);
}

// Function to read from PCM3168A register
HAL_StatusTypeDef PCM3168A_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef status;
    
    // Send register address
    status = HAL_I2C_Master_Transmit(hi2c, PCM3168A_I2C_ADDR, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    
    // Read register value
    return HAL_I2C_Master_Receive(hi2c, PCM3168A_I2C_ADDR + 1, value, 1, HAL_MAX_DELAY);
}
// Initialize DSP structure
void PCM3168A_DSP_Init(PCM3168A_DSP* dsp) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(1000);

    uint8_t output;
    HAL_StatusTypeDef actualState = HAL_OK;
    actualState = PCM3168A_ReadReg(dsp->hi2c1, DAC_CONTROL_1, &output);
    if(actualState != HAL_OK)
        printf("HAL_KO in first ReadReg: %d",actualState);
    actualState = PCM3168A_WriteReg(dsp->hi2c1, DAC_CONTROL_1, DC1_24_BIT_I2S_TDM)
        && PCM3168A_WriteReg(dsp->hi2c1, ADC_CONTROL_1, AC1_24_BIT_I2S_TDM)
        && PCM3168A_WriteReg(dsp->hi2c1, DAC_SOFT_MUTE_CONTROL, 0x00)
        && PCM3168A_WriteReg(dsp->hi2c1, ADC_SOFT_MUTE_CONTROL, 0x00);
    if(actualState != HAL_OK)
        printf("HAL_KO in WriteReg: %d",actualState);
    actualState = PCM3168A_ReadReg(dsp->hi2c1, DAC_CONTROL_1, &output);
    if(actualState != HAL_OK)
        printf("HAL_KO in second ReadReg: %d",actualState);
    memset(dsp, 0, sizeof(PCM3168A_DSP));
    dsp->sample_rate = PCM3168A_SAMPLE_RATE;
    dsp->bits_per_sample = PCM3168A_BITS_PER_SAMPLE;
    
    // Initialize channels
    for(int i = 0; i < PCM3168A_NUM_CHANNELS; i++) {
        dsp->dac_channels[i].volume = 1.0f;
        dsp->dac_channels[i].src_ratio = 1.0f;
        // Initialize FIR filter coefficients with lowpass filter
        for(int j = 0; j < FILTER_TAP_NUM; j++) {
            // Simple lowpass filter coefficients - should be replaced with actual design
            dsp->dac_channels[i].filter_coeffs[j] = 1.0f / FILTER_TAP_NUM;
        }
    }
    
    for(int i = 0; i < PCM3168A_NUM_ADC; i++) {
        dsp->adc_channels[i].volume = 1.0f;
        dsp->adc_channels[i].src_ratio = 1.0f;
    }
}

// Apply FIR filter to a single sample
float PCM3168A_DSP_ApplyFIR(PCM3168A_DSP_Channel* channel, float input) {
    // Update delay line
    channel->delay_line[channel->delay_line_index] = input;
    channel->delay_line_index = (channel->delay_line_index + 1) % FILTER_TAP_NUM;
    
    float output = 0.0f;
    int index = channel->delay_line_index;
    
    // Apply FIR filter
    for(int i = 0; i < FILTER_TAP_NUM; i++) {
        output += channel->delay_line[index] * channel->filter_coeffs[i];
        index = (index + 1) % FILTER_TAP_NUM;
    }
    
    return output;
}

// Process a block of samples for DAC output
void PCM3168A_DSP_ProcessDACBlock(PCM3168A_DSP* dsp, uint32_t channel, 
                                 const int32_t* input, int32_t* output, 
                                 uint32_t num_samples) {
    PCM3168A_DSP_Channel* chan = &dsp->dac_channels[channel];
    
    for(uint32_t i = 0; i < num_samples; i++) {
        // Convert to float for processing
        float sample = (float)input[i] / (1 << (dsp->bits_per_sample - 1));
        
        // Apply DSP chain
        sample = PCM3168A_DSP_ApplyFIR(chan, sample);
        sample *= chan->volume;
        
        // Convert back to fixed point
        output[i] = (int32_t)(sample * (1 << (dsp->bits_per_sample - 1)));
    }
}

// Process a block of samples from ADC input
void PCM3168A_DSP_ProcessADCBlock(PCM3168A_DSP* dsp, uint32_t channel,
                                 const int32_t* input, int32_t* output,
                                 uint32_t num_samples) {
    PCM3168A_DSP_Channel* chan = &dsp->adc_channels[channel];
    
    for(uint32_t i = 0; i < num_samples; i++) {
        // Convert to float for processing
        float sample = (float)input[i] / (1 << (dsp->bits_per_sample - 1));
        
        // Apply DSP chain
        sample = PCM3168A_DSP_ApplyFIR(chan, sample);
        sample *= chan->volume;
        
        // Convert back to fixed point
        output[i] = (int32_t)(sample * (1 << (dsp->bits_per_sample - 1)));
    }
}

// Set channel volume (0.0 to 1.0)
void PCM3168A_DSP_SetVolume(PCM3168A_DSP* dsp, uint32_t channel, float volume) {
    if(channel < PCM3168A_NUM_CHANNELS) {
        dsp->dac_channels[channel].volume = volume;
    }
}

// Update filter coefficients for a channel
void PCM3168A_DSP_UpdateFilter(PCM3168A_DSP* dsp, uint32_t channel, 
                              const float* coeffs, uint32_t num_coeffs) {
    if(channel < PCM3168A_NUM_CHANNELS && num_coeffs <= FILTER_TAP_NUM) {
        memcpy(dsp->dac_channels[channel].filter_coeffs, coeffs, 
               num_coeffs * sizeof(float));
    }
}
void ProcessAudio(int32_t* input, int32_t* output, uint32_t size)
{
    // Simple passthrough for testing
    for(uint32_t i = 0; i < size; i++)
    {
        output[i] = input[i];
    }
}

// Initialize the frequency detector
void FrequencyDetector_Init(FrequencyDetector* detector) {
    detector->prev_sample = 0;
    detector->zero_crossings = 0;
    detector->sample_count = 0;
    detector->current_frequency = 0.0f;
}

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
  data_u = (cmd&0xf0);
  data_l = ((cmd<<4)&0xf0);
  uint8_t data_t[4];  
  data_t[0] = data_u|0x0C;  //en=1, rs=0 -> bxxxx1100
  data_t[1] = data_u|0x08;  //en=0, rs=0 -> bxxxx1000
  data_t[2] = data_l|0x0C;  //en=1, rs=0 -> bxxxx1100
  data_t[3] = data_l|0x08;  //en=0, rs=0 -> bxxxx1000
  HAL_I2C_Master_Transmit (&hi2c4, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}
void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1 -> bxxxx1101
	data_t[1] = data_u|0x09;  //en=0, rs=1 -> bxxxx1001
	data_t[2] = data_l|0x0D;  //en=1, rs=1 -> bxxxx1101
	data_t[3] = data_l|0x09;  //en=0, rs=1 -> bxxxx1001
	HAL_I2C_Master_Transmit (&hi2c4, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}
void lcd_init (void)
{
  // 4 bit initialisation
  HAL_Delay(50);  // wait for >40ms
  lcd_send_cmd (0x30);
  HAL_Delay(5);  // wait for >4.1ms
  lcd_send_cmd (0x30);
  HAL_Delay(1);  // wait for >100us
  lcd_send_cmd (0x30);
  HAL_Delay(10);
  lcd_send_cmd (0x20);  // 4bit mode
  HAL_Delay(10);

  // display initialisation
  lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
  HAL_Delay(1);
  lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
  HAL_Delay(1);
  lcd_send_cmd (0x01);  // clear display
  HAL_Delay(2);
  lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
  HAL_Delay(1);
  lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}
void lcd_send_string (char *str)
{
  while (*str) lcd_send_data (*str++);
}
void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }
    lcd_send_cmd (col);
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SAI1_Init();
  MX_I2C1_Init();
  MX_FMC_Init();
  MX_I2C4_Init();
  /* USER CODE BEGIN 2 */
  // Display Strings
  lcd_init ();
  lcd_put_cur(0, 0);
  lcd_send_string ("Starting");
  lcd_put_cur(1, 0);
  lcd_send_string("Device...");
  // Initialize DSP
  dsp.hi2c1 = &hi2c1;
  PCM3168A_DSP_Init(&dsp);
  FrequencyDetector_Init(&freq_detector);
  printf("Starting\r\n");
  // Start DMA transfers
  if(HAL_SAI_Receive_DMA(&hsai_BlockB1, DMA_RxBuffer, BUFFER_SIZE * 4) != HAL_OK)
  {
      Error_Handler();
  }
  /*if(HAL_SAI_Transmit_DMA(&hsai_BlockA1, DMA_TxBuffer, BUFFER_SIZE * 4) != HAL_OK)
  {
      Error_Handler();
  }*/
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  HAL_GPIO_TogglePin (GPIOC, GPIO_PIN_13);
	  HAL_GPIO_TogglePin (GPIOC, GPIO_PIN_14);
	  HAL_Delay (500);   /* Insert delay 100 ms */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 25;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 96;
  PeriphClkInitStruct.PLL3.PLL3P = 25;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_1;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Process samples and calculate frequency
void FrequencyDetector_Process(FrequencyDetector* detector, const int32_t* samples, uint32_t num_samples) {
    for(uint32_t i = 0; i < num_samples; i++) {
        // Detect zero crossing (positive slope)
        if (detector->prev_sample <= DETECTION_THRESHOLD && samples[i] > DETECTION_THRESHOLD) {
            detector->zero_crossings++;
        }
        
        detector->prev_sample = samples[i];
        detector->sample_count++;
        
        // Calculate frequency when we have enough samples
        if (detector->sample_count >= SAMPLE_WINDOW) {
            // Frequency = (zero crossings / 2) * (sample rate / window size)
            detector->current_frequency = (float)(detector->zero_crossings) * PCM3168A_SAMPLE_RATE / 
                                       (2.0f * SAMPLE_WINDOW);
            
            // Print frequency to debug console
            printf("Current frequency: %.2f Hz\r\n", detector->current_frequency);
            
            // Reset counters
            detector->zero_crossings = 0;
            detector->sample_count = 0;
        }
    }
}
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
    if(hsai->Instance == SAI1_Block_B)
    {
        // Convert byte buffer to int32_t buffer
        for(int i = 0; i < BUFFER_SIZE; i++) {
            ProcessRxBuffer[i] = (DMA_RxBuffer[i*4] << 24) |
                                (DMA_RxBuffer[i*4 + 1] << 16) |
                                (DMA_RxBuffer[i*4 + 2] << 8) |
                                DMA_RxBuffer[i*4 + 3];
        }
        FrequencyDetector_Process(&freq_detector, ProcessRxBuffer, BUFFER_SIZE);
        // Process received audio data
        PCM3168A_DSP_ProcessADCBlock(&dsp, 0, ProcessRxBuffer, ProcessTxBuffer, BUFFER_SIZE);
        
        // Convert processed data back to byte buffer
        for(int i = 0; i < BUFFER_SIZE; i++) {
            DMA_TxBuffer[i*4] = (ProcessTxBuffer[i] >> 24) & 0xFF;
            DMA_TxBuffer[i*4 + 1] = (ProcessTxBuffer[i] >> 16) & 0xFF;
            DMA_TxBuffer[i*4 + 2] = (ProcessTxBuffer[i] >> 8) & 0xFF;
            DMA_TxBuffer[i*4 + 3] = ProcessTxBuffer[i] & 0xFF;
        }
    }
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    if(hsai->Instance == SAI1_Block_B)
    {
        // Process first half of buffer
        // Convert byte buffer to int32_t buffer (first half)
        for(int i = 0; i < BUFFER_SIZE/2; i++) {
            ProcessRxBuffer[i] = (DMA_RxBuffer[i*4] << 24) |
                                (DMA_RxBuffer[i*4 + 1] << 16) |
                                (DMA_RxBuffer[i*4 + 2] << 8) |
                                DMA_RxBuffer[i*4 + 3];
        }

        PCM3168A_DSP_ProcessADCBlock(&dsp, 0, ProcessRxBuffer, ProcessTxBuffer, BUFFER_SIZE/2);
        
        // Convert processed data back to byte buffer (first half)
        for(int i = 0; i < BUFFER_SIZE/2; i++) {
            DMA_TxBuffer[i*4] = (ProcessTxBuffer[i] >> 24) & 0xFF;
            DMA_TxBuffer[i*4 + 1] = (ProcessTxBuffer[i] >> 16) & 0xFF;
            DMA_TxBuffer[i*4 + 2] = (ProcessTxBuffer[i] >> 8) & 0xFF;
            DMA_TxBuffer[i*4 + 3] = ProcessTxBuffer[i] & 0xFF;
        }
    }
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
