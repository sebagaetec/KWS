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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "audio_sample.h" //right audio sample
#include "mfcc_right_sample_uint8.h"

#include <stdio.h>
#include <string.h>
#include "kws.h"
#include "kws_data.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// parametros del modelo
#define AI_INPUT_SIZE       AI_KWS_IN_1_SIZE // 490 (49*10)
#define AI_OUTPUT_SIZE      AI_KWS_OUT_1_SIZE // 12 labels



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// variables relacionadas al modelo
static ai_handle kws_model = AI_HANDLE_NULL;

AI_ALIGNED(4)
static ai_u8 kws_activations[AI_KWS_DATA_ACTIVATIONS_SIZE];


//buffer de entrada y salida
ai_buffer ai_input[AI_KWS_IN_NUM];
ai_buffer ai_output[AI_KWS_OUT_NUM];


//punteros a las variables de entrada y salida del modelo
//AI_ALIGNED(4) static uint8_t input[490];
AI_ALIGNED(4) static uint8_t output[12];

// labels
const char* kws_labels[AI_OUTPUT_SIZE] = {
    "silence", "unknown", "yes", "no", "up", "down",
    "left", "right", "on", "off", "stop", "go"
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

// Print strings via uart
void print_msg(const char* msg) {
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// modelo
void ai_init(void);
void ai_run_inference(void);

void print_free_ram(void) {
    extern uint32_t _estack, _sbss, _ebss;
    uint32_t free_ram = (uint32_t)&_estack - (uint32_t)&_ebss;
    char msg[64];
    sprintf(msg, "Free RAM: %lu bytes\r\n", free_ram);
    print_msg(msg);
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void debug_print_buffer_info(const ai_buffer* buf, const char* label) {
    if (!buf->shape.data) {
        print_msg("Error: shape.data is NULL!\r\n");
        return;
    }

    char msg[256];
    sprintf(msg,
        "%s buffer info:\r\n"
        "  Format = 0x%x\r\n"
        "  Size   = %lu elements\r\n"
        "  Shape  = [%u, %u, %u, %u]\r\n",
        label,
        buf->format,
        buf->size,
        buf->shape.data[0],
        buf->shape.data[1],
        buf->shape.data[2],
        buf->shape.data[3]
    );
    print_msg(msg);
}




void ai_init(void) {
    ai_error ai_err;
    print_msg("Initializing AI model...\r\n");

    // pesos y activacines
    const ai_handle kws_weights = ai_kws_data_weights_get();
    const ai_handle activations[] = { kws_activations };

    ai_err = ai_kws_create_and_init(&kws_model, activations, kws_weights);
        if (ai_err.type != AI_ERROR_NONE) {
            char msg[100];
            sprintf(msg, "Error creating and initializing AI model: type=%d, code=%d\r\n", ai_err.type, ai_err.code);
            print_msg(msg);
            Error_Handler();
    }

    ai_network_report report;
    if (!ai_kws_get_info(kws_model, &report)) {
        print_msg("Error getting model report info.\r\n");
        Error_Handler();
    }

    ai_buffer* input_buf = report.inputs;
    ai_buffer* output_buf = report.outputs;

    // Now we can print shape safely
    debug_print_buffer_info(input_buf, "INPUT (from report)");
    debug_print_buffer_info(output_buf, "OUTPUT (from report)");

    //static ai_shape_dimension input_shape_dims[4] = {1, 1, 10, 49};
    //static ai_shape_dimension output_shape_dims[4] = {1, 12, 1, 1};
    static ai_shape_dimension input_shape_dims[] = {1, 49, 10, 1};
    static ai_shape_dimension output_shape_dims[] = {1, 1, 1, 12};
    // Setup input buffer
    ai_input[0].format = AI_BUFFER_FORMAT_U8 | AI_BUFFER_FMT_FLAG_IS_IO;
    ai_input[0].data = AI_HANDLE_PTR(g_right_mfcc_sample_uint8);
    ai_input[0].meta_info = NULL;
    ai_input[0].flags = 0;
    ai_input[0].size = AI_KWS_IN_1_SIZE;
    ai_input[0].shape = (ai_buffer_shape) {
        .type = 0,
        .size = 4,
        .data = input_shape_dims
    };

    // Setup output buffer
    ai_output[0].format = AI_BUFFER_FORMAT_U8  | AI_BUFFER_FMT_FLAG_IS_IO;
    ai_output[0].data = AI_HANDLE_PTR(output);
    ai_output[0].meta_info = NULL;
    ai_output[0].flags = 0;
    ai_output[0].size = AI_KWS_OUT_1_SIZE;
    ai_output[0].shape = (ai_buffer_shape) {
        .type = 0,
        .size = 4,
        .data = output_shape_dims
    };

    print_msg("Modelo correctamente inicializado.\r\n");
    print_msg("----------------------------------\r\n");
}

void ai_run_inference(void) {
    if (!ai_input[0].data) {
        print_msg("Error: ai_input[0].data is NULL!\r\n");
        Error_Handler();
    }

    print_msg("Verifying input tensor shape...\r\n");
    debug_print_buffer_info(&ai_input[0], "INPUT");
    debug_print_buffer_info(&ai_output[0], "OUTPUT");


    char msg2[128];
    sprintf(msg2, "Input buffer pointer: %p\r\n", ai_input[0].data);
    print_msg(msg2);

    char msg0[128];
    sprintf(msg0, "Activations ptr=%p size=%d\r\n", kws_activations, AI_KWS_DATA_ACTIVATIONS_SIZE);
    print_msg(msg0);

    char msgg[128];
    sprintf(msgg, "ai_input[0].data address: %p\r\n", ai_input[0].data);
    print_msg(msgg);

    for (int i = 0; i < 10; i++) {
        //char msg[64];
        sprintf(msgg, "ai_input[%d] = %u\r\n", i, ((uint8_t*)ai_input[0].data)[i]);
        print_msg(msgg);
    }


    print_msg("Running the model...\r\n");
    if (ai_kws_run(kws_model, ai_input, ai_output) != 1) {
        ai_error err = ai_kws_get_error(kws_model);
        char msg[128];
        sprintf(msg, "Error running AI model. Type: %d Code: %d\r\n", err.type, err.code);
        print_msg(msg);
        //Error_Handler();
    }

    print_msg("Buscando similitud...\r\n");
    // 4. Find the keyword with the highest score
    const float out_scale = 0.00390625f;  // 1/256
    const int out_zp = 0;

    int max_idx = 0;
    float max_val = -1.0f;

    for (int i = 0; i < 12; i++) {
        float score = ((int)output[i] - out_zp) * out_scale;
        if (score > max_val) {
            max_val = score;
            max_idx = i;
        }
    }
    char msg[128];
    sprintf(msg, "Prediction: %s (Score: %.2f)\r\n", kws_labels[max_idx], max_val);
    print_msg(msg);

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialize the AI model
  print_msg("RAM before ai_init\r\n");
  print_free_ram();
  ai_init();

  print_msg("\r\n--- KWS Test Application Ready ---\r\n");
  print_msg("Running inference on test audio file...\r\n");
  print_msg("--\r\n");

  // Run inference on the test data
  print_msg("RAM Before ai_run_inference\r\n");
  print_free_ram();
  ai_run_inference();

  print_msg("\r\n--- Test complete. Entering infinite loop. ---\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	HAL_Delay(5000);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
