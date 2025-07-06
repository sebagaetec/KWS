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
#include "audio_sample.h" //right audio sample

#include <stdio.h>
#include <string.h>
#include "kws.h"
#include "kws_data.h"

//mel filter bank and dct matrix
#include "mel_filter_bank_and_dct_coeff_matrix.h"

// del DSP
#define ARM_MATH_CM4
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Audio and mfcc Parameters
#define SAMPLING_RATE       16000 // estandar
#define AUDIO_BUFFER_SIZE   SAMPLING_RATE //porque los audios son de 1 segundo
#define N_FFT               640
#define HOP_LENGTH          320
#define N_MELS              40 // parametro acorde al que usan en el modelo ds_cnn
#define N_MFCC              10
//#define N_FRAMES            49 // (AUDIO_BUFFER_SIZE - N_FFT) / HOP_LENGTH + 1
#define N_FRAMES ((AUDIO_BUFFER_SIZE - N_FFT) / HOP_LENGTH + 1)



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
static ai_network_report kws_report;

// manejo de activaciones y pesos
AI_ALIGNED(4)static ai_u8 kws_activations[AI_KWS_DATA_ACTIVATIONS_SIZE];
//AI_ALIGNED(4) __attribute__((aligned(4))) static ai_u8 kws_activations[AI_KWS_DATA_ACTIVATIONS_SIZE];
//static ai_u8 kws_activations[AI_KWS_DATA_ACTIVATIONS_SIZE] __attribute__((aligned(16)));
//__attribute__((section(".activations"))) AI_ALIGNED(16) static ai_u8 kws_activations[AI_KWS_DATA_ACTIVATIONS_SIZE];
ai_handle kws_weights = NULL; //se definira dentro de ai_init()


//const ai_handle weights = ai_kws_data_weights_get();

//buffer de entrada y salida
static ai_buffer ai_input[AI_KWS_IN_NUM] = { AI_BUFFER_INIT(
    AI_BUFFER_FORMAT_FLOAT,
    AI_KWS_IN_1_HEIGHT, AI_KWS_IN_1_WIDTH, AI_KWS_IN_1_CHANNEL, 1,
    AI_HANDLE_NULL) };
static ai_buffer ai_output[AI_KWS_OUT_NUM] = { AI_BUFFER_INIT(
    AI_BUFFER_FORMAT_FLOAT,
    AI_KWS_OUT_1_HEIGHT, AI_KWS_OUT_1_WIDTH, AI_KWS_OUT_1_CHANNEL, 1,
    AI_HANDLE_NULL) };

//punteros a las variables de entrada y salida del modelo
static float *ai_input_buffer;
static float *ai_output_buffer;

// labels
const char* kws_labels[AI_OUTPUT_SIZE] = {
    "silence", "unknown", "yes", "no", "up", "down",
    "left", "right", "on", "off", "stop", "go"
};


// Buffer para el MFCC final
float32_t g_mfcc_output_buffer[N_FRAMES * N_MFCC] __attribute__((aligned(4)));; // 49 * 10 = 490

// buffers intermedios
float32_t g_fft_buffer[N_FFT] __attribute__((aligned(4)));;
float32_t g_power_spectrum_buffer[N_FFT] __attribute__((aligned(4)));;
float32_t g_mel_energies_buffer[N_MELS] __attribute__((aligned(4)));;

// CMSIS-DSP instances for FFT
arm_rfft_fast_instance_f32 fft_instance;
//arm_rfft_instance_f32 fft_instance;
//arm_cfft_radix4_instance_f32 cfft_instance;

//matrices de mel filter bank y hann dct en header
//mel_filter_bank_and_dct_coeff_matrix.h


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

// preprocesamiento del audio
void preprocess_audio_to_mfcc(const int16_t* audio_in, const uint32_t audio_len, float32_t* mfcc_out);

// Funcion load test data
//void load_test_audio_data(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ai_init(void) {
    ai_error ai_err;
    print_msg("Initializing AI model...\r\n");

    // pesos y params
    kws_weights = ai_kws_data_weights_get();

    ai_network_params ai_params = {
          AI_KWS_DATA_WEIGHTS(kws_weights),
          AI_KWS_DATA_ACTIVATIONS(kws_activations)
     };

    // Se crea el modelo
    ai_err = ai_kws_create(&kws_model, AI_KWS_DATA_CONFIG);
    if (ai_err.type != AI_ERROR_NONE) {
        char msg[100];
        sprintf(msg, "Error creating the model: type=%d, code=%d\r\n", ai_err.type, ai_err.code);
        print_msg(msg);
        Error_Handler();
    }

    // Inicializa el modelo con los parametros
    if (!ai_kws_init(kws_model, &ai_params)) {
    	// notificar error
    	char test_msg[] = "error model init\r\n";
    	HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), HAL_MAX_DELAY);
        Error_Handler();
    }

    // Get pointers to the model's IO buffers
    ai_input_buffer = (float*)ai_kws_inputs_get(kws_model, NULL)[0].data;
    ai_output_buffer = (float*)ai_kws_outputs_get(kws_model, NULL)[0].data;

    // Initialize CMSIS-DSP instances
    //arm_cfft_radix4_init_f32(&cfft_instance, N_FFT / 2, 0, 1);
    arm_rfft_fast_init_f32(&fft_instance, N_FFT);
    //arm_rfft_init_f32(&fft_instance, N_FFT, 0, 1);
    //arm_rfft_init_f32(&fft_instance, &cfft_instance, N_FFT, 0, 1);

    print_msg("modelo correctamente inicializado.\r\n");
}

void preprocess_audio_to_mfcc(const int16_t* audio_in, const uint32_t audio_len, float32_t* mfcc_out) {

	print_msg("Procesando frame by frame...\r\n");
    for (int frame_idx = 0; frame_idx < N_FRAMES; ++frame_idx) {
        int frame_start = frame_idx * HOP_LENGTH;

        // 1. Get frame, convert to float, normalize, and apply Hann window.
        // optimizacion
        for (int i = 0; i < N_FFT; ++i) {
            int16_t sample = 0;
            uint32_t sample_idx = frame_start + i;


            if (sample_idx < audio_len) {
                sample = audio_in[sample_idx];
            }

            float float_sample = (float32_t)sample / 32768.0f;
            g_fft_buffer[i] = float_sample * hann_window[i];
        }

        print_msg("Computando FFT...\r\n");

        // 2. FFT
        arm_rfft_fast_f32(&fft_instance, g_fft_buffer, g_power_spectrum_buffer, 0);
        //arm_rfft_f32(&fft_instance, g_fft_buffer, g_power_spectrum_buffer);

        // 3. Power Spectrum
        arm_cmplx_mag_squared_f32(g_power_spectrum_buffer, g_power_spectrum_buffer, N_FFT / 2 + 1);

        print_msg("Mel Filter Bank...\r\n");
        // 4. Mel Filterbank
        for (int mel_idx = 0; mel_idx < N_MELS; ++mel_idx) {
            float sum = 0.0f;
            for (int spec_idx = 0; spec_idx < (N_FFT / 2 + 1); ++spec_idx) {
                sum += g_power_spectrum_buffer[spec_idx] * mel_filterbank[spec_idx * N_MELS + mel_idx];
            }
            g_mel_energies_buffer[mel_idx] = sum;
        }

        // 5. Log Mel Energies (librosa.power_to_db)

        for (int i = 0; i < N_MELS; ++i) {
            g_mel_energies_buffer[i] = 10.0f * log10f(g_mel_energies_buffer[i] + 1e-6f);
        }

        print_msg("Computando MFCCs via DCT...\r\n");
        // 6. MFCCs via DCT
                for (int i = 0; i < N_MFCC; ++i) {
                    float sum = 0.0f;
                    for (int j = 0; j < N_MELS; ++j) {
                        sum += dct_matrix[i * N_MELS + j] * g_mel_energies_buffer[j];
                    }
                    // Temporarily store the result in the fft buffer to avoid overwriting mel energies
                    g_fft_buffer[i] = sum;
                }

        print_msg("Guardando los coeficientes del frame...\r\n");

        memcpy(&mfcc_out[frame_idx * N_MFCC], g_fft_buffer, N_MFCC * sizeof(float32_t));
    }
}

void ai_run_inference(void) {
    char msg[64];

    print_msg("Preprocesando...\r\n");

    preprocess_audio_to_mfcc(g_right_audio_sample, G_RIGHT_AUDIO_SAMPLE_LEN, g_mfcc_output_buffer);

    print_msg("Copiando la data al input buffer...\r\n");

    memcpy(ai_input_buffer, g_mfcc_output_buffer, sizeof(float) * AI_INPUT_SIZE);

    ai_input[0].data = AI_HANDLE_PTR(ai_input_buffer);
    ai_output[0].data = AI_HANDLE_PTR(ai_output_buffer);

    ai_input[0].size = AI_INPUT_SIZE;
    ai_output[0].size = AI_OUTPUT_SIZE;


    /*
    char dbg_msg[100];
    sprintf(dbg_msg, "Input: fmt=%08X size=%d ptr=%p\r\n",
            ai_input[0].format, ai_input[0].size, ai_input[0].data);
    print_msg(dbg_msg);

    char dbg_msg2[100];
    sprintf(dbg_msg2, "Output: fmt=%08X size=%d ptr=%p\r\n",
            ai_output[0].format, ai_output[0].size, ai_output[0].data);
    print_msg(dbg_msg2);

    char dbg_msg3[100];
    for (int i = 0; i < 5; i++) {
        sprintf(dbg_msg3, "MFCC[%d]: %.3f\r\n", i, ai_input_buffer[i]);
        print_msg(dbg_msg3);
    }*/

    // 3. Run the model
    /*
    print_msg("Corriendo el modelo...\r\n");
    if (ai_kws_run(kws_model, &ai_input[0], &ai_output[0]) != 1) {
        print_msg("Error running AI model.\r\n");
        Error_Handler();
    }*/
    char msg6[128];
    char msg7[128];
    sprintf(msg6, "Input ptr=%p size=%d\n", ai_input_buffer, AI_INPUT_SIZE);
    print_msg(msg6);
    sprintf(msg7, "Output ptr=%p size=%d\n", ai_output_buffer, AI_OUTPUT_SIZE);
    print_msg(msg7);

    char msg5[128];
    sprintf(msg5, "Activations ptr=%p size=%d\r\n", kws_activations, AI_KWS_DATA_ACTIVATIONS_SIZE);
    print_msg(msg5);

    print_msg("Corriendo el modelo...\r\n");
    if (ai_kws_run(kws_model, &ai_input[0], &ai_output[0]) != 1) {
        ai_error err = ai_kws_get_error(kws_model);
        char msg[128];
        sprintf(msg, "Error running AI model. Type: %d Code: %d\r\n", err.type, err.code);
        print_msg(msg);
        Error_Handler();
    }

    print_msg("Buscando similitud...\r\n");
    // 4. Find the keyword with the highest score
    int max_idx = 0;
    float max_val = -FLT_MAX;
    int i;
    for (i = 0; i < AI_OUTPUT_SIZE; i++) {
        if (ai_output_buffer[i] > max_val) {
            max_val = ai_output_buffer[i];
            max_idx = i;
        }
    }

    // 5. Print the result
    sprintf(msg, "Prediction: %-8s (Score: %.2f)\r\n", kws_labels[max_idx], max_val);
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
  ai_init();

  print_msg("\r\n--- KWS Test Application Ready ---\r\n");
  print_msg("Running inference on test audio file...\r\n");

  // Run inference on the test data
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
