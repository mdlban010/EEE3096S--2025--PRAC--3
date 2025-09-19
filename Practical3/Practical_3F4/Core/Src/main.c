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
#include <stdint.h>
#include <math.h>
#include "core_cm4.h"
#include <stdio.h>
#include <inttypes.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint16_t width;
    uint16_t height;
    uint32_t exec_time_ms;
    uint64_t checksum;
} Task1Result;
typedef struct {
    uint16_t width;
    uint16_t height;
    uint32_t exec_time_ms;
    uint16_t max_iter;
    uint64_t checksum;
} Task2Result;

typedef struct {
    uint16_t width;
    uint16_t height;
    uint32_t exec_time_ms;
    uint16_t max_iter;
    uint64_t checksum;
    uint32_t cpu_cycles;
    float throughput_pixels_per_sec;
} Task3Result;

typedef struct {
    uint16_t width;
    uint16_t height;
    uint32_t exec_time_ms;
    uint64_t checksum;
    uint8_t processed_in_parts; // 0 = single run, 1-255 = number of parts
} Task4Result;

typedef struct {
    uint16_t width;
    uint16_t height;
    uint32_t exec_time_ms;
    uint64_t checksum;
    uint32_t cpu_cycles;
    float throughput_pixels_per_sec;
    uint8_t fpu_enabled;
    uint8_t data_type;
} Task5Result;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_ITER   100
#define SCALE      1000000
#define CPU_FREQ   120000000 // STM32F4 at 120MHz

#ifndef FPU_BUILD
// Will be defined by Makefile targets: -DFPU_BUILD=1 (fpu_on) or 0 (fpu_off)
#define FPU_BUILD 0
#endif

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
    // Stub implementation - replace with UART code if needed
    return ch;
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
static const uint16_t max_iter_values[] = {100, 250, 500, 750, 1000};
static const uint8_t num_max_iter_tests = 5;

// Performance timing variables
static const uint16_t test_sizes[][2] = {
    {128, 128}, {160, 160}, {192, 192}, {224, 224}, {256, 256}
};

// Per-size results for each kernel
volatile Task1Result task1_fixed[5];
volatile Task1Result task1_double[5];
volatile Task2Result task2_results[5][5]; // [max_iter_index][size_index]
volatile Task3Result task3_results[5];

// Handy live variables to watch
volatile uint32_t start_time = 0, end_time = 0;
volatile uint32_t start_cycles = 0, end_cycles = 0;
volatile uint16_t current_width  = 0;
volatile uint16_t current_height = 0;
volatile uint16_t current_max_iter = 0;
volatile uint32_t current_cycles = 0;
volatile float current_throughput = 0;

volatile uint64_t checksum_fixed   = 0;
volatile uint32_t exec_time_fixed  = 0;

volatile uint64_t checksum_double  = 0;
volatile uint32_t exec_time_double = 0;

volatile uint64_t current_checksum = 0;
volatile uint32_t current_exec_time = 0;

// Progress: 0..9 (5 sizes × 2 kernels)
volatile uint32_t progress = 0;

// Task 4 variables
static const uint16_t task4_sizes[][2] = {
    {320, 240},   // QVGA
    {640, 480},   // VGA
    {800, 600},   // SVGA
    {1024, 768},  // XGA
    {1280, 720},  // HD
    {1920, 1080}  // Full HD
};
static const uint8_t num_task4_tests = 6;

volatile Task4Result task4_results[6];
volatile uint16_t t4_current_w = 0, t4_current_h = 0;
volatile uint32_t t4_exec_ms = 0;
volatile uint8_t t4_parts = 0;

// Task 5 variables
volatile Task5Result task5_results[5][2];
volatile uint8_t task5_done = 0;

volatile uint32_t t5_exec_ms = 0;
volatile uint32_t t5_cycles = 0;
volatile float    t5_throughput = 0.0f;
volatile uint8_t  t5_fpu_enabled = 0;  // build-time status copied during run
volatile uint8_t  t5_data_type = 0;    // 0 float, 1 double

// Task 6 variables (Live Expressions)
volatile uint8_t  task6_done = 0;
volatile uint32_t t6_total_ms = 0;
volatile uint32_t t6_total_cycles = 0;
volatile uint32_t t6_total_pixels = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

// Mandelbrot function prototypes
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
void dwt_init(void);
void run_task3_benchmark(void);
void run_task4_scalability_test_f4(void);

//// TASK -5
//void run_task5_fpu_test(void);
//uint64_t calculate_mandelbrot_float(int width, int height, int max_iterations);
//uint64_t calculate_mandelbrot_double_nofpu(int width, int height, int max_iterations);
//uint64_t calculate_mandelbrot_float_nofpu(int width, int height, int max_iterations);
//void enable_fpu(void);
//void disable_fpu(void);
//uint8_t is_fpu_enabled(void);
//void run_task5_fpu_test(void);
//uint64_t calculate_mandelbrot_float_fpu(int width, int height, int max_iterations);
//uint64_t calculate_mandelbrot_double_fpu(int width, int height, int max_iterations);
//static inline uint32_t cycles_delta(uint32_t start, uint32_t end);
uint64_t calculate_mandelbrot_float(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
void enable_fpu(void);
void disable_fpu(void);
uint8_t is_fpu_enabled(void);
void run_task5_fpu_test(void);

static inline uint32_t cycles_delta(uint32_t start, uint32_t end);
void run_task6_total_runtime(void);

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  dwt_init(); // calling dwt_init before running task3
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  static uint8_t task1_done = 1;
	  static uint8_t task2_done = 1;
	  static uint8_t task3_done = 1;
	  static uint8_t task4_done = 1;
    if (!task1_done){
      // Visual indicator: Turn on LED0 to signal processing start
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

      // Benchmark and Profile Performance for both implementations
      for (int i = 0; i < 5; i++) {
        current_width = test_sizes[i][0];
        current_height = test_sizes[i][1];
        
        // Fixed-point arithmetic implementation
        start_time = HAL_GetTick();
        checksum_fixed = calculate_mandelbrot_fixed_point_arithmetic(current_width, current_height, MAX_ITER);
        end_time = HAL_GetTick();
        exec_time_fixed = end_time - start_time;
        
        task1_fixed[i].width = current_width;
        task1_fixed[i].height = current_height;
        task1_fixed[i].exec_time_ms = exec_time_fixed;
        task1_fixed[i].checksum = checksum_fixed;
        
        progress++;
        
        // Double implementation
        start_time = HAL_GetTick();
        checksum_double = calculate_mandelbrot_double(current_width, current_height, MAX_ITER);
        end_time = HAL_GetTick();
        exec_time_double = end_time - start_time;
        
        task1_double[i].width = current_width;
        task1_double[i].height = current_height;
        task1_double[i].exec_time_ms = exec_time_double;
        task1_double[i].checksum = checksum_double;
        
        progress++;
      }

      // Visual indicator: Turn on LED1 to signal processing completion
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

      // Keep the LEDs ON for 2s
      HAL_Delay(2000);

      // Turn OFF LEDs
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

      task1_done = 1;
    }

    else if(!task2_done){
      // Task 2: Test different MAX_ITER values
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // LED2 for Task 2
    
    for (int iter_idx = 0; iter_idx < num_max_iter_tests; iter_idx++) {
        current_max_iter = max_iter_values[iter_idx];
        
        for (int size_idx = 0; size_idx < 5; size_idx++) {
            current_width = test_sizes[size_idx][0];
            current_height = test_sizes[size_idx][1];
            
            // Fixed-point implementation
            start_time = HAL_GetTick();
            uint64_t checksum = calculate_mandelbrot_fixed_point_arithmetic(
                current_width, current_height, current_max_iter);
            end_time = HAL_GetTick();
            
            task2_results[iter_idx][size_idx].width = current_width;
            task2_results[iter_idx][size_idx].height = current_height;
            task2_results[iter_idx][size_idx].max_iter = current_max_iter;
            task2_results[iter_idx][size_idx].exec_time_ms = end_time - start_time;
            task2_results[iter_idx][size_idx].checksum = checksum;
            
            progress++;
        }
    }
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // LED3 for completion
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    
    task2_done = 1;
    }

    else if (!task3_done) {
        // Visual indicator: Turn on LED0 to signal Task 3 start
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        
        // Run Task 3: Extended execution time measurement
        run_task3_benchmark();
        
        // Visual indicator: Turn on LED1 to signal completion
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        
        // Keep LEDs on for 2 seconds
        HAL_Delay(2000);
        
        // Turn off LEDs
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
        
        task3_done = 1;
    }
    else if (!task4_done) {
    // Task 4: Scalability Test
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // LED2 for Task 4
    run_task4_scalability_test_f4();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // LED3 for completion
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);
    task4_done = 1;
}
    else if (task5_done) {
        // Task 5: FPU Impact Test (STM32F4 only)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        printf("\r\n=== Task 5: FPU Impact Test ===\r\n");
        
        run_task5_fpu_test();
        
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        printf("=== Task 5 Complete ===\r\n");
        HAL_Delay(2000);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
        task5_done = 1;
    }
    // heartbeat led to show program is alive
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
    HAL_Delay(1000);
  }
  else if (!task6_done) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // start LED
    run_task6_total_runtime();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // end LED
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
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Mandelbrot implementations
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations){
  uint64_t mandelbrot_sum = 0;
    for (int y = 0; y < height; y++){
      for(int x = 0; x < width; x++){
        int32_t x0 = ((int64_t)x *3500000)/width-2500000;
        int32_t y0 = ((int64_t)y *2000000)/height-1000000;

        int32_t xi = 0, yi = 0;
        int iterations = 0;

        while(iterations < max_iterations){
          int64_t xi2 = ((int64_t)xi * xi) / SCALE;
          int64_t yi2 = ((int64_t)yi * yi) / SCALE;

          if(xi2 + yi2 > 4000000) break;
          int32_t temp = xi2 - yi2;
          yi = (2 * (int64_t)xi * yi) / SCALE + y0;
          xi = temp + x0;
          iterations++;
        }
        mandelbrot_sum += iterations;
      }
    }
    return mandelbrot_sum;
}

uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations){
    uint64_t mandelbrot_sum = 0;
    for (int y = 0; y < height; y++){
      for(int x = 0; x < width; x++){
        double x0 = ((double)x/width)*3.5-2.5;
        double y0 = ((double)y/height)*2.0-1.0;
        double xi = 0.0, yi = 0.0;
        int iterations = 0;

        while(iterations < max_iterations && (xi * xi + yi * yi) <=4.0){
          double temp = (xi * xi- yi * yi);
          yi = 2.0* xi * yi + y0;
          xi = temp + x0;
          iterations++;
        }
        mandelbrot_sum += iterations;
      }
    }
    return mandelbrot_sum;
}
/**
 * @brief Initialize DWT cycle counter for accurate timing
 */
void dwt_init(void)
{
    /* Enable trace and debug blocks (needed for DWT) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Reset and enable the cycle counter */
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief Run Task 3 benchmark with extended timing measurements
 */
void run_task3_benchmark(void)
{
    progress = 0;
    
    // Test all 5 image sizes from Practical 1B
    for (unsigned i = 0; i < (sizeof(test_sizes) / sizeof(test_sizes[0])); i++)
    {
        current_max_iter = MAX_ITER;
        uint16_t width = test_sizes[i][0];
        uint16_t height = test_sizes[i][1];
        uint32_t total_pixels = width * height;
        
        // Update current test info for debugger viewing
        current_width = width;
        current_height = height;
        
        // Progress indication
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
        
        // Reset and start measurements
        DWT->CYCCNT = 0;  // Reset cycle counter
        start_cycles = DWT->CYCCNT;
        start_time = HAL_GetTick();
        
        // Execute Mandelbrot calculation
        uint64_t checksum = calculate_mandelbrot_fixed_point_arithmetic(width, height, MAX_ITER);
        
        // Stop measurements
        end_time = HAL_GetTick();
        end_cycles = DWT->CYCCNT;
        
        // Calculate metrics
        uint32_t exec_time_ms = end_time - start_time;
        uint32_t cpu_cycles = end_cycles - start_cycles;
        
        // Calculate throughput (pixels per second)
        float throughput = 0.0f;
        if (exec_time_ms > 0) {
            // Convert ms to seconds and calculate pixels/sec
            throughput = (float)total_pixels / ((float)exec_time_ms / 1000.0f);
        }
        
        // Store results
        task3_results[i] = (Task3Result){
            .width = width,
            .height = height,
            .exec_time_ms = exec_time_ms,
            .cpu_cycles = cpu_cycles,
            .throughput_pixels_per_sec = throughput,
            .checksum = checksum,
            .max_iter = MAX_ITER
        };
        
        // Update current values for debugger viewing
        current_checksum = checksum;
        current_exec_time = exec_time_ms;
        current_cycles = cpu_cycles;
        current_throughput = throughput;
        
        // Brief pause between tests
        HAL_Delay(100);
    }
    
    // Turn off progress LEDs
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4, GPIO_PIN_RESET);
}

// task 4 method for scalability
void run_task4_scalability_test_f4(void) {
    for (int i = 0; i < num_task4_tests; ++i) {
        uint16_t w = task4_sizes[i][0];
        uint16_t h = task4_sizes[i][1];
        uint32_t total_pixels = (uint32_t)w * (uint32_t)h;
        
        t4_current_w = w;
        t4_current_h = h;
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5); // Progress indicator
        
        uint64_t checksum = 0;
        uint32_t start_time = HAL_GetTick();
        uint8_t parts_used = 1;
        
        // STM32F4 has more memory, so we handle larger images
        // But for very large images (2K+), we might need to split
        if (total_pixels > 2000000) { // 2MPixels threshold for F4
            parts_used = 2;
            // Split into two halves
            int mid_point = h / 2;
            checksum += calculate_mandelbrot_fixed_point_arithmetic(w, mid_point, MAX_ITER);
            checksum += calculate_mandelbrot_fixed_point_arithmetic(w, h - mid_point, MAX_ITER);
        } else {
            // Process in one go - F4 can handle most sizes
            checksum = calculate_mandelbrot_fixed_point_arithmetic(w, h, MAX_ITER);
        }
        
        uint32_t end_time = HAL_GetTick();
        uint32_t elapsed_ms = end_time - start_time;
        
        task4_results[i] = (Task4Result){
            .width = w,
            .height = h,
            .exec_time_ms = elapsed_ms,
            .checksum = checksum,
            .processed_in_parts = parts_used
        };
        
        t4_exec_ms = elapsed_ms;
        t4_parts = parts_used;
        
        HAL_Delay(50); // Short delay between tests
    }
}
// Mandelbrot implementation using float (with FPU)
uint64_t calculate_mandelbrot_float(int width, int height, int max_iterations) {
    uint64_t mandelbrot_sum = 0;
    for (int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++) {
            float x0 = ((float)x/width)*3.5f-2.5f;
            float y0 = ((float)y/height)*2.0f-1.0f;
            float xi = 0.0f, yi = 0.0f;
            int iterations = 0;

            while(iterations < max_iterations && (xi * xi + yi * yi) <= 4.0f) {
                float temp = xi * xi - yi * yi;
                yi = 2.0f * xi * yi + y0;
                xi = temp + x0;
                iterations++;
            }
            mandelbrot_sum += iterations;
        }
    }
    return mandelbrot_sum;
}
uint64_t calculate_mandelbrot_float_fpu(int width, int height, int max_iterations) {
    uint64_t mandelbrot_sum = 0;
    for (int y = 0; y < height; y++) {
        // Progress indicator
        if ((y & 15) == 0) {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
        }
        
        for(int x = 0; x < width; x++) {
            float x0 = ((float)x/(float)width)*3.5f-2.5f;
            float y0 = ((float)y/(float)height)*2.0f-1.0f;
            float xi = 0.0f, yi = 0.0f;
            int iterations = 0;

            while(iterations < max_iterations && (xi * xi + yi * yi) <= 4.0f) {
                float temp = xi * xi - yi * yi;
                yi = 2.0f * xi * yi + y0;
                xi = temp + x0;
                iterations++;
            }
            mandelbrot_sum += iterations;
        }
    }
    return mandelbrot_sum;
}
// Double implementation assuming FPU enabled
uint64_t calculate_mandelbrot_double_fpu(int width, int height, int max_iterations) {
    uint64_t mandelbrot_sum = 0;
    for (int y = 0; y < height; y++) {
        // Progress indicator
        if ((y & 15) == 0) {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
        }
        
        for(int x = 0; x < width; x++) {
            double x0 = ((double)x/(double)width)*3.5-2.5;
            double y0 = ((double)y/(double)height)*2.0-1.0;
            double xi = 0.0, yi = 0.0;
            int iterations = 0;

            while(iterations < max_iterations && (xi * xi + yi * yi) <= 4.0) {
                double temp = xi * xi - yi * yi;
                yi = 2.0 * xi * yi + y0;
                xi = temp + x0;
                iterations++;
            }
            mandelbrot_sum += iterations;
        }
    }
    return mandelbrot_sum;
}

// FPU control functions
void enable_fpu(void) {
    // Enable CP10 and CP11 (FPU coprocessors)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
    __DSB();
    __ISB();
}

void disable_fpu(void) {
    // Disable CP10 and CP11 (FPU coprocessors)
    SCB->CPACR &= ~((3UL << 10*2)|(3UL << 11*2));
    __DSB();
    __ISB();
}

uint8_t is_fpu_enabled(void) {
    return ((SCB->CPACR & ((3UL << 10*2)|(3UL << 11*2))) != 0) ? 1 : 0;
}


/**
 * @brief Run Task 5 FPU impact test
 * Tests all 4 combinations: float/double with FPU enabled/disabled
 */
void run_task5_fpu_test(void)
{
    dwt_init();

    t5_fpu_enabled = (FPU_BUILD ? 1 : 0); // reflect build-time FPU state in live expressions

    const uint16_t sizes[5][2] = {
        {128,128}, {160,160}, {192,192}, {224,224}, {256,256}
    };

    for (int size_idx = 0; size_idx < 5; ++size_idx) {
        uint16_t w = sizes[size_idx][0];
        uint16_t h = sizes[size_idx][1];
        uint32_t total_pixels = (uint32_t)w * (uint32_t)h;

        // --- FLOAT run ---
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
        DWT->CYCCNT = 0;
        uint32_t c0 = DWT->CYCCNT;
        uint32_t t0 = HAL_GetTick();

        uint64_t chk_f = calculate_mandelbrot_float(w, h, MAX_ITER);

        uint32_t t1 = HAL_GetTick();
        uint32_t c1 = DWT->CYCCNT;

        uint32_t ms_f = t1 - t0;
        uint32_t cyc_f = c1 - c0;
        float thr_f = (ms_f > 0) ? ((float)total_pixels / ((float)ms_f / 1000.0f)) : 0.0f;

        task5_results[size_idx][0] = (Task5Result){
            .width = w, .height = h,
            .exec_time_ms = ms_f,
            .checksum = chk_f,
            .cpu_cycles = cyc_f,
            .throughput_pixels_per_sec = thr_f,
            .fpu_enabled = t5_fpu_enabled,
            .data_type = 0  // float
        };

        t5_exec_ms = ms_f;
        t5_cycles = cyc_f;
        t5_throughput = thr_f;
        t5_data_type = 0;

        HAL_Delay(30);

        // --- DOUBLE run ---
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
        DWT->CYCCNT = 0;
        c0 = DWT->CYCCNT;
        t0 = HAL_GetTick();

        uint64_t chk_d = calculate_mandelbrot_double(w, h, MAX_ITER);

        t1 = HAL_GetTick();
        c1 = DWT->CYCCNT;

        uint32_t ms_d = t1 - t0;
        uint32_t cyc_d = c1 - c0;
        float thr_d = (ms_d > 0) ? ((float)total_pixels / ((float)ms_d / 1000.0f)) : 0.0f;

        task5_results[size_idx][1] = (Task5Result){
            .width = w, .height = h,
            .exec_time_ms = ms_d,
            .checksum = chk_d,
            .cpu_cycles = cyc_d,
            .throughput_pixels_per_sec = thr_d,
            .fpu_enabled = t5_fpu_enabled,
            .data_type = 1  // double
        };

        t5_exec_ms = ms_d;
        t5_cycles = cyc_d;
        t5_throughput = thr_d;
        t5_data_type = 1;

        HAL_Delay(50);
    }

    task5_done = 1;
}


// Return cycles elapsed between two reads (handles 32-bit wrap naturally)
static inline uint32_t cycles_delta(uint32_t start, uint32_t end) {
    return (uint32_t)(end - start);
}

void run_task6_total_runtime(void)
{
    // Ensure DWT is ready
    dwt_init();

    // Use the same sizes as Practical 1B (you already have test_sizes[])
    uint32_t pixels_sum = 0;
    uint64_t checksum_sum = 0; // optional: lets you sanity-check results

    // Start “whole program” timing
    DWT->CYCCNT = 0;
    uint32_t c0 = DWT->CYCCNT;
    uint32_t t0 = HAL_GetTick();

    for (unsigned i = 0; i < (sizeof(test_sizes)/sizeof(test_sizes[0])); ++i) {
        uint16_t w = test_sizes[i][0];
        uint16_t h = test_sizes[i][1];
        pixels_sum += (uint32_t)w * (uint32_t)h;

        // Use one consistent kernel (fixed-point is fine) and MAX_ITER=100
        uint64_t chk = calculate_mandelbrot_fixed_point_arithmetic(w, h, MAX_ITER);
        checksum_sum += chk;

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6); // progress blink
    }

    uint32_t t1 = HAL_GetTick();
    uint32_t c1 = DWT->CYCCNT;

    t6_total_ms     = t1 - t0;
    t6_total_cycles = c1 - c0;
    t6_total_pixels = pixels_sum;

    // optional: you can also compute overall throughput if you want to watch it
    // float t6_throughput = (t6_total_ms>0) ? (float)pixels_sum / (t6_total_ms/1000.0f) : 0.0f;

    task6_done = 1;
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
