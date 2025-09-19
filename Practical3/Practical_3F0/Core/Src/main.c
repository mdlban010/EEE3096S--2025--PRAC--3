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
#include <limits.h>   // for INT32_MAX/MIN
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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
    float throughput_pixels_per_sec;
    uint32_t cpu_cycles_est;
} Task3Result;

typedef struct {
    uint16_t width;
    uint16_t height;
    uint32_t exec_time_ms;
    uint64_t checksum;
    uint8_t processed_in_parts; // 0 = single run, 1-255 = number of parts
} Task4Result;

// ===== Task 7: Fixed-Point Scaling (works on F4 and F0) =====
typedef struct {
    int32_t   scale;                  // e.g. 1000, 10000, 1000000
    uint16_t  width, height;          // image size
    uint32_t  exec_time_ms;           // wall time
    uint64_t  checksum_fixed;         // fixed-point checksum
    uint64_t  checksum_double;        // double baseline checksum
    uint64_t  abs_diff;               // |fixed - double|
    uint32_t  overflow_events;        // number of clamps detected
} Task7Result;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_ITER   100
#define SCALE      1000000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TODO: Define variables you think you might need
// - Performance timing variables (e.g execution time, throughput, pixels per second, clock cycles)
static const uint16_t max_iter_values[] = {100, 250, 500, 750, 1000};
static const uint8_t num_max_iter_tests = 5;

static const uint16_t test_sizes[][2] = {
    {128, 128}, {160, 160}, {192, 192}, {224, 224}, {256, 256}
};

volatile Task2Result task2_results[5][5];
volatile uint32_t start_time = 0, end_time = 0;
volatile uint16_t current_width  = 0;
volatile uint16_t current_height = 0;
volatile uint16_t current_max_iter = 0;
volatile uint32_t progress = 0;
volatile uint8_t task2_done = 0;

volatile Task3Result task3_results[5];
volatile uint16_t t3_current_w=0, t3_current_h=0;
volatile uint32_t t3_exec_ms=0, t3_cycles_est=0;
volatile float    t3_throughput=0.0f;

static uint8_t task3_done = 0;

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
volatile uint8_t task4_done = 0;
volatile uint16_t t4_current_w = 0, t4_current_h = 0;
volatile uint32_t t4_exec_ms = 0;
volatile uint8_t t4_parts = 0;

// ===== Task 6 (STM32F0) – Live Expressions =====
volatile uint8_t  task6_done        = 0;     // becomes 1 when the sweep finishes
volatile uint32_t t6_total_ms       = 0;     // total wall time (ms) across all sizes
volatile uint32_t t6_total_pixels   = 0;     // sum of pixels processed
volatile float    t6_throughput_pps = 0.0f;  // pixels per second
volatile uint64_t t6_checksum_sink  = 0;
// The P1B image sizes (same set you used on F0)
static const uint16_t t6_sizes[][2] = {
    {128,128}, {160,160}, {192,192}, {224,224}, {256,256}
};

#ifndef OPT_LEVEL_STR
#define OPT_LEVEL_STR "-O?"      // set from Makefile; default if not provided
#endif
const char* opt_level = OPT_LEVEL_STR;

// ---- Live Expressions to watch while Task 7 runs ----
volatile uint8_t  task7_done          = 0;
volatile int32_t  t7_scale            = 0;
volatile uint16_t t7_width            = 0, t7_height = 0;
volatile uint32_t t7_exec_ms          = 0;
volatile uint64_t t7_checksum_fixed   = 0;
volatile uint64_t t7_checksum_double  = 0;
volatile uint64_t t7_abs_diff         = 0;
volatile uint32_t t7_overflow_count   = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
//TODO: Define any function prototypes you might need such as the calculate Mandelbrot function among others
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
static uint64_t get_time_us(void);
static void run_task3_benchmark_f0(void);
static void run_task4_scalability_test(void);
void run_task6_total_runtime_f0(void);
uint64_t calculate_mandelbrot_fixed_scaled(int width, int height, int max_iterations,
                                           int32_t scale, uint32_t *overflow_ctr);
void run_task7_fixed_scale_sweep(void);

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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
	  //TODO: Visual indicator: Turn on LED0 to signal processing start
	  //TODO: Benchmark and Profile Performance
    if(task2_done){
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
    else if (task3_done) {
      // LEDs to show start/finish
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // LED0 on (start)
      run_task3_benchmark_f0();
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // LED1 on (done)
      HAL_Delay(2000);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
      task3_done = 1;
}
    else if (task4_done) {
    // Task 4: Scalability Test
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // LED2 for Task 4
    run_task4_scalability_test();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // LED3 for completion
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);
    task4_done = 1;
}
else if (!task6_done) {
        // Task 6: Compiler Optimizations Test
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        
        run_task6_optimization_test();
        
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_Delay(2000);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
        task6_done = 1;
}


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
//TODO: Function signatures you defined previously , implement them here
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

// Combine HAL_GetTick() (ms) with SysTick->VAL for ~us resolution
uint64_t get_time_us(void)
{
    uint32_t load = SysTick->LOAD + 1U;            // ticks per 1 ms
    uint32_t tick1, val1, tick2, val2;

    // read atomically
    tick1 = HAL_GetTick();
    val1  = SysTick->VAL;
    tick2 = HAL_GetTick();
    val2  = SysTick->VAL;

    // if a millisecond rolled over between reads, use the second set
    uint32_t ms;
    uint32_t val;
    if (tick2 != tick1) { ms = tick2; val = val2; }
    else                { ms = tick1; val = val1; }

    // SysTick counts DOWN from LOAD to 0 each ms
    uint32_t sub_ms_ticks = load - val;            // ticks since last ms
    uint64_t us = (uint64_t)ms * 1000ULL
                + ((uint64_t)sub_ms_ticks * 1000ULL) / load;
    return us;
}


void run_task3_benchmark_f0(void)
{
    for (int i=0; i<5; ++i) {
        uint16_t w = test_sizes[i][0];
        uint16_t h = test_sizes[i][1];
        uint32_t pixels = (uint32_t)w * (uint32_t)h;

        t3_current_w = w;
        t3_current_h = h;
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4); // progress blip

        uint64_t t0 = get_time_us();
        uint64_t checksum = calculate_mandelbrot_fixed_point_arithmetic(w, h, MAX_ITER);
        uint64_t t1 = get_time_us();

        uint32_t elapsed_ms  = (uint32_t)((t1 - t0) / 1000ULL);
        float    elapsed_s   = (float)(t1 - t0) / 1e6f;

        float throughput = (elapsed_s > 0.0f) ? (pixels / elapsed_s) : 0.0f;
        uint32_t cycles_est = (uint32_t)((t1 - t0) * (SystemCoreClock / 1000000UL));

        task3_results[i] = (Task3Result){
            .width=w, .height=h, .max_iter=MAX_ITER,
            .exec_time_ms=elapsed_ms, .checksum=checksum,
            .throughput_pixels_per_sec=throughput,
            .cpu_cycles_est=cycles_est
        };

        t3_exec_ms     = elapsed_ms;
        t3_throughput  = throughput;
        t3_cycles_est  = cycles_est;

        HAL_Delay(50);
    }
}

void run_task4_scalability_test(void) {
    for (int i = 0; i < num_task4_tests; ++i) {
        uint16_t w = task4_sizes[i][0];
        uint16_t h = task4_sizes[i][1];
        uint32_t total_pixels = (uint32_t)w * (uint32_t)h;
        
        t4_current_w = w;
        t4_current_h = h;
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5); // Progress indicator
        
        uint64_t checksum = 0;
        uint32_t start_time = HAL_GetTick();
        uint8_t parts_used = 1; // Default: process in one part
        
        // Check if we need to split due to memory constraints
        if (total_pixels > 50000) { // Arbitrary threshold for F0 memory
            parts_used = 2; // Split into 2 parts
            for (int part = 0; part < parts_used; part++) {
                int mid_point = h/2;
                
                checksum += calculate_mandelbrot_fixed_point_arithmetic(
                    w, mid_point, MAX_ITER);
                
                checksum += calculate_mandelbrot_fixed_point_arithmetic(
                    w, h - mid_point, MAX_ITER);
                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6); // Part completion blip
                HAL_Delay(10);
            }
        } else {
            // Process in one go
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
        
        HAL_Delay(100); // Short delay between tests
    }
}
void run_task6_total_runtime_f0(void)
{
    // Optional: quick LED blink to show Task 6 start (adjust port/pin to your board)
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    t6_total_ms       = 0;
    t6_total_pixels   = 0;
    t6_throughput_pps = 0.0f;
    t6_checksum_sink  = 0;

    // Measure total time across the whole 5-size sweep
    uint32_t t0 = HAL_GetTick();

    for (unsigned i = 0; i < (sizeof(t6_sizes)/sizeof(t6_sizes[0])); ++i) {
        const uint16_t w = t6_sizes[i][0];
        const uint16_t h = t6_sizes[i][1];
        t6_total_pixels += (uint32_t)w * (uint32_t)h;

        // ---- Do the work: use your existing fixed-point Mandelbrot ----
        // Must have: uint64_t calculate_mandelbrot_fixed_point_arithmetic(int w,int h,int max_iter);
        uint64_t chk = calculate_mandelbrot_fixed_point_arithmetic(w, h, 100 /* MAX_ITER */);

        // Consume result so it cannot be optimized away at -O2/-Os
        t6_checksum_sink ^= chk;

        // Optional: progress blink
        // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
    }

    uint32_t t1 = HAL_GetTick();
    t6_total_ms = (t1 - t0);

    // Derive throughput (px/s). If timing <1 ms, consider repeating the sweep N times.
    if (t6_total_ms > 0) {
        t6_throughput_pps = (float)t6_total_pixels / ((float)t6_total_ms / 1000.0f);
    } else {
        t6_throughput_pps = 0.0f;
    }

    task6_done = 1;

    // Optional: end LED
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
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
