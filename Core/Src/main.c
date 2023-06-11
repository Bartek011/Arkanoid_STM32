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
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PLATFORM_LVL 47
#define st_JOY1 joystick1[1]
#define st_JOY0 joystick1[0]
#define nd_JOY1 joystick2[1]
#define nd_JOY0 joystick2[0]
#define JOYSTICK_DELAY 15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t joystick1[2];
uint16_t joystick2[2];
uint8_t UartMessage[32];
uint16_t joystick_left = 0;
uint16_t joystick_right = 0;
uint8_t platform_length = 20;
uint8_t platform_pos = 1;
uint8_t ball_pos_x = 42;
uint8_t ball_pos_y = 24;
int8_t ball_dir_x = 1;
int8_t ball_dir_y = 1;
bool joystick_flag = false;
uint8_t licznik = 0;
// Prostokątne bloczki
uint8_t blockWidth = 8;
uint8_t blockHeight = 5;
uint8_t gap = 2; // Przerwa pomiędzy bloczkami
uint8_t numBlocksPerRow = 7; //default 7
uint8_t numRows = 3; //default 3
uint8_t topOffset = 5; // Odległość od górnej krawędzi ekranu
uint8_t blocks[3][7]; // Tablica do przechowywania stanu bloczków
uint8_t numerBloczka = 0;
uint8_t numerBloczkaLevel = 0;
uint8_t last_x_block = 0; //Pozycja x ostatniego odbitego bloczka, fix do rysowania
uint8_t last_y_block = 0; //Pozycja y ostatniego odbitego bloczka, fix do rysowania
bool odbicieBloczek = false; //Flaga do detekcji pojedycznego odbicia od bloczka
bool draw_new_block = false; //Flaga do rysowania nowego rodzaju bloczka
uint8_t new_block_type = 0;
uint8_t ball_pos_bounce_x = 0; //Pozycja x pilki przy odbiciu od ostatniego bloczka
uint8_t ball_pos_bounce_y = 0; //Pozycja y pilki przy odbiciu od ostatniego bloczka
uint8_t last_level_x_block = 0; //Pozcyja ostatniego zbitego bloczka w poziomie
uint8_t last_level_y_block = 0; // Pozycja ostatniego zbitego bloczka w poziomie
uint8_t last_level_type_block = 1; //Typ ostatniego zbitego bloczka w poziomie
uint8_t bonusHits = 0;	//Licznik uderzen na niestandardowej platformie
bool flagPlatformLong = false; //flaga czy platforma wydluzona
bool flagPlatformShort = false; //flaga czy platforma skrocona
bool blockPlatform = false;
bool flagShuffle = false; //Flaga czy pilka przekroczyla prog do mieszania tablicy
bool flagCanBeShuffled = true;
bool flagPlayer = 0;
bool flagChangePlayer = false;
uint8_t posYToChangePlayers = 0;
//Wynik
char score[3];
uint8_t scoreint = 0;
uint8_t collapsedBlocks = 0; //aktualna liczba zbitych bloczkow

//Menu
uint8_t screen = 0;
uint8_t temp_screen = 0;
bool initGame = false;
bool startGame = false;
bool overGame = false;
bool flagOverScreen = false;
bool flagNextLevel = false;
bool przedostatniBloczek = false;


//Konfiguracja gry
uint8_t plansza = 0;
/* plaszna = 0 -> pelna,
 * plaszna = 1 -> szachownica,
 * plansza = 2 -> losowa,
 */
uint8_t randomArray[8];
uint8_t randomNumber = 0;
uint8_t level = 1;
uint8_t players = 1; // Liczba graczy
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_RNG_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
static void PlatformMoveRight(int startPoint, int length);
static void PlatformMoveLeft(int startPoint, int length);
static void PlatformExtend(int startPoint);
static void PlatformShorten(int startPoint);
static void BallMoveLeftUp(int x, int y);
static void BallMovement();
void randomArrayGen();
static void RefreshScore();
static void Shuffle(uint8_t array[numRows][numBlocksPerRow]);
int __io_putchar(int ch); //debug uart
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
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
  LCD_setRST(LCD_RST_GPIO_Port, LCD_RST_Pin);
  LCD_setCE(LCD_CE_GPIO_Port, LCD_CE_Pin);
  LCD_setDC(LCD_DC_GPIO_Port, LCD_DC_Pin);
  LCD_setDIN(LCD_DIN_GPIO_Port, LCD_DIN_Pin);
  LCD_setCLK(LCD_CLK_GPIO_Port, LCD_CLK_Pin);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_RNG_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Init(&htim3);
  //HAL_TIM_Base_Init(&htim6);
  HAL_ADC_Start_DMA(&hadc1, joystick1, 2);
  HAL_ADC_Start_DMA(&hadc2, joystick2, 2);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  LCD_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //Generowanie pozycji lepszych bloczkow
  randomArrayGen();


  while (1){
	  //Odbicia od ścian
	  if (ball_pos_x + 1 > 68)
		  ball_dir_x = -1;
	  else if (ball_pos_x - 1 <= 0)
		  ball_dir_x = 1;
	  if (ball_pos_y - 1 <= 0)
		  ball_dir_y = -1;
	  // Odbicie od podłogi -> Koniec gry
	  if (ball_pos_y + 1 > 46){
		  flagOverScreen = true;
	  }
	  //Odbicia od platformy
	  if (ball_pos_y + 1 > 45 && (ball_pos_x >= platform_pos && ball_pos_x <= (platform_pos + platform_length))){
		  ball_dir_y = 1;
		  flagCanBeShuffled = true;
		  flagShuffle = false;
		  if(players == 2){
			  flagChangePlayer = true;
			  posYToChangePlayers = ball_pos_y;
		  }
		  if (ball_pos_x > (platform_pos + (platform_length/2)))
			  ball_dir_x = 1;
		  else if (ball_pos_x <= (platform_pos + (platform_length/2)))
			  ball_dir_x = -1;
	  }
	  //Odbicia od bloczkow
	  for (int row = 0; row < numRows; row++) {
	      for (int col = 0; col < numBlocksPerRow; col++) {

	        if (blocks[row][col] > 0) {
	          int blockX = col * (blockWidth + gap);
	          int blockY = row * (blockHeight + gap) + topOffset;
	          // Sprawdź kolizję piłki z bloczkiem
	          if ((ball_pos_x + 1) >= blockX && (ball_pos_x - 1) <= (blockX + blockWidth) && (ball_pos_y - 1) <= blockY && (ball_pos_y + 1) >= (blockY - blockHeight) && !odbicieBloczek) {
	        	  odbicieBloczek  = true;
	        	  draw_new_block = true;
	        	  blockPlatform = true;
	        	  if (flagPlatformLong || flagPlatformShort){
	        		bonusHits++;
	        	  }
	        	  ball_pos_bounce_x = ball_pos_x;
	        	  ball_pos_bounce_y = ball_pos_y;
	        	  // Pozycja bloczka do pozniejszego rysowania
	        	  last_x_block = blockX;
	        	  last_y_block = blockY;
	        	  // wykrycie odbicia od pustego bloczka
	        	  if(blocks[row][col] == 1){
	        		  new_block_type = 0;
	        		  scoreint += 1;
	        		  if(przedostatniBloczek){
	        			  last_level_x_block = blockX;
	        			  last_level_y_block = blockY;
	        			  numerBloczkaLevel = (row * numBlocksPerRow) + col + 1;
	        		  }
	        	  }
	        	  // wykrycie odbicia od kratkowanego bloczka
	        	  else if(blocks[row][col] == 2){
	        		  new_block_type = 1;
	        		  scoreint += 1;
	        	  }
	        	  // wykrycie odbicia od pelnego bloczka
	        	  else if (blocks[row][col] == 3){
	        		  new_block_type = 2;
	        		  scoreint += 1;
	        	  }
	        	  // wykrycie odbicia od bonusowego bloczka
	        	  else if (blocks[row][col] == 4){
	        		  new_block_type = 1;
	        		  scoreint += 1;
	        		  blocks[row][col] = 2;
	        		  bonusHits = 0;
	        		  //wydluz platforme
	        		  if (HAL_RNG_GetRandomNumber(&hrng)%2 == 0 && !flagPlatformLong){
	        			  PlatformExtend(platform_pos);
	        			  flagPlatformLong = true;
	        		  }
	        		  //skroc platforme
	        		  else if(!flagPlatformShort){
	        			  PlatformShorten(platform_pos);
	        			  flagPlatformShort = true;
	        		  }


	        	  }
	        	  blocks[row][col] -= 1;

	            RefreshScore();

	            //Wykrycie odbicia od scian bloczkow - zmien kierunek w osi X
	            if ((ball_pos_x + 1) == blockX || (ball_pos_x - 1) == blockX + blockWidth){
	            	ball_dir_x *= -1;
	            }
	            //Wykrycie odbicia od podstaw bloczkow - zmien kierunek w osi Y
	            if ((ball_pos_y - 1) == blockY || (ball_pos_y + 1) == (blockY - blockHeight)){
	            	ball_dir_y *= -1;
	            }
	          }
	        }
	      }
	    }
	  // Sprawdz czy pilka ruszyla sie od momentu odbicia
	  if(abs(ball_pos_x - ball_pos_bounce_x) != 0 || abs(ball_pos_y - ball_pos_bounce_y) != 0){
		  odbicieBloczek = false;
		  blockPlatform = false;
	  }
	  //sprawdz czy wszystkie bloczki zbite
	  collapsedBlocks = 0;
	  for (int row = 0; row < numRows; row++) {
		  for (int col = 0; col < numBlocksPerRow; col++) {
			  if(blocks[row][col] == 0 && startGame){
				  collapsedBlocks += 1;
			  }
			  if (collapsedBlocks == (numRows * numBlocksPerRow)){
				  //wygrana
				  if(level == 3){
					  startGame = false;
					  blockPlatform = false;
					  bonusHits = 3;
					  level = 1;
					  screen = 20;
					  temp_screen = 20;
					  break;
				  }
				  else if(level < 3){
					  level += 1;
				  }
				  LCD_clrScr();
				  LCD_printLevelUp();
				  ball_pos_x = platform_pos + (platform_length/2);
				  ball_pos_y = PLATFORM_LVL-2;
				  startGame = false;
				  flagNextLevel = true;
				  screen = 19;
				  temp_screen = 19;
				  break;
			  }
			  else if (collapsedBlocks == (numRows * numBlocksPerRow) - 1){
				  przedostatniBloczek = true;
			  }
		  }
	  }
	  // Narysuj inny typ bloczka na ostatnim miescu odbicia
	  if((!odbicieBloczek && draw_new_block) && !flagNextLevel){
		  licznik++;
		  draw_new_block = false;
		  LCD_drawEmptyRectangle(last_x_block, last_y_block, last_x_block + blockWidth, last_y_block - blockHeight);
		  switch (new_block_type) {
			case 1:
				LCD_drawRectangle(last_x_block, last_y_block, last_x_block + blockWidth, last_y_block - blockHeight);
				break;
			case 2:
				LCD_drawChequeredRectangle(last_x_block, last_y_block, last_x_block + blockWidth, last_y_block - blockHeight);
				break;
			case 3:
				LCD_drawFilledRectangle(last_x_block, last_y_block, last_x_block + blockWidth, last_y_block - blockHeight);
				break;
		}
		  for (int i = 0; i < numRows; i++){
			  for (int j = 0; j < numBlocksPerRow; j++){
				  printf("%d ", blocks[i][j]);
			  }
			  printf("\n");
		  }
		  printf("\n \n");
	  }
	  if(flagChangePlayer && ((posYToChangePlayers - ball_pos_y) != 0)){
		  flagChangePlayer = false;
		  flagPlayer = !flagPlayer;
	  }

	  //Powrot do normalnej platformy po 3 zbiciach
	  if (bonusHits == 3 && (flagPlatformLong || flagPlatformShort)){
		  if(flagPlatformLong){
			  flagPlatformLong = false;
			  PlatformShorten(platform_pos);
		  }
		  if(flagPlatformShort){
			  flagPlatformShort = false;
			  PlatformExtend(platform_pos);
		  }
		  bonusHits = 0;
	  }
	  //Sprawdz warunek na mieszanie planszy
	  if ((plansza == 2 && ball_dir_y == (-1)) && (ball_pos_y == (numRows*(blockHeight+gap)))){
		  flagShuffle = true;
	  }

	  if(flagShuffle && flagCanBeShuffled){
		  flagShuffle = false;
		  flagCanBeShuffled = false;
		  Shuffle(blocks);
		  blockPlatform = true;
		  for (int row = 0; row < numRows; row++) {
				for (int col = 0; col < numBlocksPerRow; col++) {
						//numerBloczka++;
						int blockX = col * (blockWidth + gap);
						int blockY = row * (blockHeight + gap) + topOffset;
						LCD_drawEmptyRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);
						switch(blocks[row][col]){
						case 1:
							LCD_drawRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);
							break;
						case 2:
							LCD_drawChequeredRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);
							break;
						case 3:
							LCD_drawFilledRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);
							break;
						case 4:
							LCD_drawBonusRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);
							break;
						}
				}
		  }
		  //numerBloczka = 0;
		  blockPlatform = false;
	  }


	  //Wyswietl ekran konca gry
	  if(flagOverScreen){
		  flagOverScreen = false;
		  overGame = true;
		  startGame = false;
		  level = 1;
		  temp_screen = 18;
		  screen = 18;
		  scoreint = 0;
		  ball_pos_x = platform_pos + (platform_length/2);
		  ball_pos_y = PLATFORM_LVL-2;
		  LCD_clrScr();
		  LCD_printGameOver();
	  }




	  //randomNumber = HAL_RNG_GetRandomNumber(&hrng);
	  //for (int i = 0; i < 8; i++){
		  //printf("%d, ",randomArray[i]);
	  //}
	  //printf("\t");
	  //printf("ekran: %d\t licznik: %d\t sstartGame: %d \t initGame: %d \t overGame: %d \t x: %d \t y: %d \n",screen, licznik, startGame, initGame, overGame, ball_pos_x, ball_pos_y);
	  //printf("przedostatni: %d \t typ: %d \t x: %d \t y: %d \t number: %d \n",przedostatniBloczek, last_level_type_block, last_level_x_block, last_level_y_block, numerBloczkaLevel);
	  //printf("poz: %d \t length: %d \t flaga: %d\n", platform_pos, platform_length, flagPlatformExtend);
	  //printf("bonusHits: %d \t flagSHORT: %d \t flagLONG: %d \n", bonusHits, flagPlatformShort, flagPlatformLong);
	  //printf("dirX: %d \t dirY: %d\n",ball_dir_x, ball_dir_y);
	  //printf("%d \n",abs(HAL_RNG_GetRandomNumber(&hrng)));
	  //printf("balX: %d \t ballY: %d \t flagShuffle: %d \t flagCanBeShufled: %d\n", ball_pos_x, ball_pos_y, flagShuffle, flagCanBeShuffled);
	  printf("1stJOY x: %d \t 1stJOY y: %d \t 2ndJOY x: %d \t 2ndJOY y: %d \t player: %d \n", st_JOY1, st_JOY0, nd_JOY1, nd_JOY0, flagPlayer);
  }
  {
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
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
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
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 180;
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
  htim4.Init.Prescaler = 7999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 800;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 499;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 300;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_DC_Pin|LCD_CE_Pin|LCD_RST_Pin|LCD_DIN_Pin
                          |LCD_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DC_Pin LCD_CE_Pin LCD_RST_Pin LCD_DIN_Pin
                           LCD_CLK_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin|LCD_CE_Pin|LCD_RST_Pin|LCD_DIN_Pin
                          |LCD_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JOYSTICK_BUTTON_Pin JOYSTICK2_BUTTON_Pin */
  GPIO_InitStruct.Pin = JOYSTICK_BUTTON_Pin|JOYSTICK2_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
static void PlatformMoveRight(int startPoint, int length){
	LCD_setPixel(startPoint, PLATFORM_LVL, 0); // zgas lewy skrajny pxl
	LCD_refreshArea(startPoint, PLATFORM_LVL, startPoint, PLATFORM_LVL);
	LCD_setPixel(startPoint+length+1, PLATFORM_LVL, 1); // zapal prawy skrajny pxl
	LCD_refreshArea(startPoint+length+1, PLATFORM_LVL, startPoint+length+1, PLATFORM_LVL);
}
static void PlatformMoveLeft(int startPoint, int length){
	LCD_setPixel(startPoint+length, PLATFORM_LVL, 0); // zgas prawy skrajny pxl
	LCD_refreshArea(startPoint+length, PLATFORM_LVL, startPoint+length+1, PLATFORM_LVL);
	LCD_setPixel(startPoint-1, PLATFORM_LVL, 1); // zapal lewy skrajny pxl
	LCD_refreshArea(startPoint-1, PLATFORM_LVL, startPoint+length+1, PLATFORM_LVL);
}
static void PlatformExtend(int startPoint){
	platform_length += 10;
	if (startPoint<5){
		platform_pos = 0;
		for(int i = 0; i<platform_length; i++){
			LCD_setPixel(i, PLATFORM_LVL, 1);
		}
	}
	else if((platform_pos-5) + platform_length >= 68){
		platform_pos = 68 - platform_length;
		for(int i = 68; i>=platform_pos; i--){
			LCD_setPixel(i, PLATFORM_LVL, 1);
		}
	}
	else{
		platform_pos -= 5;
		for (int i = platform_pos; i<(platform_pos+platform_length); i++){
			LCD_setPixel(i, PLATFORM_LVL, 1);
		}
	}
	LCD_refreshArea(platform_pos, PLATFORM_LVL, platform_pos+platform_length, PLATFORM_LVL);
}
static void PlatformShorten(int startPoint){
	platform_length -= 10;
	platform_pos += 5;
	for (int i = platform_pos; i>= platform_pos -5; i--){
		LCD_setPixel(i, PLATFORM_LVL, 0);
	}
	for (int i = platform_pos; i<(platform_pos + platform_length); i++){
		LCD_setPixel(i, PLATFORM_LVL, 1);
	}
	for (int i = (platform_pos + platform_length); i<=(platform_pos + platform_length + 5); i++){
		LCD_setPixel(i, PLATFORM_LVL, 0);
	}
	LCD_refreshArea(platform_pos-5, PLATFORM_LVL, platform_pos+platform_length+6, PLATFORM_LVL);
}
static void BallMoveLeftUp(int x, int y){
	int *newX = &ball_pos_x;
	int *newY = &ball_pos_y;
	LCD_drawBall(x, y, 1);
	LCD_drawBall(x, y, 0);
	LCD_drawBall(x-1, y-1, 1);
	*newX -= 1;
	*newY -=1;
}
static void BallMoveLeftDown(int x, int y){
	int *newX = &ball_pos_x;
	int *newY = &ball_pos_y;
	LCD_drawBall(x, y, 1);
	LCD_drawBall(x, y, 0);
	LCD_drawBall(x-1, y+1, 1);
	*newX -= 1;
	*newY +=1;
}
static void BallMoveRightUp(int x, int y){
	int *newX = &ball_pos_x;
	int *newY = &ball_pos_y;
	LCD_drawBall(x, y, 1);
	LCD_drawBall(x, y, 0);
	LCD_drawBall(x+1, y-1, 1);
	*newX += 1;
	*newY -=1;
}
static void BallMoveRightDown(int x, int y){
	int *newX = &ball_pos_x;
	int *newY = &ball_pos_y;
	LCD_drawBall(x, y, 1);
	LCD_drawBall(x, y, 0);
	LCD_drawBall(x+1, y+1, 1);
	*newX += 1;
	*newY +=1;
}
static void BallMovement(){
	if (ball_dir_x > 0 && ball_dir_y > 0)
		BallMoveRightUp(ball_pos_x, ball_pos_y);
	if (ball_dir_x  < 0 && ball_dir_y > 0)
		BallMoveLeftUp(ball_pos_x, ball_pos_y);
	if (ball_dir_x > 0 && ball_dir_y < 0)
		BallMoveRightDown(ball_pos_x, ball_pos_y);
	if (ball_dir_x < 0 && ball_dir_y < 0)
		BallMoveLeftDown(ball_pos_x, ball_pos_y);
}
int __io_putchar(int ch){
	if (ch == '\n') {
		uint8_t ch2 = '\r';
		HAL_UART_Transmit(&huart2, &ch2, 1, HAL_MAX_DELAY);
	}
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
	return 1;
}
void randomArrayGen(){
    uint8_t i, j;
    uint8_t tempNumbers[21];
    uint8_t temp;
    // Wypełniamy tablicę wartościami od 1 do 21
    for (i = 0; i < 21; i++) {
        tempNumbers[i] = i + 1;
    }
    // Mieszamy tablicę tempNumbers
    for (i = 0; i < 21; i++) {
        j = HAL_RNG_GetRandomNumber(&hrng) % 21;
        temp = tempNumbers[i];
        tempNumbers[i] = tempNumbers[j];
        tempNumbers[j] = temp;
    }
    // Wypełniamy randomArray pierwszymi 8 unikalnymi liczbami z tablicy tempNumbers
    for (i = 0; i < 8; i++){
        randomArray[i] = tempNumbers[i];
    }
}
//Odswiez wynik po prawej stronie ekranu
static void RefreshScore(){
	LCD_drawVLine(70, 0, 48);
	LCD_refreshScr();
	LCD_goXY(72, 0);
	sprintf(score, "%d", scoreint);
	LCD_print(score, 72, 0);
}

//Zmien pozycje bloczkow
static void Shuffle(uint8_t blocks[numRows][numBlocksPerRow]){
	for (int i = (numRows-1); i > 0; i--) {
	        for (int j = (numBlocksPerRow-1); j > 0; j--) {
	            // Losowanie nowego indeksu
	            uint8_t new_i = abs(HAL_RNG_GetRandomNumber(&hrng)) % numRows;
	            uint8_t new_j = abs(HAL_RNG_GetRandomNumber(&hrng)) % numBlocksPerRow;

	            // Zamiana elementów
	            uint8_t temp = blocks[i][j];
	            blocks[i][j] = blocks[new_i][new_j];
	            blocks[new_i][new_j] = temp;
	        }
	    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if ((htim == &htim4 && startGame) && level == 1){
		BallMovement();
	  }
	if ((htim == &htim6 && startGame) && level == 2){
		BallMovement();
	}
	if ((htim == &htim7 && startGame) && level == 3){
			BallMovement();
		}

  if (htim == &htim3 && ((startGame || initGame) && !overGame && !blockPlatform)) {
	  switch(players){
	  //Ruch platformy dla jednego gracza
	  case 1:
		  if (st_JOY1 > 2300 && platform_pos > 0){
			  PlatformMoveLeft(platform_pos, platform_length);
			  platform_pos--;
		  }
	  	  if (st_JOY1 < 1600 && platform_pos + platform_length < 70){
	  		  PlatformMoveRight(platform_pos, platform_length);
	  		  platform_pos++;
	  	  }

		  break;
	  //Ruch platformy dla dwoch graczy
	  case 2:
		  //Ruch pierwszego gracza
		  if(flagPlayer == 0){
			  if (st_JOY1 > 2300 && platform_pos > 0){
				  PlatformMoveLeft(platform_pos, platform_length);
				  platform_pos--;
			  }
		  	  if (st_JOY1 < 1600 && platform_pos + platform_length < 70){
		  		  PlatformMoveRight(platform_pos, platform_length);
		  		  platform_pos++;
		  	  }
		  }
		  //Ruch drugiego gracza
		  else{
			  if (nd_JOY1 > 2300 && platform_pos > 0){
				  PlatformMoveLeft(platform_pos, platform_length);
				  platform_pos--;
			  }
		  	  if (nd_JOY1 < 1600 && platform_pos + platform_length < 70){
		  		  PlatformMoveRight(platform_pos, platform_length);
		  		  platform_pos++;
		  	  }
		  }
		  break;
	  }

  	  if (!startGame){
  		  //Rysowanie pilki przed startem gry
  		  ball_pos_x = platform_pos + (platform_length/2);
  		  ball_pos_y = PLATFORM_LVL-2;
  		LCD_drawBall(ball_pos_x-1, ball_pos_y, 0);
  		LCD_drawBall(ball_pos_x+1, ball_pos_y, 0);
  		LCD_drawBall(ball_pos_x, ball_pos_y, 1);
  		LCD_drawRectangle(last_level_x_block, last_level_y_block, last_level_x_block + blockWidth, last_level_y_block - blockHeight);
  		for (int i = 0; i < 6; i++){
			if(randomArray[i] == numerBloczkaLevel){
				if (randomArray[i]%3==0){
					LCD_drawFilledRectangle(last_level_x_block, last_level_y_block, last_level_x_block + blockWidth, last_level_y_block - blockHeight);
				}
				else if (randomArray[i]%3==1){
					LCD_drawChequeredRectangle(last_level_x_block, last_level_y_block, last_level_x_block + blockWidth, last_level_y_block - blockHeight);
				}
				else if (randomArray[i]%3 == 2){
					LCD_drawBonusRectangle(last_level_x_block, last_level_y_block, last_level_x_block + blockWidth, last_level_y_block - blockHeight);
				}
				break;
			}
		}
  	  }
  }
  if (htim == &htim2 && (!initGame && !startGame && !overGame)){
	  if((st_JOY0<=1000) || (nd_JOY0<=1000))
	  		{
	  			switch(temp_screen)
	  				 {
	  				case 0:
	  					temp_screen = 1;
	  					break;
	  				 case 1:
	  					 	 temp_screen=5;
	  					 	 break;
	  				 case 2:
	  				 		 temp_screen=1;
	  				 		 break;
	  				 case 3:
	  				 		 temp_screen=2;
	  				 		 break;
	  				 case 4:
	  				 		 temp_screen=3;
	  				 		 break;
	  				 case 5:
	  				 		 temp_screen=4;
	  				 		 break;
	  				 case 6:
	  				 		 temp_screen=9;
	  				 		 break;
	  				 case 7:
	  				 		 temp_screen=6;
	  				 		 break;
	  				 case 8:
	  				 		 temp_screen=7;
	  				 		 break;
	  				 case 9:
	  				 		 temp_screen=8;
	  				 		 break;
	  				 case 10:
	  				 		 temp_screen=13;
	  				 		 break;
	  				 case 11:
	  				 		 temp_screen=10;
	  				 		 break;
	  				 case 12:
	  				 		 temp_screen=11;
	  				 		 break;
	  				 case 13:
	  				 		 temp_screen=12;
	  				 		 break;
	  				 case 14:
	  				 		 temp_screen=16;
	  				 		 break;
	  				 case 15:
	  				 		 temp_screen=14;
	  				 		 break;
	  				 case 16:
	  				 		 temp_screen=15;
	  				 		 break;
	  				 }
	  				 screen=temp_screen;
	  		}
	  	if((st_JOY0>=3000) || (nd_JOY0>=3000))
	  	{
	  		 switch(temp_screen)
	  			 {
	  			case 1:
	  				temp_screen=2;
	  				break;
	  			case 2:
	  				temp_screen=3;
	  				break;
	  			case 3:
	  				temp_screen=4;
	  				break;
	  			case 4:
	  				temp_screen=5;
	  				break;
	  			case 5:
	  				temp_screen=1;
	  				break;
	  			case 6:
	  				temp_screen=7;
	  				break;
	  			case 7:
	  				temp_screen=8;
	  				break;
	  			case 8:
	  				temp_screen=9;
	  				break;
	  			case 9:
	  				temp_screen=6;
	  				break;
	  			case 10:
	  				temp_screen=11;
	  				break;
	  			case 11:
	  				temp_screen=12;
	  				break;
	  			case 12:
	  				temp_screen=13;
	  				break;
	  			case 13:
	  				temp_screen=10;
	  				break;
	  			case 14:
	  				temp_screen=15;
	  				break;
	  			case 15:
	  				temp_screen=16;
	  				break;
	  			case 16:
	  				temp_screen=14;
	  				break;
	  			 }
	  				screen=temp_screen;

	  	}
	  	  switch(screen)
	  	  	 {
	  	  	 case 1:
	  	  LCD_clrScr();
	  	  LCD_invertText(1);
	  	  LCD_print("1.START", 0, 0);
	  	  LCD_invertText(0);
	  	  LCD_print("2.PLANSZA", 0, 1);
	  	  LCD_print("3.POZIOM", 0, 2);
	  	  LCD_print("4.GRACZE", 0, 3);
	  	  LCD_print("5.REKORDY", 0, 4);
	  	  break;
	  	  	 case 2:
	  	  LCD_print("1.START", 0, 0);
	  	  LCD_invertText(1);
	  	  LCD_print("2.PLANSZA", 0, 1);
	  	  LCD_invertText(0);
	  	  LCD_print("3.POZIOM", 0, 2);
	  	  LCD_print("4.GRACZE", 0, 3);
	  	  LCD_print("5.REKORDY", 0, 4);
	  	  break;
	  	  	 case 3:
	  	  LCD_print("1.START", 0, 0);
	  	  LCD_print("2.PLANSZA", 0, 1);
	  	  LCD_invertText(1);
	  	  LCD_print("3.POZIOM", 0, 2);
	  	  LCD_invertText(0);
	  	  LCD_print("4.GRACZE", 0, 3);
	  	  LCD_print("5.REKORDY", 0, 4);
	  	  break;
	  	  	 case 4:
	  	  LCD_print("1.START", 0, 0);
	  	  LCD_print("2.PLANSZA", 0, 1);
	  	  LCD_print("3.POZIOM", 0, 2);
	  	  LCD_invertText(1);
	  	  LCD_print("4.GRACZE", 0, 3);
	  	  LCD_invertText(0);
	  	  LCD_print("5.REKORDY", 0, 4);
	  	  break;
	  	  	 case 5:
	  	  LCD_print("1.START", 0, 0);
	  	  LCD_print("2.PLANSZA", 0, 1);
	  	  LCD_print("3.POZIOM", 0, 2);
	  	  LCD_print("4.GRACZE", 0, 3);
	  	  LCD_invertText(1);
	  	  LCD_print("5.REKORDY", 0, 4);
	  	  LCD_invertText(0);
	  	     break;
	  	  	 case 6:
	  	  LCD_clrScr();
	  	  LCD_invertText(1);
	  	  LCD_print("1.PELNA", 0, 0);
	  	  LCD_invertText(0);
	  	  LCD_print("2.SZACHOWNICA", 0, 1);
	  	  LCD_print("3.LOSOWA", 0, 2);
	  	  LCD_print("4.COFNIJ", 0, 3);
	  	  break;
	  	  	 case 7:
	  	  LCD_print("1.PELNA", 0, 0);
	  	  LCD_invertText(1);
	  	  LCD_print("2.SZACHOWNICA", 0, 1);
	  	  LCD_invertText(0);
	  	  LCD_print("3.LOSOWA", 0, 2);
	  	  LCD_print("4.COFNIJ", 0, 3);
	  	  break;
	  	  	 case 8:
	  	  LCD_print("1.PELNA", 0, 0);
	  	  LCD_print("2.SZACHOWNICA", 0, 1);
	  	  LCD_invertText(1);
	  	  LCD_print("3.LOSOWA", 0, 2);
	  	  LCD_invertText(0);
	  	  LCD_print("4.COFNIJ", 0, 3);
	  	  break;
	  	  	 case 9:
	  	  LCD_print("1.PELNA", 0, 0);
	  	  LCD_print("2.SZACHOWNICA", 0, 1);
	  	  LCD_print("3.LOSOWA", 0, 2);
	  	  LCD_invertText(1);
	  	  LCD_print("4.COFNIJ", 0, 3);
	  	  LCD_invertText(0);
	  	  break;
	  	  	 case 10:
	  	  LCD_clrScr();
	  	  LCD_invertText(1);
	  	  LCD_print("1.PIERWSZY", 0, 0);
	  	  LCD_invertText(0);
	  	  LCD_print("2.DRUGI", 0, 1);
	  	  LCD_print("3.TRZECI", 0, 2);
	  	  LCD_print("4.COFNIJ", 0, 3);
	  	  break;
	  	  	 case 11:
	  	  LCD_print("1.PIERWSZY", 0, 0);
	  	  LCD_invertText(1);
	  	  LCD_print("2.DRUGI", 0, 1);
	  	  LCD_invertText(0);
	  	  LCD_print("3.TRZECI", 0, 2);
	  	  LCD_print("4.COFNIJ", 0, 3);
	  	  break;
	  	  	 case 12:
	  	  LCD_print("1.PIERWSZY", 0, 0);
	  	  LCD_print("2.DRUGI", 0, 1);
	  	  LCD_invertText(1);
	  	  LCD_print("3.TRZECI", 0, 2);
	  	  LCD_invertText(0);
	  	  LCD_print("4.COFNIJ", 0, 3);
	  	  break;
	  	  	 case 13:
	  	  LCD_print("1.PIERWSZY", 0, 0);
	  	  LCD_print("2.DRUGI", 0, 1);
	  	  LCD_print("3.TRZECI", 0, 2);
	  	  LCD_invertText(1);
	  	  LCD_print("4.COFNIJ", 0, 3);
	  	  LCD_invertText(0);
	  	  break;
	  	  	 case 14:
	  	  LCD_clrScr();
	  	  LCD_invertText(1);
	  	  LCD_print("1.JEDEN", 0, 0);
	  	  LCD_invertText(0);
	  	  LCD_print("2.DWOCH", 0, 1);
	  	  LCD_print("3.COFNIJ", 0, 2);
	  	  break;
	  	  	 case 15:
	  	  LCD_print("1.JEDEN", 0, 0);
	  	  LCD_invertText(1);
	  	  LCD_print("2.DWOCH", 0, 1);
	  	  LCD_invertText(0);
	  	  LCD_print("3.COFNIJ", 0, 2);
	  	  break;
	  	  	 case 16:
	  	  LCD_print("1.JEDEN", 0, 0);
	  	  LCD_print("2.DWOCH", 0, 1);
	  	  LCD_invertText(1);
	  	  LCD_print("3.COFNIJ", 0, 2);
	  	  LCD_invertText(0);
	  	  break;
	  	  case 17:
	  		initGame = true;
	  		flagNextLevel = false;
	  		randomArrayGen();
	  		LCD_clrScr();
	  		LCD_drawHLine(platform_pos, PLATFORM_LVL, platform_length); // poczatkowe polozenie platformy
	  		LCD_refreshArea(platform_pos, PLATFORM_LVL, platform_pos + platform_length, PLATFORM_LVL);


	  	  // Inicjalizuj stan bloczkow
	  		switch(plansza){
	  		// Rysowanie pelnej planszy
	  		case 0:
	  			for (int row = 0; row < numRows; row++) {
					for (int col = 0; col < numBlocksPerRow; col++) {
							numerBloczka++;
							int blockX = col * (blockWidth + gap);
							int blockY = row * (blockHeight + gap) + topOffset;

							blocks[row][col] = 1; // Wszystkie bloczki są widoczne na początku
							LCD_drawRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);

							for (int i = 0; i < 8; i++){
								if(randomArray[i] == numerBloczka){
									if (randomArray[i]%3==0){
										blocks[row][col] = 3; // rysuj pelny bloczek
										LCD_drawFilledRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);
									}
									else if (randomArray[i]%3==1){
										blocks[row][col] = 2; // rysuj kratkowany bloczek
										LCD_drawChequeredRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);
									}
									else if(randomArray[i]%3==2){
										blocks[row][col] = 4; // rysuj bonusowy bloczek
										LCD_drawBonusRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);
									}
									break;
								}
							}
					}
				  }
				  // Rysuj wynik po prawej
				  RefreshScore();
				  numerBloczka = 0;
			  break;

				  // Rysowanie szachownicy planszy
	  		case 1:
	  			for (int row = 0; row < numRows; row++) {
						for (int col = 0; col < numBlocksPerRow; col++) {
							int blockX = col * (blockWidth + gap);
							int blockY = row * (blockHeight + gap) + topOffset;
							LCD_drawEmptyRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);
							if((row+col)%2==0){
								numerBloczka++;
								blocks[row][col] = 1; // Widoczny co drugi bloczek
								int blockY = row * (blockHeight + gap) + topOffset;
								LCD_drawRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);

								for (int i = 0; i < 8; i++){
									if(randomArray[i] == numerBloczka){
										if (randomArray[i]%3==0){
											blocks[row][col] = 3; // rysuj pelny bloczek
											LCD_drawFilledRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);
										}
										else if(randomArray[i]%3==1){
											blocks[row][col] = 2; // rysuj kratkowany bloczek
											LCD_drawChequeredRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);
										}
										else if(randomArray[i]%3==2){
											blocks[row][col] = 4; // rysuj bonusowy bloczek
											LCD_drawBonusRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);
										}
										break;
									}
								}
							}
							else{
								blocks[row][col] = 0; // Widoczny co drugi bloczek
							}
						}
					  }
					  // Rysuj wynik po prawej
					  LCD_drawVLine(70, 0, 48); //narysuj kreske oddzielajacą wynik
					  LCD_refreshScr();
					  LCD_goXY(72, 0);
					  sprintf(score, "%d", scoreint);
					  LCD_print(score, 72, 0);
					  numerBloczka = 0;
			  break;
			  //Rysowanie losowej planszy
	  		case 2:
	  			for (int row = 0; row < numRows; row++) {
					for (int col = 0; col < numBlocksPerRow; col++) {
							numerBloczka++;
							int blockX = col * (blockWidth + gap);
							int blockY = row * (blockHeight + gap) + topOffset;

							blocks[row][col] = 1; // Wszystkie bloczki są widoczne na początku
							LCD_drawRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);

							for (int i = 0; i < 8; i++){
								if(randomArray[i] == numerBloczka){
									if (randomArray[i]%3==0){
										blocks[row][col] = 3; // rysuj pelny bloczek
										LCD_drawFilledRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);
									}
									else if (randomArray[i]%3==1){
										blocks[row][col] = 2; // rysuj kratkowany bloczek
										LCD_drawChequeredRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);
									}
									else if(randomArray[i]%3==2){
										blocks[row][col] = 4; // rysuj bonusowy bloczek
										LCD_drawBonusRectangle(blockX, blockY, blockX + blockWidth, blockY - blockHeight);
									}
									break;
								}
							}
					}
				  }
				  // Rysuj wynik po prawej
				  RefreshScore();
				  numerBloczka = 0;
				  break;
	  		}
	  		break;
	  		//Ekran konca gry
	  		case 18:
	  			LCD_clrScr();
	  			LCD_printGameOver();
	  			break;
	  		//Ekran level Up
	  		case 19:
	  			LCD_clrScr();
	  			LCD_printLevelUp();
	  			break;
			//Ekran wygranej
	  		case 20:
	  			LCD_clrScr();
	  			LCD_printVictory();
	  			sprintf(score, "%d", scoreint);
	  			LCD_print(score, 36, 4);
	  			break;
	  	  	 }

  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if((GPIO_Pin == JOYSTICK_BUTTON_Pin) || (GPIO_Pin == JOYSTICK2_BUTTON_Pin)){

		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		//Menu
		switch(temp_screen)
				 	 {
				 	case 2:
				 		temp_screen = 6;
				 		break;
				 	case 3:
				 		temp_screen = 10;
				 		break;
				 	case 4:
				 		temp_screen = 14;
				 		break;
				 	// Wybierz plansza pelna, default
				 	case 6:
				 		plansza = 0;
				 		temp_screen = 1;
				 		break;
					// Wybierz plansza szachownica
				 	case 7:
				 		plansza = 1;
				 		temp_screen = 1;
				 		break;
					// Wybierz plansza losowa
				 	case 8:
				 		plansza = 2;
				 		temp_screen = 1;
				 		break;
					// powrot do menu glownego
				 	 case 9:
					 	temp_screen = 1;
			 		 	break;
					// Wybierz 1 poziom trudnosci
				 	 case 10:
				 		 level = 1;
				 		 temp_screen = 1;
				 		 break;
					 // Wybierz 2 poziom trudnosci
				 	 case 11:
				 		 level = 2;
				 		 temp_screen = 1;
				 		 break;
					 // Wybierz 3 poziom trudnosci
				 	 case 12:
				 		 level = 3;
				 		 temp_screen = 1;
				 		 break;
					 // powrot do menu glownego
				 	 case 13:
				 		 temp_screen = 1;
					 	break;
					//Wybierz jednego gracza
				 	 case 14:
				 		 players = 1;
				 		 temp_screen = 1;
				 		 break;
					 //Wybierz dwoch graczy
				 	 case 15:
				 		 players = 2;
				 		 temp_screen = 1;
					 //Powrot do menu glownego
				 	 case 16:
					 	temp_screen = 1;
					 	break;
					// start gry
				 	 case 1:
				 		 temp_screen = 17;
						 break;
					// Gra
				 	 case 17:
				 		 startGame = true;
						 initGame = false;
						 temp_screen = 17;
				 		 break;
				 	//Ekran konca gry
				 	case 18:
						 overGame = false;
						 temp_screen = 1;
						 break;
					 //Ekran level up
				 	case 19:
				 		temp_screen = 17;
				 		break;
					//Ekran wygranej
				 	case 20:
				 		temp_screen = 1;
				 	 }
				 	screen=temp_screen;
	}
	// Menu powrot to glownego ekranu
	if(GPIO_Pin == B1_Pin)
		 {
			 switch(temp_screen)
			 	 {
			 	case 6:
			 		temp_screen=1;
			 		break;
			 	case 7:
			 		temp_screen=1;
			 		break;
			 	case 8:
			 		temp_screen=1;
			 		break;
			 	 case 9:
				 	temp_screen=1;
		 		 	break;
			 	 case 10:
			 		 temp_screen=1;
				 	break;
			 	 case 11:
				 	temp_screen=1;
				 	break;
			 	 case 12:
				 	temp_screen=1;
				 	break;
			 	 case 13:
				 	temp_screen=1;
				 	break;
			 	 case 14:
				 	temp_screen=1;
				 	break;
			 	 case 15:
				 	temp_screen=1;
				 	break;
			 	 case 16:
				 	temp_screen=1;
				 	break;
			 	 case 17:
			 		 startGame = false;
					 initGame = false;
				 	temp_screen=1;
				 	break;
			 	 case 18:
			 		 overGame = false;
			 		 temp_screen = 1;
			 		 break;
			 	 case 20:
			 		 temp_screen = 1;
			 		 break;
			 	 }
			 	screen=temp_screen;
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
