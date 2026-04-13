/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<string.h>
#include "print3.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UDP_BUFFER_SIZE 512 //bytes
#define SPI_BUFFER_SIZE_BYTES 64 // keep 32 bit alignment
#define SPI_BUFFER_SIZE_WORDS 32 // to keep 32 bit alignment
#define NUM_ACTUATORS 26 //

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
extern struct netif gnetif;

union udp_data {
	uint8_t bytes[UDP_BUFFER_SIZE];
	int16_t values[UDP_BUFFER_SIZE / 2];
};

union udp_data udp_rx;
uint8_t spi_buffer_length_bytes = SPI_BUFFER_SIZE_BYTES;

/* SPI buffers */
// Use compiler attributes to align to 32 bytes and pad the size to a multiple of 32

__attribute__((section(".dma_buffer"), used, aligned(32)))   uint16_t spi1_tx_buffer[SPI_BUFFER_SIZE_WORDS]; // 64 bytes
__attribute__((section(".dma_buffer"), used, aligned(32)))   uint16_t spi1_rx_buffer[SPI_BUFFER_SIZE_WORDS]; // 64 bytes
__attribute__((section(".dma_buffer"), used, aligned(32)))   uint16_t spi2_tx_buffer[SPI_BUFFER_SIZE_WORDS]; // 64 bytes
__attribute__((section(".dma_buffer"), used, aligned(32)))   uint16_t spi2_rx_buffer[SPI_BUFFER_SIZE_WORDS]; // 64 bytes
__attribute__((section(".dma_buffer"), used, aligned(32)))   uint16_t spi3_tx_buffer[SPI_BUFFER_SIZE_WORDS]; // 64 bytes
__attribute__((section(".dma_buffer"), used, aligned(32)))   uint16_t spi3_rx_buffer[SPI_BUFFER_SIZE_WORDS]; // 64 bytes
__attribute__((section(".dma_buffer"), used, aligned(32)))   uint16_t spi4_tx_buffer[SPI_BUFFER_SIZE_WORDS]; // 64 bytes
__attribute__((section(".dma_buffer"), used, aligned(32)))   uint16_t spi4_rx_buffer[SPI_BUFFER_SIZE_WORDS]; // 64 bytes


volatile uint8_t spi_txfer_complete = 0; // All DMA SPI transfers are complete
volatile uint8_t spi_data_ready = 0; // flag to indicate SPI data is ready for processing

volatile uint32_t sys_time_ms = {0};
volatile uint32_t start_time_us={0};
volatile uint32_t end_time_us={0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
/* USER CODE BEGIN PFP */
void DWT_Init(void);
void update_stats(uint32_t val);
void SPI1_DMA_txfer(void);
void SPI2_DMA_txfer(void);
void SPI3_DMA_txfer(void);
void SPI4_DMA_txfer(void);
void udp_receive_callback(void *arg, // User argument - udp_recv `arg` parameter
		struct udp_pcb *upcb,   // Receiving Protocol Control Block
		struct pbuf *p,         // Pointer to Datagram
		const ip_addr_t *addr,  // Address of sender
		u16_t port);

uint32_t sys_time(void);

//HAL_StatusTypeDef print_uart3(const char *pData);

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_DMA_Init();
  MX_LWIP_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();

	HAL_TIM_Base_Start_IT(&htim2);
	memset(&spi1_tx_buffer, 0, sizeof(spi1_tx_buffer));
	memset(&spi1_rx_buffer, 0, sizeof(spi1_rx_buffer));
	memset(&spi2_tx_buffer, 0, sizeof(spi2_tx_buffer));
	memset(&spi2_rx_buffer, 0, sizeof(spi2_rx_buffer));
	memset(&spi3_tx_buffer, 0, sizeof(spi3_tx_buffer));
	memset(&spi3_rx_buffer, 0, sizeof(spi3_rx_buffer));
	memset(&spi4_tx_buffer, 0, sizeof(spi4_tx_buffer));
	memset(&spi4_rx_buffer, 0, sizeof(spi4_rx_buffer));

	char msg[1024] = { 0 };

	print_uart3("Starting KASM Interface\r\n");

	/*wait for IP address to be assigned. DHCP can take a few seconds*/

	char no_ip[] = "0.0.0.0";

	print_uart3("Waiting for IP address to be assigned\r\n");

	while (strcmp(no_ip, ip4addr_ntoa(netif_ip4_addr(&gnetif))) == 0) {
		MX_LWIP_Process();
	}

	/* Determine IP address */
	sprintf(msg, "IP address: %s\r\n", ip4addr_ntoa(netif_ip4_addr(&gnetif)));
	print_uart3(msg);
	sprintf(msg, "Netmask: %s\r\n", ip4addr_ntoa(netif_ip4_netmask(&gnetif)));
	print_uart3(msg);
	sprintf(msg, "Gateway: %s\r\n", ip4addr_ntoa(netif_ip4_gw(&gnetif)));
	print_uart3(msg);

	u16_t port = 5001; // port to listen from

	struct udp_pcb *my_udp = udp_new();

	udp_bind(my_udp, IP_ADDR_ANY, port);
	udp_recv(my_udp, udp_receive_callback, NULL);
	if (((uint32_t) spi1_tx_buffer & 0xFF000000) != 0x30000000) {
		sprintf(msg, "Buffer not in RAM D2: %lu\r\n",
				(uint32_t) spi1_tx_buffer);
		print_uart3(msg);
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		MX_LWIP_Process();

		if (spi_data_ready == 1) {
//			sprintf(msg,
//					"Elapsed time from receipt of ethernet to spi complete %lu\r\n",
//					(end_time_us - start_time_us);
//			print_uart3(msg);

			SPI1_DMA_txfer(); // initiate SPI transfers DMA version
			SPI2_DMA_txfer();
			SPI3_DMA_txfer();
			SPI4_DMA_txfer();
			spi_data_ready = 0;
		}
		if (spi_txfer_complete == 1) {
			end_time_us = sys_time();
			update_stats(end_time_us - start_time_us);
//			char msg[100];
//			sprintf(msg, "t=%lu\r\n",
//					end_time_us - start_time_us);
//			print_uart3(msg);
			/* read transferred bytes */
//			for (int i = 0; i < NUM_ACTUATORS; i++) {
//				sprintf(msg, "Actuator %d : %d \r\n", i, spi4_rx_buffer[i]);
//				print_uart3(msg);
//			}
			spi_txfer_complete = 0;
		}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_0, (uint32_t) spi1_tx_buffer,
			(uint32_t) &(SPI1->TXDR), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_1, (uint32_t) &(SPI1->RXDR),
			(uint32_t) spi1_rx_buffer, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
  /**SPI1 GPIO Configuration
  PA4   ------> SPI1_NSS
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PD7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* SPI1 DMA Init */

  /* SPI1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_0, LL_DMAMUX1_REQ_SPI1_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_0, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_EnableFifoMode(DMA1, LL_DMA_STREAM_0);

  LL_DMA_SetFIFOThreshold(DMA1, LL_DMA_STREAM_0, LL_DMA_FIFOTHRESHOLD_1_2);

  LL_DMA_SetMemoryBurstxfer(DMA1, LL_DMA_STREAM_0, LL_DMA_MBURST_SINGLE);

  LL_DMA_SetPeriphBurstxfer(DMA1, LL_DMA_STREAM_0, LL_DMA_PBURST_SINGLE);

  /* SPI1_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_1, LL_DMAMUX1_REQ_SPI1_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_1, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_EnableFifoMode(DMA1, LL_DMA_STREAM_1);

  LL_DMA_SetFIFOThreshold(DMA1, LL_DMA_STREAM_1, LL_DMA_FIFOTHRESHOLD_FULL);

  LL_DMA_SetMemoryBurstxfer(DMA1, LL_DMA_STREAM_1, LL_DMA_MBURST_SINGLE);

  LL_DMA_SetPeriphBurstxfer(DMA1, LL_DMA_STREAM_1, LL_DMA_PBURST_SINGLE);

  /* SPI1 interrupt Init */
  NVIC_SetPriority(SPI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(SPI1_IRQn);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_HARD_OUTPUT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI1, LL_SPI_FIFO_TH_04DATA);
  LL_SPI_EnableNSSPulseMgt(SPI1);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_2, (uint32_t) spi2_tx_buffer,
			(uint32_t) &(SPI2->TXDR), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_3, (uint32_t) &(SPI2->RXDR),
			(uint32_t) spi2_rx_buffer, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* USER CODE END SPI2_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PC2_C   ------> SPI2_MISO
  PC3_C   ------> SPI2_MOSI
  PB10   ------> SPI2_SCK
  PB12   ------> SPI2_NSS
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SPI2 DMA Init */

  /* SPI2_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_2, LL_DMAMUX1_REQ_SPI2_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_EnableFifoMode(DMA1, LL_DMA_STREAM_2);

  LL_DMA_SetFIFOThreshold(DMA1, LL_DMA_STREAM_2, LL_DMA_FIFOTHRESHOLD_FULL);

  LL_DMA_SetMemoryBurstxfer(DMA1, LL_DMA_STREAM_2, LL_DMA_MBURST_SINGLE);

  LL_DMA_SetPeriphBurstxfer(DMA1, LL_DMA_STREAM_2, LL_DMA_PBURST_SINGLE);

  /* SPI2_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_3, LL_DMAMUX1_REQ_SPI2_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_3);

  /* SPI2 interrupt Init */
  NVIC_SetPriority(SPI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),4, 0));
  NVIC_EnableIRQ(SPI2_IRQn);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_HARD_OUTPUT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI2, LL_SPI_FIFO_TH_02DATA);
  LL_SPI_EnableNSSPulseMgt(SPI2);
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_0, (uint32_t) spi3_tx_buffer,
			(uint32_t) &(SPI3->TXDR), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_1, (uint32_t) &(SPI3->RXDR),
			(uint32_t) spi3_rx_buffer, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* USER CODE END SPI3_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  /**SPI3 GPIO Configuration
  PA15 (JTDI)   ------> SPI3_NSS
  PC10   ------> SPI3_SCK
  PC11   ------> SPI3_MISO
  PC12   ------> SPI3_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11|LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* SPI3 DMA Init */

  /* SPI3_TX Init */
  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_0, LL_DMAMUX1_REQ_SPI3_TX);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_EnableFifoMode(DMA2, LL_DMA_STREAM_0);

  LL_DMA_SetFIFOThreshold(DMA2, LL_DMA_STREAM_0, LL_DMA_FIFOTHRESHOLD_FULL);

  LL_DMA_SetMemoryBurstxfer(DMA2, LL_DMA_STREAM_0, LL_DMA_MBURST_SINGLE);

  LL_DMA_SetPeriphBurstxfer(DMA2, LL_DMA_STREAM_0, LL_DMA_PBURST_SINGLE);

  /* SPI3_RX Init */
  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_1, LL_DMAMUX1_REQ_SPI3_RX);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_EnableFifoMode(DMA2, LL_DMA_STREAM_1);

  LL_DMA_SetFIFOThreshold(DMA2, LL_DMA_STREAM_1, LL_DMA_FIFOTHRESHOLD_FULL);

  LL_DMA_SetMemoryBurstxfer(DMA2, LL_DMA_STREAM_1, LL_DMA_MBURST_SINGLE);

  LL_DMA_SetPeriphBurstxfer(DMA2, LL_DMA_STREAM_1, LL_DMA_PBURST_SINGLE);

  /* SPI3 interrupt Init */
  NVIC_SetPriority(SPI3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));
  NVIC_EnableIRQ(SPI3_IRQn);

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_HARD_OUTPUT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0;
  LL_SPI_Init(SPI3, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI3, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_EnableNSSPulseMgt(SPI3);
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_2, (uint32_t) spi4_tx_buffer,
			(uint32_t) &(SPI4->TXDR), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_3, (uint32_t) &(SPI4->RXDR),
			(uint32_t) spi4_rx_buffer, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* USER CODE END SPI4_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 30;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 4;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI4);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
  /**SPI4 GPIO Configuration
  PE2   ------> SPI4_SCK
  PE4   ------> SPI4_NSS
  PE5   ------> SPI4_MISO
  PE6   ------> SPI4_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* SPI4 DMA Init */

  /* SPI4_TX Init */
  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_2, LL_DMAMUX1_REQ_SPI4_TX);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_EnableFifoMode(DMA2, LL_DMA_STREAM_2);

  LL_DMA_SetFIFOThreshold(DMA2, LL_DMA_STREAM_2, LL_DMA_FIFOTHRESHOLD_FULL);

  LL_DMA_SetMemoryBurstxfer(DMA2, LL_DMA_STREAM_2, LL_DMA_MBURST_SINGLE);

  LL_DMA_SetPeriphBurstxfer(DMA2, LL_DMA_STREAM_2, LL_DMA_PBURST_SINGLE);

  /* SPI4_RX Init */
  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_3, LL_DMAMUX1_REQ_SPI4_RX);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_EnableFifoMode(DMA2, LL_DMA_STREAM_3);

  LL_DMA_SetFIFOThreshold(DMA2, LL_DMA_STREAM_3, LL_DMA_FIFOTHRESHOLD_FULL);

  LL_DMA_SetMemoryBurstxfer(DMA2, LL_DMA_STREAM_3, LL_DMA_MBURST_SINGLE);

  LL_DMA_SetPeriphBurstxfer(DMA2, LL_DMA_STREAM_3, LL_DMA_PBURST_SINGLE);

  /* SPI4 interrupt Init */
  NVIC_SetPriority(SPI4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(SPI4_IRQn);

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_HARD_OUTPUT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0;
  LL_SPI_Init(SPI4, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI4, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI4, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_EnableNSSPulseMgt(SPI4);
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 240000-1;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  /* DMA1_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void update_stats(uint32_t val){
	const uint32_t n = {1000};
	static uint32_t vals[1000];
	static uint32_t index={0};
	uint32_t sum={0};
	double ave={0.0};
	static uint32_t min = {40};
	static uint32_t max = {0};
	char msg[100];

	if(val < min) min = val;
	if(val > max) max = val;

	vals[index] = val;
	index++;

	if(index >= n){
		/* compute stats */
		for(int i=0; i<n; i++){
			sum +=vals[i];
		}
		ave = (double)sum/(double)n;
		sprintf(msg, "%lu\t %f \t %lu\r\n", min, ave, max);
		print_uart3(msg);
		/* reset stats counter and vals*/
		index=0;
		min = 40;
		max = 0;
	}

}

void DWT_Init(void){
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

}

void udp_receive_callback(void *arg, // User argument - udp_recv `arg` parameter
		struct udp_pcb *upcb,   // Receiving Protocol Control Block
		struct pbuf *p,         // Pointer to Datagram
		const ip_addr_t *addr,  // Address of sender
		u16_t port) {

	start_time_us = sys_time();
	// process the data
	uint16_t udp_size = p->len;
	memcpy(udp_rx.bytes, p->payload, udp_size);

	/* data are big endian and need to be converted to little endian */
	int size = udp_size / 2;
	for (int i = 0; i < size; i++) {
		udp_rx.values[i] = __builtin_bswap16(udp_rx.values[i]);
	}

	/* handle data here and distribute to individual SPI channels */
	memcpy(spi1_tx_buffer, udp_rx.values, udp_size);
	memcpy(spi2_tx_buffer, udp_rx.values, udp_size);
	memcpy(spi3_tx_buffer, udp_rx.values, udp_size);
	memcpy(spi4_tx_buffer, udp_rx.values, udp_size);

	// signal main to initiate SPI transfer
	spi_data_ready = 1;

	pbuf_free(p);
}

uint32_t sys_time(void) {
	uint32_t current = sys_time_ms * 1000 + (TIM2->CNT) / 240;
	return (current); // current time in microseconds

}

/**************************************************************/

void SPI1_DMA_txfer(void) {
	// 1. Clear ALL relevant SPI flags (EOT, TXTF, and Errors)
	// If any error flags are set, the SPI might refuse to start CSTART
	LL_SPI_ClearFlag_EOT(SPI1);
	LL_SPI_ClearFlag_TXTF(SPI1);
	LL_SPI_ClearFlag_OVR(SPI1);
	LL_SPI_ClearFlag_UDR(SPI1);

	// 2. Cache Maintenance
	SCB_CleanDCache_by_Addr((uint32_t*) spi1_tx_buffer, SPI_BUFFER_SIZE_BYTES);

	// 3. Ensure both DMA streams are fully disabled before reconfiguration
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
	while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_0))
		;
	while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_1))
		;

	// 4. Clear DMA Interrupt Flags
	LL_DMA_ClearFlag_TC0(DMA1); // TX Stream
	LL_DMA_ClearFlag_TC1(DMA1); // RX Stream

	// 5. Re-set lengths
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, NUM_ACTUATORS);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, NUM_ACTUATORS);

	// 6. Enable DMA Streams (RX FIRST!)
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);

	// 7. SPI Configuration & Start
	LL_SPI_SetTransferSize(SPI1, NUM_ACTUATORS);

	// Enable the transfer complete IRQ
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);

	// CRITICAL: Ensure DMA Requests are enabled in the SPI peripheral
	LL_SPI_EnableDMAReq_RX(SPI1);
	LL_SPI_EnableDMAReq_TX(SPI1);

	if (!LL_SPI_IsEnabled(SPI1)) {
		// Add this right before LL_SPI_Enable(...)
		while (LL_SPI_IsActiveFlag_RXP(SPI1)) {
			(void) LL_SPI_ReceiveData16(SPI1); // Read and discard
		}
		LL_SPI_Enable(SPI1);
	}
	// Trigger the transaction
	LL_SPI_StartMasterTransfer(SPI1);
}

void SPI2_DMA_txfer(void) {
	// 1. Clear ALL relevant SPI flags (EOT, TXTF, and Errors)
	// If any error flags are set, the SPI might refuse to start CSTART
	LL_SPI_ClearFlag_EOT(SPI2);
	LL_SPI_ClearFlag_TXTF(SPI2);
	LL_SPI_ClearFlag_OVR(SPI2);
	LL_SPI_ClearFlag_UDR(SPI2);

	// 2. Cache Maintenance
	SCB_CleanDCache_by_Addr((uint32_t*) spi2_tx_buffer, SPI_BUFFER_SIZE_BYTES);

	// 3. Ensure both DMA streams are fully disabled before reconfiguration
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_2);
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
	while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_2))
		;
	while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_3))
		;

	// 4. Clear DMA Interrupt Flags
	LL_DMA_ClearFlag_TC2(DMA1); // TX Stream
	LL_DMA_ClearFlag_TC3(DMA1); // RX Stream

	// 5. Re-set lengths
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_2, NUM_ACTUATORS);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, NUM_ACTUATORS);

	// 6. Enable DMA Streams (RX FIRST!)
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_2);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);

	// 7. SPI Configuration & Start
	LL_SPI_SetTransferSize(SPI2, NUM_ACTUATORS);

	// Enable the transfer complete IRQ
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_3);

	// CRITICAL: Ensure DMA Requests are enabled in the SPI peripheral
	LL_SPI_EnableDMAReq_RX(SPI2);
	LL_SPI_EnableDMAReq_TX(SPI2);

	if (!LL_SPI_IsEnabled(SPI2)) {
		// Add this right before LL_SPI_Enable(...)
		while (LL_SPI_IsActiveFlag_RXP(SPI2)) {
			(void) LL_SPI_ReceiveData16(SPI2); // Read and discard
		}
		LL_SPI_Enable(SPI2);
	}
	// Trigger the transaction
	LL_SPI_StartMasterTransfer(SPI2);
}

void SPI3_DMA_txfer(void) {
	// 1. Clear ALL relevant SPI flags (EOT, TXTF, and Errors)
	// If any error flags are set, the SPI might refuse to start CSTART
	LL_SPI_ClearFlag_EOT(SPI3);
	LL_SPI_ClearFlag_TXTF(SPI3);
	LL_SPI_ClearFlag_OVR(SPI3);
	LL_SPI_ClearFlag_UDR(SPI3);

	// 2. Cache Maintenance
	SCB_CleanDCache_by_Addr((uint32_t*) spi3_tx_buffer, SPI_BUFFER_SIZE_BYTES);

	// 3. Ensure both DMA streams are fully disabled before reconfiguration
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1);
	while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_0))
		;
	while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_1))
		;

	// 4. Clear DMA Interrupt Flags
	LL_DMA_ClearFlag_TC0(DMA2); // TX Stream
	LL_DMA_ClearFlag_TC1(DMA2); // RX Stream

	// 5. Re-set lengths
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, NUM_ACTUATORS);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, NUM_ACTUATORS);

	// 6. Enable DMA Streams
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);

	// 7. SPI Configuration & Start
	LL_SPI_SetTransferSize(SPI3, NUM_ACTUATORS);

	// Enable the transfer complete IRQ
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_1);

	// CRITICAL: Ensure DMA Requests are enabled in the SPI peripheral
	LL_SPI_EnableDMAReq_RX(SPI3);
	LL_SPI_EnableDMAReq_TX(SPI3);

	if (!LL_SPI_IsEnabled(SPI3)) {
		// Add this right before LL_SPI_Enable(...)
		while (LL_SPI_IsActiveFlag_RXP(SPI3)) {
			(void) LL_SPI_ReceiveData16(SPI3); // Read and discard
		}
		LL_SPI_Enable(SPI3);
	}
	// Trigger the transaction
	LL_SPI_StartMasterTransfer(SPI3);
}

void SPI4_DMA_txfer(void) {
	// 1. Clear ALL relevant SPI flags (EOT, TXTF, and Errors)
	// If any error flags are set, the SPI might refuse to start CSTART
	LL_SPI_ClearFlag_EOT(SPI4);
	LL_SPI_ClearFlag_TXTF(SPI4);
	LL_SPI_ClearFlag_OVR(SPI4);
	LL_SPI_ClearFlag_UDR(SPI4);

	// 2. Cache Maintenance
	SCB_CleanDCache_by_Addr((uint32_t*) spi4_tx_buffer, SPI_BUFFER_SIZE_BYTES);

	// 3. Ensure both DMA streams are fully disabled before reconfiguration
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_3);
	while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_2))
		;
	while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_3))
		;

	// 4. Clear DMA Interrupt Flags
	LL_DMA_ClearFlag_TC2(DMA2); // TX Stream
	LL_DMA_ClearFlag_TC3(DMA2); // RX Stream

	// 5. Re-set lengths
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, NUM_ACTUATORS);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_3, NUM_ACTUATORS);

	// 6. Enable DMA Streams
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_3);

	// 7. SPI Configuration & Start
	LL_SPI_SetTransferSize(SPI4, NUM_ACTUATORS);

	// Enable the transfer complete IRQ
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_3);

	// CRITICAL: Ensure DMA Requests are enabled in the SPI peripheral
	LL_SPI_EnableDMAReq_RX(SPI4);
	LL_SPI_EnableDMAReq_TX(SPI4);

	if (!LL_SPI_IsEnabled(SPI4)) {
		// Add this right before LL_SPI_Enable(...)
		while (LL_SPI_IsActiveFlag_RXP(SPI4)) {
			(void) LL_SPI_ReceiveData16(SPI4); // Read and discard
		}
		LL_SPI_Enable(SPI4);
	}
	// Trigger the transaction
	LL_SPI_StartMasterTransfer(SPI4);
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30020000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512B;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
	while (1) {
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
