/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : Main program body
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    I2C_IDLE,
    I2C_WAIT_TX,
    I2C_WAIT_RX,
    I2C_DONE,
    I2C_ERROR
} I2C_State;

typedef enum {
    I2C_OP_ADC,
    I2C_OP_DAC
} I2C_Operation;

typedef enum {
    SPI_IDLE,
    SPI_BUSY,
    SPI_ERROR
} SPI_State;

typedef enum {
    SPI_OP_NONE,
    SPI_OP_BOOT,
    SPI_OP_FRAME
} SPI_Operation;

typedef enum {
    DISPLAY_NONE,
    DISPLAY_L,
    DISPLAY_T,
    DISPLAY_V
} Display_Sensor;

typedef enum {
    DISPLAY_LETTER,
    DISPLAY_SIGN
} Display_Phase;

typedef enum {
    CMD_INVALID,
    CMD_READ_LDR,
    CMD_READ_TEMP,
    CMD_READ_VOLT,
    CMD_SET_DAC,
    CMD_LED_TEST,
    CMD_LED_CLEAR,
    CMD_HELP
} Command_Type;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PCF8591_ADDR_HAL       (0x48U << 1)
#define UART_COMMAND_LEN       50U
#define DISPLAY_INTERVAL_MS    500U
#define SPI_TIMEOUT_MS         100U
#define MAX7219_BOOT_COMMANDS  14U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile I2C_State i2c_state = I2C_IDLE;
static volatile I2C_Operation i2c_operation = I2C_OP_ADC;
static volatile uint32_t i2c_error_code = 0;
static volatile uint32_t i2c_start_status = 0;
static uint8_t i2c_control = 0;
static uint8_t adc_buffer[2] = {0};
static uint8_t dac_buffer[2] = {0};
static uint8_t current_channel = 0;
static uint8_t dac_value = 128;
static uint8_t LDR = 0, Temp = 0, Pot = 0;

static uint8_t uart_rx_char = 0;
static char command_buffer[UART_COMMAND_LEN] = {0};
static volatile uint8_t command_index = 0;
static volatile uint8_t command_ready = 0;

static volatile SPI_State spi_state = SPI_IDLE;
static volatile SPI_Operation spi_operation = SPI_OP_NONE;
static volatile uint8_t spi_boot_index = 0;
static volatile uint8_t spi_frame_index = 0;
static volatile uint32_t spi_error_code = 0;
static uint32_t spi_start_tick = 0;
static uint8_t spi_error_reported = 0;
static uint8_t spi_frame_running = 0;
static uint8_t spi_frame_pending = 0;
static uint8_t spi_tx_buffer[2] = {0};
static uint8_t spi_frame_active[8] = {0};
static uint8_t spi_frame_requested[8] = {0};

static Display_Sensor display_sensor = DISPLAY_NONE;
static Display_Phase display_phase = DISPLAY_LETTER;
static uint8_t display_positive = 0;
static uint32_t display_last_tick = 0;

static const uint8_t GLYPH_L[8] = {
    0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7E, 0x00
};
static const uint8_t GLYPH_T[8] = {
    0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00
};
static const uint8_t GLYPH_V[8] = {
    0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x00
};
static const uint8_t GLYPH_PLUS[8] = {
    0x00, 0x18, 0x18, 0x7E, 0x7E, 0x18, 0x18, 0x00
};
static const uint8_t GLYPH_MINUS[8] = {
    0x00, 0x00, 0x00, 0x7E, 0x7E, 0x00, 0x00, 0x00
};
static const uint8_t GLYPH_TEST[8] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
static const uint8_t GLYPH_CLEAR[8] = {
    0, 0, 0, 0, 0, 0, 0, 0
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void UART_Send(const char *message);
static void UART_Start_Receive(void);
static Command_Type Parse_Command(const char *command);
static void Process_Command(const char *command);
static uint8_t Parse_DAC_Value(const char *text, uint8_t *value);
static void UART_Process(void);

static void PCF8591_StartRead(uint8_t channel);
static void PCF8591_StartWriteDAC(uint8_t value);
static void I2C_Process(void);

static void SPI_StartWord(uint8_t address, uint8_t value, SPI_Operation operation);
static void SPI_Process(void);
static void SPI_GetBootWord(uint8_t index, uint8_t *address, uint8_t *value);
static void SPI_QueueFrame(const uint8_t frame[8]);
static void Display_Publish(void);
static void Display_SetMeasurement(Display_Sensor sensor, uint8_t value);
static void Display_Process(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UART_Send(const char *message)
{

    (void)HAL_UART_Transmit(&huart2, (uint8_t *)message,
                            (uint16_t)strlen(message), 100U);
}

static void UART_Start_Receive(void)
{
    if (HAL_UART_Receive_IT(&huart2, &uart_rx_char, 1U) != HAL_OK) {
        Error_Handler();
    }
}

static Command_Type Parse_Command(const char *command)
{

    if (strcmp(command, "Read_AIN0") == 0 || strcmp(command, "LDR") == 0) {
        return CMD_READ_LDR;
    }
    if (strcmp(command, "Read_AIN1") == 0 || strcmp(command, "Temp") == 0) {
        return CMD_READ_TEMP;
    }
    if (strcmp(command, "Read_AIN3") == 0 || strcmp(command, "Volt") == 0) {
        return CMD_READ_VOLT;
    }
    if (strncmp(command, "Set_DAC_", 8U) == 0) {
        return CMD_SET_DAC;
    }
    if (strcmp(command, "LED_TEST") == 0) {
        return CMD_LED_TEST;
    }
    if (strcmp(command, "LED_CLEAR") == 0) {
        return CMD_LED_CLEAR;
    }
    if (strcmp(command, "HELP") == 0) {
        return CMD_HELP;
    }
    return CMD_INVALID;
}

static uint8_t Parse_DAC_Value(const char *text, uint8_t *value)
{
    uint16_t number = 0;

    if (*text == '\0') {
        return 0U;
    }

    while (*text != '\0') {
        if (*text < '0' || *text > '9') {
            return 0U;
        }
        number = (uint16_t)(number * 10U + (uint16_t)(*text - '0'));
        if (number > 255U) {
            return 0U;
        }
        ++text;
    }
    *value = (uint8_t)number;
    return 1U;
}

static void Process_Command(const char *command)
{
    uint8_t value = 0;

    switch (Parse_Command(command)) {
        case CMD_READ_LDR:
            PCF8591_StartRead(0U);
            break;

        case CMD_READ_TEMP:
            PCF8591_StartRead(1U);
            break;

        case CMD_READ_VOLT:
            PCF8591_StartRead(3U);
            break;

        case CMD_SET_DAC:
            if (Parse_DAC_Value(&command[8], &value) != 0U) {
                PCF8591_StartWriteDAC(value);
            } else {
                UART_Send("Valor DAC invalido! Use 0 a 255.\r\n");
            }
            break;

        case CMD_LED_TEST:
            display_sensor = DISPLAY_NONE;
            SPI_QueueFrame(GLYPH_TEST);
            UART_Send("LED_TEST: todos os LEDs ligados.\r\n");
            break;

        case CMD_LED_CLEAR:
            display_sensor = DISPLAY_NONE;
            SPI_QueueFrame(GLYPH_CLEAR);
            UART_Send("LED_CLEAR: matriz apagada.\r\n");
            break;

        case CMD_HELP:
            UART_Send("Read_AIN0 | Read_AIN1 | Read_AIN3 | Set_DAC_0..255\r\n"
                      "LDR | Temp | Volt | LED_TEST | LED_CLEAR | HELP\r\n");
            break;

        case CMD_INVALID:
        default:
            UART_Send("Comando invalido! Digite HELP.\r\n");
            break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) {
        return;
    }

    if (command_ready == 0U) {
        switch (uart_rx_char) {
            case '\r':
            case '\n':
                if (command_index > 0U) {
                    command_buffer[command_index] = '\0';
                    command_ready = 1U;
                }
                break;

            default:
                if (command_index < (UART_COMMAND_LEN - 1U)) {
                    command_buffer[command_index] = (char)uart_rx_char;
                    ++command_index;
                }
                break;
        }
    }

    (void)HAL_UART_Receive_IT(&huart2, &uart_rx_char, 1U);
}

static void UART_Process(void)
{
    char command[UART_COMMAND_LEN];

    if (command_ready == 0U || i2c_state != I2C_IDLE) {
        return;
    }

    __disable_irq();
    memcpy(command, command_buffer, UART_COMMAND_LEN);
    command_ready = 0U;
    command_index = 0U;
    __enable_irq();

    Process_Command(command);
}

static void PCF8591_StartRead(uint8_t channel)
{
    HAL_StatusTypeDef status;

    switch (channel) {
        case 0U:
        case 1U:
        case 3U:
            break;
        default:
            return;
    }

    if (i2c_state != I2C_IDLE) {
        return;
    }

    current_channel = channel;
    i2c_control = channel;
    i2c_operation = I2C_OP_ADC;
    i2c_error_code = 0U;
    i2c_start_status = 0U;
    i2c_state = I2C_WAIT_TX;

    status = HAL_I2C_Master_Transmit_IT(&hi2c1, PCF8591_ADDR_HAL,
                                         &i2c_control, 1U);
    if (status != HAL_OK) {
        i2c_start_status = (uint32_t)status;
        i2c_state = I2C_ERROR;
    }
}

static void PCF8591_StartWriteDAC(uint8_t value)
{
    HAL_StatusTypeDef status;

    if (i2c_state != I2C_IDLE) {
        return;
    }

    dac_value = value;
    dac_buffer[0] = 0x40U;
    dac_buffer[1] = value;
    i2c_operation = I2C_OP_DAC;
    i2c_error_code = 0U;
    i2c_start_status = 0U;
    i2c_state = I2C_WAIT_TX;

    status = HAL_I2C_Master_Transmit_IT(&hi2c1, PCF8591_ADDR_HAL,
                                         dac_buffer, 2U);
    if (status != HAL_OK) {
        i2c_start_status = (uint32_t)status;
        i2c_state = I2C_ERROR;
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;

    if (hi2c->Instance != I2C1 || i2c_state != I2C_WAIT_TX) {
        return;
    }

    switch (i2c_operation) {
        case I2C_OP_ADC:
            i2c_state = I2C_WAIT_RX;

            status = HAL_I2C_Master_Receive_IT(&hi2c1, PCF8591_ADDR_HAL,
                                                adc_buffer, 2U);
            if (status != HAL_OK) {
                i2c_start_status = (uint32_t)status;
                i2c_state = I2C_ERROR;
            }
            break;

        case I2C_OP_DAC:
            i2c_state = I2C_DONE;
            break;

        default:
            i2c_state = I2C_ERROR;
            break;
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1 && i2c_state == I2C_WAIT_RX) {
        i2c_state = I2C_DONE;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        i2c_error_code = HAL_I2C_GetError(hi2c);
        i2c_state = I2C_ERROR;
    }
}

static void I2C_Process(void)
{
    char message[80];

    switch (i2c_state) {
        case I2C_DONE:
            switch (i2c_operation) {
                case I2C_OP_ADC:
                    switch (current_channel) {
                        case 0U:
                            LDR = adc_buffer[1];
                            snprintf(message, sizeof(message),
                                     "AIN0 (LDR): %u\r\n", (unsigned)LDR);
                            Display_SetMeasurement(DISPLAY_L, LDR);
                            break;

                        case 1U:
                            Temp = adc_buffer[1];
                            snprintf(message, sizeof(message),
                                     "AIN1 (Temp): %u\r\n", (unsigned)Temp);
                            Display_SetMeasurement(DISPLAY_T, Temp);
                            break;

                        case 3U:
                            Pot = adc_buffer[1];
                            snprintf(message, sizeof(message),
                                     "AIN3 (Pot): %u\r\n", (unsigned)Pot);
                            Display_SetMeasurement(DISPLAY_V, Pot);
                            break;

                        default:
                            snprintf(message, sizeof(message),
                                     "Canal I2C invalido.\r\n");
                            break;
                    }
                    break;

                case I2C_OP_DAC:
                    snprintf(message, sizeof(message),
                             "Valor do DAC: %u\r\n", (unsigned)dac_value);
                    break;

                default:
                    snprintf(message, sizeof(message),
                             "Operacao I2C invalida.\r\n");
                    break;
            }
            i2c_state = I2C_IDLE;
            UART_Send(message);
            break;

        case I2C_ERROR:
            snprintf(message, sizeof(message),
                     "Erro I2C: status=%lu, erro=0x%lX\r\n",
                     (unsigned long)i2c_start_status,
                     (unsigned long)i2c_error_code);
            i2c_state = I2C_IDLE;
            UART_Send(message);
            break;

        case I2C_IDLE:
        case I2C_WAIT_TX:
        case I2C_WAIT_RX:
        default:
            break;
    }
}

static void SPI_QueueFrame(const uint8_t frame[8])
{
    memcpy(spi_frame_requested, frame, 8U);
    spi_frame_pending = 1U;
}

static void Display_Publish(void)
{
    const uint8_t *glyph = GLYPH_CLEAR;

    switch (display_phase) {
        case DISPLAY_LETTER:
            switch (display_sensor) {
                case DISPLAY_L:
                    glyph = GLYPH_L;
                    break;
                case DISPLAY_T:
                    glyph = GLYPH_T;
                    break;
                case DISPLAY_V:
                    glyph = GLYPH_V;
                    break;
                case DISPLAY_NONE:
                default:
                    glyph = GLYPH_CLEAR;
                    break;
            }
            break;

        case DISPLAY_SIGN:
            switch (display_positive) {
                case 0U:
                    glyph = GLYPH_MINUS;
                    break;
                case 1U:
                default:
                    glyph = GLYPH_PLUS;
                    break;
            }
            break;

        default:
            glyph = GLYPH_CLEAR;
            break;
    }
    SPI_QueueFrame(glyph);
}

static void Display_SetMeasurement(Display_Sensor sensor, uint8_t value)
{
    display_sensor = sensor;
    display_positive = (value >= 128U) ? 1U : 0U;
    display_phase = DISPLAY_LETTER;
    display_last_tick = HAL_GetTick();
    Display_Publish();
}

static void Display_Process(void)
{
    if (display_sensor == DISPLAY_NONE) {
        return;
    }

    if ((uint32_t)(HAL_GetTick() - display_last_tick) < DISPLAY_INTERVAL_MS) {
        return;
    }

    display_last_tick = HAL_GetTick();

    switch (display_phase) {
        case DISPLAY_LETTER:
            display_phase = DISPLAY_SIGN;
            break;
        case DISPLAY_SIGN:
        default:
            display_phase = DISPLAY_LETTER;
            break;
    }
    Display_Publish();
}

static void SPI_StartWord(uint8_t address, uint8_t value,
                          SPI_Operation operation)
{
    HAL_StatusTypeDef status;

    if (spi_state != SPI_IDLE) {
        return;
    }

    spi_tx_buffer[0] = address;
    spi_tx_buffer[1] = value;
    spi_operation = operation;
    spi_state = SPI_BUSY;
    spi_start_tick = HAL_GetTick();

    HAL_GPIO_WritePin(MAX7219_CS_GPIO_Port, MAX7219_CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit_IT(&hspi1, spi_tx_buffer, 2U);

    if (status != HAL_OK) {
        HAL_GPIO_WritePin(MAX7219_CS_GPIO_Port, MAX7219_CS_Pin, GPIO_PIN_SET);
        spi_error_code = (uint32_t)status;
        spi_state = SPI_ERROR;
    }
}

static void SPI_GetBootWord(uint8_t index, uint8_t *address, uint8_t *value)
{
    switch (index) {
        case 0U:
            *address = 0x0CU; *value = 0x00U;
            break;
        case 1U:
            *address = 0x0FU; *value = 0x00U;
            break;
        case 2U:
            *address = 0x09U; *value = 0x00U;
            break;
        case 3U:
            *address = 0x0BU; *value = 0x07U;
            break;
        case 4U:
            *address = 0x0AU; *value = 0x02U;
            break;
        case 13U:
            *address = 0x0CU; *value = 0x01U;
            break;
        default:
            *address = (uint8_t)(index - 4U);
            *value = 0x00U;
            break;
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1 || spi_state != SPI_BUSY) {
        return;
    }

    HAL_GPIO_WritePin(MAX7219_CS_GPIO_Port, MAX7219_CS_Pin, GPIO_PIN_SET);

    switch (spi_operation) {
        case SPI_OP_BOOT:
            ++spi_boot_index;
            break;
        case SPI_OP_FRAME:
            ++spi_frame_index;
            break;
        case SPI_OP_NONE:
        default:
            break;
    }
    spi_operation = SPI_OP_NONE;
    spi_state = SPI_IDLE;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        HAL_GPIO_WritePin(MAX7219_CS_GPIO_Port, MAX7219_CS_Pin, GPIO_PIN_SET);
        spi_error_code = HAL_SPI_GetError(hspi);
        spi_state = SPI_ERROR;
    }
}

static void SPI_Process(void)
{
    uint8_t address = 0, value = 0;
    char message[64];

    switch (spi_state) {
        case SPI_BUSY:
            if ((uint32_t)(HAL_GetTick() - spi_start_tick) > SPI_TIMEOUT_MS) {

                (void)HAL_SPI_Abort(&hspi1);
                HAL_GPIO_WritePin(MAX7219_CS_GPIO_Port,
                                  MAX7219_CS_Pin, GPIO_PIN_SET);
                spi_error_code = 0xFFFFFFFFUL;
                spi_state = SPI_ERROR;
            }
            return;

        case SPI_ERROR:
            if (spi_error_reported == 0U) {
                snprintf(message, sizeof(message),
                         "Erro SPI1: codigo=0x%lX\r\n",
                         (unsigned long)spi_error_code);
                UART_Send(message);
                spi_error_reported = 1U;
            }
            return;

        case SPI_IDLE:
        default:
            break;
    }

    if (spi_boot_index < MAX7219_BOOT_COMMANDS) {
        SPI_GetBootWord(spi_boot_index, &address, &value);
        SPI_StartWord(address, value, SPI_OP_BOOT);
        return;
    }

    if (spi_frame_running != 0U) {
        if (spi_frame_index < 8U) {
            SPI_StartWord((uint8_t)(spi_frame_index + 1U),
                          spi_frame_active[spi_frame_index], SPI_OP_FRAME);
            return;
        }
        spi_frame_running = 0U;
    }

    if (spi_frame_pending != 0U) {
        memcpy(spi_frame_active, spi_frame_requested, 8U);
        spi_frame_pending = 0U;
        spi_frame_index = 0U;
        spi_frame_running = 1U;
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  UART_Start_Receive();
  SPI_QueueFrame(GLYPH_CLEAR);
  UART_Send("NUCLEO pronta. Digite HELP.\r\n");
/* USER CODE END 2 */
/* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    I2C_Process();
    UART_Process();
    Display_Process();
    SPI_Process();
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(MAX7219_CS_GPIO_Port, MAX7219_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : MAX7219_CS_Pin */
  GPIO_InitStruct.Pin = MAX7219_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MAX7219_CS_GPIO_Port, &GPIO_InitStruct);

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
