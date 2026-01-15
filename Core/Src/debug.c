#include <stdio.h>
#include <string.h>

#include <stdint.h>
#include "cmsis_gcc.h"

//#include "stm32h5xx_hal.h"
#include "stm32h7xx_hal.h"
#include "common_defs.h"
#include "debug.h"

extern UART_HandleTypeDef UART_DEBUG;

uint32_t sysclk;
float cycles_per_us,  us_per_cycles;
volatile uint32_t time_tick;
volatile uint32_t time_dwt;

// 콜백 함수 저장용 전역 변수
static ErrorCallback error_callback = NULL;
// log 메세지 출력용 변수
LogFifo_t log_fifo;
char log_msg[MAX_LOG_MSG_LEN];

//---------------------------------------------------------------------------------------------------------------------------------------------
#define HAL_UART_MAX_DELAY 10

//int _write(int file, char *ptr, int len)
//{
//    HAL_UART_Transmit(UART_DEBUG_ADD, (uint8_t *)ptr, len, HAL_UART_MAX_DELAY);
//    return len;
//}
//
//int _read(int file, char *ptr, int len)
//{
//    int DataIdx = 0;
//    uint8_t receivedChar;
//
//    for (DataIdx = 0; DataIdx < len; DataIdx++)
//    {
//        HAL_UART_Receive(UART_DEBUG_ADD, &receivedChar, 1, HAL_MAX_DELAY);
//        ptr[DataIdx] = receivedChar;
//        if (receivedChar == '\n' || receivedChar == '\r') // Enter Ű ó��
//        {
//            ptr[DataIdx] = '\0';
//			HAL_UART_Transmit(UART_DEBUG_ADD, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
//            return DataIdx;
//        }
//    }
//    return DataIdx;
//}

int __io_putchar(int ch)
{
	if(HAL_UART_Transmit(UART_DEBUG_ADD, (uint8_t *)&ch, 1, 10) != HAL_OK)
		return -1;
	return ch;
}

int __io_getchar(void)
{
	char data[4];
	uint8_t ch, len = 1;

	while(HAL_UART_Receive(UART_DEBUG_ADD, &ch, 1, 10) != HAL_OK);

	memset(data, 0x00, 4);
	switch(ch)
	{
		case '\r':
		case '\n':
			len = 2;
			sprintf(data, "\r\n");
			break;

		case '\b':
		case 0x7F:
			len = 3;
			sprintf(data, "\b \b");
			break;

		default:
			data[0] = ch;
			break;
	}
	HAL_UART_Transmit(UART_DEBUG_ADD, (uint8_t *)data, len, 10);
	return ch;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	DWT->LAR = 0xC5ACCE55;
	
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
//	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; /* Disable counter */

	__DSB(); __ISB();
//	sysclk = HAL_RCC_GetSysClockFreq();
	sysclk         = SystemCoreClock;
	cycles_per_us = sysclk/1000000.0f;
	us_per_cycles = 1000000.0f/sysclk;

	printf("SYSCLK = %lu Hz\r\n", sysclk);
}

void DWT_Start(void)
{
	time_dwt = DWT->CYCCNT;
//	DWT->CYCCNT = 0; /* Clear count of clock cycles */
//	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; /* Enable counter */
}

void DWT_Stop(void)
{
	uint32_t elapsed_cycle = DWT->CYCCNT - time_dwt;
//	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; /* Disable counter */
//  	volatile uint32_t elapsed_cycle = DWT->CYCCNT; /* Read count of clock cycles */
	
	float elapsed_time_us = (float)elapsed_cycle / cycles_per_us;
//	printf("Elapsed Cycle: %lu, Elapsed time: %.3f us\r\n", elapsed_cycle, elapsed_time_us);
//	printf("%.3f us\r\n", elapsed_time_us);

	snprintf(log_msg, MAX_LOG_MSG_LEN, "%.3f us\r\n", elapsed_time_us);
	LogFifo_Push(log_msg);
}

void delay_cycle(uint32_t cycles)
{
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t delay_cycles = us * cycles_per_us;
    while ((DWT->CYCCNT - start) < delay_cycles);
}

void delay_ms(uint32_t ms)
{
    while (ms--)
        delay_us(1000);
}

void Timer_Start(void)
{
    time_tick = HAL_GetTick();
//    printf("Timer started at %lu ms\r\n", time_tick);
}

void Timer_Stop(void)
{
    uint32_t current_tick = HAL_GetTick();
    uint32_t elapsed_ms = current_tick - time_tick;
//	printf("Timer stopped at %lu ms\r\n", current_tick);
    printf("Elapsed time: %lu ms\r\n", elapsed_ms);
}

//---------------------------------------------------------------------------------------------------------------------------------------------
// 콜백 함수 등록
void RegisterErrorCallback(ErrorCallback callback)
{
    error_callback = callback;
}

// 에러 보고 함수
void ReportError(const char *func, const char *file, uint32_t line, uint32_t error_code, const char *message)
{
    if (error_callback != NULL) {
        ErrorInfo error_info = {
            .function_name = func,
            .file_name = file,
            .line_number = line,
            .error_code = error_code,
            .error_message = message
        };
        error_callback(&error_info);
    }
}

void MyErrorCallback(ErrorInfo *error_info)
{
    printf("Error!!\r\nFunction: %s\r\n", error_info->function_name);
    printf("File: %s, Line: %lu\r\n", error_info->file_name, error_info->line_number);
}

// 버전 1 콜백: 함수, 파일, 라인, 에러 메시지 출력
void MyErrorCallback_Msg(ErrorInfo *error_info)
{
    printf("Error!!\r\nFunction: %s\r\n", error_info->function_name);
    printf("File: %s, Line: %lu\r\n", error_info->file_name, error_info->line_number);
    printf("Message: %s\r\n", error_info->error_message);
}

// 버전 2 콜백: 함수, 파일, 라인, 에러 코드 출력
void MyErrorCallback_Code_Msg(ErrorInfo *error_info)
{
    printf("Error!!\r\nFunction: %s\r\n", error_info->function_name);
    printf("File: %s, Line: %lu\r\n", error_info->file_name, error_info->line_number);
    printf("Error Code: %ld, Message: %s\n", error_info->error_code, error_info->error_message);
}

//---------------------------------------------------------------------------------------------------------------------------------------------
// FIFO 버퍼 초기화
void LogFifo_Init(void)
{
    log_fifo.head = 0;
    log_fifo.tail = 0;
    log_fifo.count = 0;
}

// FIFO에 메시지 추가 (ISR에서 호출)
int LogFifo_Push(const char *msg)
{
    if (log_fifo.count >= LOG_BUFFER_SIZE)
	{
        return -1; // 버퍼가 가득 참 (오버플로우 처리)
    }
    
//    strncpy(log_fifo.buffer[log_fifo.head], msg, MAX_LOG_MSG_LEN - 1);
//    log_fifo.buffer[log_fifo.head][MAX_LOG_MSG_LEN - 1] = '\0'; // Null 종료 보장

    // 문자열 길이 제한 후 복사, 널 종단 보장
    size_t copy_len;
    copy_len = strnlen(msg, (size_t)(MAX_LOG_MSG_LEN - 1));
    memcpy(log_fifo.buffer[log_fifo.head], msg, copy_len);
    log_fifo.buffer[log_fifo.head][copy_len] = '\0';
    
    log_fifo.head = (log_fifo.head + 1U) % LOG_BUFFER_SIZE;
    log_fifo.count++;
	
    return 0;
}

// FIFO에서 메시지 읽기 (메인 루프에서 호출)
int LogFifo_Pop(char *msg)
{
	uint32_t primask;
    primask = __get_PRIMASK();
    __disable_irq();

    if (log_fifo.count == 0)
	{
		__set_PRIMASK(primask);
        return -1; // 버퍼가 비어 있음
    }
    
//    strncpy(msg, log_fifo.buffer[log_fifo.tail], MAX_LOG_MSG_LEN);

    // 복사 및 널 종단
    size_t copy_len;
    copy_len = strnlen(log_fifo.buffer[log_fifo.tail], (size_t)(MAX_LOG_MSG_LEN - 1));
    memcpy(msg, log_fifo.buffer[log_fifo.tail], copy_len);
    msg[copy_len] = '\0';
	
    log_fifo.tail = (log_fifo.tail + 1U) % LOG_BUFFER_SIZE;
    log_fifo.count--;

	__set_PRIMASK(primask);
    return 0;
}

