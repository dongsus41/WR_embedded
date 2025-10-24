#ifndef __DEBUG_H
#define __DEBUG_H

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>

//int _write(int file, char *ptr, int len);
//int _read(int file, char *ptr, int len);
int __io_putchar(int ch);
int __io_getchar(void);

void DWT_Init(void);
void DWT_Start(void);
void DWT_Stop(void);
void delay_cycle(uint32_t cycles);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

void Timer_Start(void);
void Timer_Stop(void);

//---------------------------------------------------------------------------------------------------------------------------------------------
// 에러 정보 구조체
typedef struct
{
    const char *function_name;
    const char *file_name;
    uint32_t line_number;
    int32_t error_code;
    const char *error_message;
} ErrorInfo;

// 콜백 함수 포인터 타입 정의
typedef void (*ErrorCallback)(ErrorInfo *error_info);

void RegisterErrorCallback(ErrorCallback callback);
void ReportError(const char *func, const char *file, uint32_t line, uint32_t error_code, const char *message);
void MyErrorCallback(ErrorInfo *error_info);
void MyErrorCallback_Msg(ErrorInfo *error_info);
void MyErrorCallback_Code_Msg(ErrorInfo *error_info);

// 매크로를 사용해 간편하게 에러 보고
#define REPORT_ERROR() ReportError(__FUNCTION__, __FILE__, __LINE__, 0U, "")
#define REPORT_ERROR_MSG(msg) ReportError(__FUNCTION__, __FILE__, __LINE__, 0U, msg)
#define REPORT_ERROR_CODE_MSG(code, msg) ReportError(__FUNCTION__, __FILE__, __LINE__, (uint32_t)code, msg)

//---------------------------------------------------------------------------------------------------------------------------------------------
// FIFO 텍스트 버퍼 설정
#define LOG_BUFFER_SIZE 64
#define MAX_LOG_MSG_LEN 128

typedef struct
{
    char buffer[LOG_BUFFER_SIZE][MAX_LOG_MSG_LEN];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t count;
} LogFifo_t;

void LogFifo_Init(void);
int LogFifo_Push(const char *msg);
int LogFifo_Pop(char *msg);

#ifdef __cplusplus
	}
#endif

#endif /* __DEBUG_H */
