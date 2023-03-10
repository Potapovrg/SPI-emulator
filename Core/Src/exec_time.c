//#ifndef INC_EXEC_TIME_H_
#define INC_EXEC_TIME_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include "exec_time.h"

//#define DWT_CONTROL *(volatile unsigned long *)0xE0001000
//#define SCB_DEMCR   *(volatile unsigned long *)0xE000EDFC

void DWT_Init(void)
 {
 	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
 	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;   // запускаем счётчик
 }

void start_exec_time(void)
 {
	 DWT->CYCCNT = 0U;
 }

exec_time_struct stop_exec_time(void)
 {
	exec_time_struct ext;
	uint32_t DWT_result;
	DWT_result=DWT->CYCCNT;
	ext.int_et=(DWT_result)/48000;
	ext.div_et=(DWT_result)%48000;
	return (ext); //returns in msec
 }

float stop_exec_time_float(void)
 {
	float ext;
	uint32_t DWT_result;
	DWT_result=DWT->CYCCNT;
	ext = DWT_result/48000.0; //returns in msec
	return (ext);

 }


#ifdef __cplusplus
}
#endif


