
typedef struct
{
	uint32_t int_et;
	uint32_t div_et;

} exec_time_struct;



void DWT_Init(void);
void start_exec_time(void);
exec_time_struct stop_exec_time(void);
float stop_exec_time_float(void);

