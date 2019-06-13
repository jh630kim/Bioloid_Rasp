
//*****************************************************************
//    함수선언(com_test하구 6_serial에서 같이 쓴다, 형을 함부로 바꾸지 말자)
//    6_serial에서 쓴다고? 안되는데... (RaspPi)
//*****************************************************************
/* DOS용 Timer... 
void interrupt timer_interrupt(...);
void Timer_Interrupt_Init(void);
void Timer_Interrupt_Close(void);
void PC_ElapsedInit(void);
void PC_ElapsedStart(void);
INT16U PC_ElapsedStop(void);
void PC_SetTickRate (unsigned int freq);
*/

void timer_handler (int signum);
void CurrTime(void);
void SetTimer();

//*****************************************************************/
//    매크로 선언
//*****************************************************************
