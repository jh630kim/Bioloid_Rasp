
//*****************************************************************
//    �Լ�����(com_test�ϱ� 6_serial���� ���� ����, ���� �Ժη� �ٲ��� ����)
//    6_serial���� ���ٰ�? �ȵǴµ�... (RaspPi)
//*****************************************************************
/* DOS�� Timer... 
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
//    ��ũ�� ����
//*****************************************************************
