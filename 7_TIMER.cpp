////////////////////////////////////////////////////////////////////
// 작성일 : 051231
// 제  목 : Timer Interrutp
// 내  용 : 일정 시간간격으로 인터럽트를 발생하는 프로그램
////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "define.h"
#include "timer.h"
#include "debug.h"
#include "serial.h"

#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

//*****************************************************************
//    전역 변수 선언 선언
//*****************************************************************
volatile unsigned int	g_Send_Flag;

extern unsigned char g_Tx_Packet[TX_QUEUE_LENGTH][BUFFER_LENGTH];
extern volatile unsigned char g_Tx_Qfirst;		// g_Qfirst : 지금 데이터를 읽어야 할 지점을 가리킴.
extern volatile unsigned char g_Tx_Qlast;		// g_Qlast : 다음 입력할 버퍼의위치를 가리킴.
extern int serial_fd;

//*****************************************************************
//   함수.
//*****************************************************************
// 새로 정의되는 interrrupt handdler
// 매 10msec(1/60)마다 인터럽트가 걸린다.
void CurrTime(void)
{
	struct timeval ts;
	gettimeofday(&ts, NULL);
	printf ("%ld.%06ld usec\n", ts.tv_sec, ts.tv_usec);
}

void timer_handler (int signum)
{
	// JHK_debug
	// Timer Interrupt의 주기 측정.
	/*
	struct timeval ts;
	gettimeofday(&ts, NULL);
	printf ("%ld.%06ld usec\n", ts.tv_sec, ts.tv_usec);
	*/
	// Tx_Queue의 내용을 꺼내서 송신, timer_handler()
	piLock(KEY_WRITE);
	// (V)g_Tx_Qfirst, (V)g_Tx_Qlast, (?)g_Send_Flag, (V)g_Tx_Packet[][],
	if (IsReadyToTX && g_Send_Flag) 
	// g_Send_Flag의 영향은 잘 모르겠다. 사실... 필요없는 변수인 것 같다.
	// 다른데서 바꾸는 중에 변수가 변한다 해도... 다음 인터럽트때 Clear가 되잖아?
	{
		SET_RTS;			// 송신 모드로 변경
		write(serial_fd,g_Tx_Packet[g_Tx_Qfirst],g_Tx_Packet[g_Tx_Qfirst][3]+4);
		// RESET_RTS;	// 데이터를 읽을 필요가 없다. 수신모드로 변경하지 말자.
		// Queue First를 하나 증가.
		NEXT_TX_FIRST;
	}
	piUnlock(KEY_WRITE);
}

// Timer 선언
void SetTimer()
{
	 struct itimerval timer;
	 
	 /* Configure the timer to expire after 10 msec... */
	 timer.it_value.tv_sec = 0;
	 timer.it_value.tv_usec = 10000;

	 /* ... and every 10 msec after that. */
	 timer.it_interval.tv_sec = 0;
	 timer.it_interval.tv_usec = 10000;

	 // setitimer (ITIMER_VIRTUAL, &timer, NULL);
	 setitimer (ITIMER_REAL, &timer, NULL);
}

