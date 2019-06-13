////////////////////////////////////////////////////////////////////
// �ۼ��� : 051231
// ��  �� : Timer Interrutp
// ��  �� : ���� �ð��������� ���ͷ�Ʈ�� �߻��ϴ� ���α׷�
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
//    ���� ���� ���� ����
//*****************************************************************
volatile unsigned int	g_Send_Flag;

extern unsigned char g_Tx_Packet[TX_QUEUE_LENGTH][BUFFER_LENGTH];
extern volatile unsigned char g_Tx_Qfirst;		// g_Qfirst : ���� �����͸� �о�� �� ������ ����Ŵ.
extern volatile unsigned char g_Tx_Qlast;		// g_Qlast : ���� �Է��� ��������ġ�� ����Ŵ.
extern int serial_fd;

//*****************************************************************
//   �Լ�.
//*****************************************************************
// ���� ���ǵǴ� interrrupt handdler
// �� 10msec(1/60)���� ���ͷ�Ʈ�� �ɸ���.
void CurrTime(void)
{
	struct timeval ts;
	gettimeofday(&ts, NULL);
	printf ("%ld.%06ld usec\n", ts.tv_sec, ts.tv_usec);
}

void timer_handler (int signum)
{
	// JHK_debug
	// Timer Interrupt�� �ֱ� ����.
	/*
	struct timeval ts;
	gettimeofday(&ts, NULL);
	printf ("%ld.%06ld usec\n", ts.tv_sec, ts.tv_usec);
	*/
	// Tx_Queue�� ������ ������ �۽�, timer_handler()
	piLock(KEY_WRITE);
	// (V)g_Tx_Qfirst, (V)g_Tx_Qlast, (?)g_Send_Flag, (V)g_Tx_Packet[][],
	if (IsReadyToTX && g_Send_Flag) 
	// g_Send_Flag�� ������ �� �𸣰ڴ�. ���... �ʿ���� ������ �� ����.
	// �ٸ����� �ٲٴ� �߿� ������ ���Ѵ� �ص�... ���� ���ͷ�Ʈ�� Clear�� ���ݾ�?
	{
		SET_RTS;			// �۽� ���� ����
		write(serial_fd,g_Tx_Packet[g_Tx_Qfirst],g_Tx_Packet[g_Tx_Qfirst][3]+4);
		// RESET_RTS;	// �����͸� ���� �ʿ䰡 ����. ���Ÿ��� �������� ����.
		// Queue First�� �ϳ� ����.
		NEXT_TX_FIRST;
	}
	piUnlock(KEY_WRITE);
}

// Timer ����
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

