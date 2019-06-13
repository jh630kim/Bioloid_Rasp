////////////////////////////////////////////////////////////////////
// 작성일 : 051126
// 제  목 : Robot의 AVR과 통신을 하는 프로그램.
// 내  용 : COM2, ?????BPS, 8 data bit, 1 stop bit, no parity로 고정됨.
//          내 PC의 COM1이 고장났다.
////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "define.h"
#include "serial.h"
#include "debug.h"

#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>

#include <unistd.h>

#include <iostream>
#include <string>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

volatile extern unsigned int g_Send_Flag;
extern int	g_Emer_Stop;
// extern int 	select_speed;
//*****************************************************************
// Serial.CPP
//*****************************************************************
// 데이터 저장을 위한 버퍼와 버퍼 정보 변수,
// 수신 버퍼,
// main에서는 g_Qfirst != g_Qlast이면 Read_From_Port를 호출한다.
unsigned char buffer[BUFFER_LENGTH];
unsigned char g_Qfirst = 0;		// g_Qfirst : 지금 데이터를 읽어야 할 지점을 가리킴.
unsigned char g_Qlast = 0;		// g_Qlast : 다음 입력할 버퍼의위치를 가리킴.

// 10msec마다 데이터를 전달하기 위한 큐
// 송신 Packet 버퍼,
// Interrupt에서 g_Tx_Qfirst != g_Tx_Qlast이면 Read_From_Port를 호출한다.
// Timer 인터럽트에서 변경함.
unsigned char g_Tx_Packet[TX_QUEUE_LENGTH][BUFFER_LENGTH];
volatile unsigned char g_Tx_Qfirst = 0;		// g_Qfirst : 지금 데이터를 읽어야 할 지점을 가리킴.
volatile unsigned char g_Tx_Qlast = 0;		// g_Qlast : 다음 입력할 버퍼의위치를 가리킴.

// 좌/우측 다리의 AX-12 모터 ID
const int    LEG_ID[][6]     = {{   8,  10,  12,  14,  16,  18}, {   9,  11,  13,  15,  17,  19}};
const int 	 WAIST_ID		 = 1;
// 조립을 잘못해서 좌우팔의 ID를 바꿨다.
// const int    ARM_ID[][4]     = {{   1,   2,   4,   6}, {   1,  3,  5,  7}};
const int    ARM_ID[][4]     = {{   1,   2,   5,   7}, {   1,  3,  4,  6}};

// FD for Serial & LAN
int serial_fd;	
int socket_fd = 0;
struct sockaddr_in servAddr;
bool	serverReady = false;
bool	start_visualize = false;

//*****************************************************************
//   함수.
//*****************************************************************
// 통신 포트로 데이터를 기다리는 Thread
// DOS에서 수신 Interrupt를 이용했는데, Linux에서는 Thread를 이용하는 듯...
PI_THREAD(serial_t1)
{
	while(1)
	{
		if(serialDataAvail(serial_fd)) // 수신할 데이터가 없으면 즉시 0 Return
		// Returns the number of characters available for reading, 
		// or -1 for any error condition, in which case errno will be set appropriately.
		{
			piLock(KEY_READ); // (V)g_Qlast, (V)buffer[]
			
			buffer[g_Qlast] = serialGetchar(serial_fd);	// 포트에서 데이터를 읽어와 저장
			NEXT_LAST;
			
			piUnlock(KEY_READ);	// g_Qlast, ()buffer[]
		}
	}
}

// Client 입장에서 Server 연결
void Socket_Open(void)
{
    // 소켓 생성, Socket FD 할당
    if((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
	{	printf("Unable to open LAN Socket: %s\n", strerror(errno));	return; }
	else
	{	printf("Ready to connect to Server\n");		}
	
    //Socket 연결 준비 및 연결 시도
    memset(&servAddr,0, sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.s_addr = inet_addr(SERVER_ADDR);
    servAddr.sin_port = htons(SERVER_PORT);

    if ((connect(socket_fd, (struct sockaddr *) &servAddr, sizeof(servAddr))) < 0)
	{	printf("Unable to open LAN Socket: %s\n", strerror(errno));	return;	}
	else
	{	printf("Connected to %s (%d)\n",SERVER_ADDR,SERVER_PORT);		}
	
	serverReady = true;
    return;
}

void Socket_Close(void)
{
	close(socket_fd);
	serverReady = false;
}

#define	G_MAX	1024
#define	G_MIN	-1024
void Send_Init_for_LAN(void)
{
	char Buff_1[1024];
	char sendBuffer[BUFSIZE];
	unsigned int sendLen;
	
	const char* Sensor_Name[9] = {"YAW(Z)","PITCH(Y)","ROLL(Z)",
								"ACC(X)","ACC(Y)","ACC(Z)",
								"GRAVITY(X)","GRAVITY(Y)","GRAVITY(Z)"};
	int	Sensor_Min[9] = {-180,-180,-180,-30000,-30000,-30000,G_MIN,G_MIN,G_MIN};
	int Sensor_Max[9] = {180,180,180,30000,30000,30000,G_MAX,G_MAX,G_MAX};
	int channel;
	
	channel = 9;	
	sprintf(sendBuffer, "init %d ",channel);
	
	for (int i=0;i<channel;i++)
	{
		sprintf(Buff_1, "%s %d %d ", Sensor_Name[i], Sensor_Min[i], Sensor_Max[i]);
		strcat(sendBuffer,Buff_1);
	}
	strcat(sendBuffer,"\n");
	printf("%s\r\n",sendBuffer);
	      
    sendLen =  write(socket_fd, sendBuffer, strlen(sendBuffer));
	if (sendLen != strlen(sendBuffer))
	{	printf("Sending Error!!!\n");	}
	else
	{
		delay(500);
		start_visualize = true;
	}
}

// Hex String을 화면으로 출력.
void PrintHexStr(unsigned char* bpBuffer, int bCount)
{
	for (int i=0;i<bCount;i++)	printf("[%02X]",bpBuffer[i]);
	printf("\r\n");
	return;
}

// Hex String을 TxQueue로 전송
// Tx Queue에서는 Timer Interrupt에서 Packet을 전송한다.
void SendHexStr_to_TxQueue(unsigned char* bpBuffer)
{
	int total_length;
	int ch;

	// Queue가 가득 차 있는동안 대기
	// Timer Interrupt가 멈추지 않는 한 무한루프에 빠질 염려는 없다.
	do
	{
		if (kbhit())
		{
			ch = getch();
			switch(ch)
			{
				case 's' : g_Send_Flag ^= 1;  break;
				case 'c' : g_Emer_Stop = 1;		break;
			}
		}
		// delay(50);
		// 여기서 50ms delay가 있다면... 계산이 안되었을텐데... 어떻게 된거지?
		// 일단 지워보자.
	}while(IsTxQueueFull);  // 여기서 Lock을 걸면, Timer에서 데이터를 꺼내지를 못한다.
  
	total_length = min(bpBuffer[3]+4,BUFFER_LENGTH);
	// Tx_Queue에 넣는다.
	piLock(KEY_WRITE);
	for (int i=0;i<total_length;i++)
	{
		g_Tx_Packet[g_Tx_Qlast][i] = bpBuffer[i];
	}

	NEXT_TX_LAST;
	piUnlock(KEY_WRITE);
}

void Sync_Write_ALL(int position_ax_LEG[2][6],int speed_ax_LEG[2][6],
					int position_ax_ARM[2][4],int speed_ax_ARM[2][4] )
{
	int i;
	unsigned char TxData[BUFFER_LENGTH];

	TxData[0] = TxData[1] = 0xFF;
	TxData[2] = BROADCASTING_ID;
	TxData[3] = 99;			// Length
	TxData[4] = INST_SYNC_WRITE;
	TxData[5] = P_GOAL_POSITION_L;
	TxData[6] = 4;
	for (i=0;i<6;i++)
	{
		TxData[7+i*5] = LEG_ID[RIGHT_LEG][i];
		TxData[8+i*5] = position_ax_LEG[RIGHT_LEG][i] & 0xFF;
		TxData[9+i*5] = (position_ax_LEG[RIGHT_LEG][i]>>8) & 0xFF;
		TxData[10+i*5] = speed_ax_LEG[RIGHT_LEG][i] & 0xFF;
		TxData[11+i*5] = (speed_ax_LEG[RIGHT_LEG][i]>>8) & 0xFF;
	}
	for (i=0;i<6;i++)
	{
		TxData[37+i*5] = LEG_ID[LEFT_LEG][i];
		TxData[38+i*5] = position_ax_LEG[LEFT_LEG][i] & 0xFF;
		TxData[39+i*5] = (position_ax_LEG[LEFT_LEG][i]>>8) & 0xFF;
		TxData[40+i*5] = speed_ax_LEG[LEFT_LEG][i] & 0xFF;
		TxData[41+i*5] = (speed_ax_LEG[LEFT_LEG][i]>>8) & 0xFF;
	}

	// 허리
	TxData[67] = WAIST_ID;
	TxData[68] = position_ax_ARM[RIGHT][0] & 0xFF;
	TxData[69] = (position_ax_ARM[RIGHT][0]>>8) & 0xFF;
	TxData[70] = speed_ax_ARM[RIGHT][0] & 0xFF;
	TxData[71] = (speed_ax_ARM[RIGHT][0]>>8) & 0xFF;

	// 오른팔
	for (i=1;i<4;i++)
	{
		TxData[67+i*5] = ARM_ID[RIGHT][i];
		TxData[68+i*5] = position_ax_ARM[RIGHT][i] & 0xFF;
		TxData[69+i*5] = (position_ax_ARM[RIGHT][i]>>8) & 0xFF;
		TxData[70+i*5] = speed_ax_ARM[RIGHT][i] & 0xFF;
		TxData[71+i*5] = (speed_ax_ARM[RIGHT][i]>>8) & 0xFF;
	}
	// 왼팔
	for (i=1;i<4;i++)
	{
		TxData[82+i*5] = ARM_ID[LEFT][i];
		TxData[83+i*5] = position_ax_ARM[LEFT][i] & 0xFF;
		TxData[84+i*5] = (position_ax_ARM[LEFT][i]>>8) & 0xFF;
		TxData[85+i*5] = speed_ax_ARM[LEFT][i] & 0xFF;
		TxData[86+i*5] = (speed_ax_ARM[LEFT][i]>>8) & 0xFF;
	}

	TxData[102] = CalCheckSum(TxData,102);

	// Data 송신.
	SendHexStr_to_TxQueue(TxData);		// 0 ~ i까지 데이터 송신
}

void Sync_Write_LEG(int position_ax[2][6],int speed_ax[2][6])
{
	int i;
	unsigned char TxData[BUFFER_LENGTH];

	TxData[0] = TxData[1] = 0xFF;
	TxData[2] = BROADCASTING_ID;
	TxData[3] = 64;			// Length
	TxData[4] = INST_SYNC_WRITE;
	TxData[5] = P_GOAL_POSITION_L;
	TxData[6] = 4;
	for (i=0;i<6;i++)
	{
		TxData[7+i*5] = LEG_ID[RIGHT_LEG][i];
		TxData[8+i*5] = position_ax[RIGHT_LEG][i] & 0xFF;
		TxData[9+i*5] = (position_ax[RIGHT_LEG][i]>>8) & 0xFF;
		TxData[10+i*5] = speed_ax[RIGHT_LEG][i] & 0xFF;
		TxData[11+i*5] = (speed_ax[RIGHT_LEG][i]>>8) & 0xFF;
	}
	for (i=0;i<6;i++)
	{
		TxData[37+i*5] = LEG_ID[LEFT_LEG][i];
		TxData[38+i*5] = position_ax[LEFT_LEG][i] & 0xFF;
		TxData[39+i*5] = (position_ax[LEFT_LEG][i]>>8) & 0xFF;
		TxData[40+i*5] = speed_ax[LEFT_LEG][i] & 0xFF;
		TxData[41+i*5] = (speed_ax[LEFT_LEG][i]>>8) & 0xFF;
	}
	TxData[67] = CalCheckSum(TxData,67);

	// Data 송신.
	SendHexStr_to_TxQueue(TxData);		// 0 ~ i까지 데이터 송신
}

void Sync_Write_BODY(int position_ax[2][4],int speed_ax[2][4])
{
	int i;
	unsigned char TxData[BUFFER_LENGTH];

	TxData[0] = TxData[1] = 0xFF;
	TxData[2] = BROADCASTING_ID;
	TxData[3] = 39;			// Length
	TxData[4] = INST_SYNC_WRITE;
	TxData[5] = P_GOAL_POSITION_L;
	TxData[6] = 4;

	// 허리
	TxData[ 7] = WAIST_ID;
	TxData[ 8] = position_ax[RIGHT][0] & 0xFF;
	TxData[ 9] = (position_ax[RIGHT][0]>>8) & 0xFF;
	TxData[10] = speed_ax[RIGHT][0] & 0xFF;
	TxData[11] = (speed_ax[RIGHT][0]>>8) & 0xFF;

	// 오른팔
	for (i=1;i<4;i++)
	{
		TxData[ 7+i*5] = ARM_ID[RIGHT][i];
		TxData[ 8+i*5] = position_ax[RIGHT][i] & 0xFF;
		TxData[ 9+i*5] = (position_ax[RIGHT][i]>>8) & 0xFF;
		TxData[10+i*5] = speed_ax[RIGHT][i] & 0xFF;
		TxData[11+i*5] = (speed_ax[RIGHT][i]>>8) & 0xFF;
	}
	// 왼팔
	for (i=1;i<4;i++)
	{
		TxData[22+i*5] = ARM_ID[LEFT][i];
		TxData[23+i*5] = position_ax[LEFT][i] & 0xFF;
		TxData[24+i*5] = (position_ax[LEFT][i]>>8) & 0xFF;
		TxData[25+i*5] = speed_ax[LEFT][i] & 0xFF;
		TxData[26+i*5] = (speed_ax[LEFT][i]>>8) & 0xFF;
	}
	TxData[42] = CalCheckSum(TxData,42);

	// Data 송신.
	SendHexStr_to_TxQueue(TxData);		// 0 ~ i까지 데이터 송신
}

// TxData의 Packet의 Checksum을 계산.
// Count는 Checksum의 위치임.
unsigned char CalCheckSum(unsigned char* TxData, int Count)
{
	unsigned char CheckSum;
	int j;

	// Check Sum의 계산
	CheckSum = 0;
	for(j=2;j<Count;j++)
	{
		CheckSum = CheckSum + TxData[j];
	}
	return ~CheckSum;
}

// return  : 받은 데이터 개수 (Error의 경우 0을 Return함)
// pRxData : 받은 데이터
int RX_data_check(unsigned char* pRxData)
{
	int i, Rx_Len, Packet_Length, Result;
	unsigned char CheckSum;

	Result = 0;			// Data 없음.
	pRxData[2] = 0;		// ID를 0으로 변경.(사용하지 않는 ID)

	piLock(KEY_READ); // g_Qlast, ()buffer[]
	if (!IsReadyToRX)	// g_Qlast == g_Qfirst
	{
		printf("No data in RX buffer \r\n");
	}
	else
	{
		do
		{
			// [FF] [FF] [ID] [LEN] [ERR] [PARM...] [CHECKSUM]
			if (g_Qlast < g_Qfirst) Rx_Len = g_Qlast+BUFFER_LENGTH - g_Qfirst;
			else 					Rx_Len = g_Qlast - g_Qfirst;

			#define MIN_PACKET  6
			//---------------------------------
			// Packet 최소길이 계산.
			//---------------------------------
			if (Rx_Len >= MIN_PACKET)
			// [정상] 수신 데이터가 Packet의 최소길이보다 긴 경우
			{
				//---------------------------------
				// Header 확인.
				//---------------------------------
				if ((buffer[g_Qfirst] == 0xFF) && (buffer[g_Qfirst+1] == 0xFF))
				// [정상] Header가 0xFF, 0xFF인 경우
				{
					// Packet Length 계산.
					Packet_Length = buffer[g_Qfirst + 3];

					//---------------------------------
					// Packet Length 확인
					//---------------------------------
					if (Rx_Len >= (Packet_Length + 4))
					// [정상] 수신 데이터가 Packet의 총길이보다 긴경우
					{
						// CheckSum의 계산.
						CheckSum = 0;
						for(i=2;i<(Packet_Length + 3);i++)
						{
							CheckSum = CheckSum + buffer[g_Qfirst + i];
						}
						CheckSum = ~CheckSum;

						//---------------------------------
						// Check Sum 확인.
						//---------------------------------
						if (buffer[g_Qfirst + Packet_Length + 3] == CheckSum)
						// [정상]
						{
							// Packet 출력.
							for (i=0;i<(Packet_Length + 4);i++)
							{
								pRxData[i] = buffer[g_Qfirst+i];	// 버퍼의 값을 반환.
							}
							Result = i;
							// g_Qfirst를 다음 Packet을 읽을 곳으로 이동.
							g_Qfirst = g_Qfirst + Packet_Length + 4;
							break;
						}
						else
						// [이상]
						{
							// printf("CheckSum Error");
							// g_Qfirst를 하나 이동시켜서 다시 검사.
							NEXT_FIRST;
						}
					}
					else
					// [이상] Length를 고려할때 Length가 짧은 경우.
					{
						// printf("RX is not complete(2) \r\n");
						break;
					}
				}
				// [이상] 정상 헤더가 아닌 경우
				else
				{
					// g_Qfirst를 하나 이동시켜서 다시 검사.
					// printf("Header Error \r\n");
					NEXT_FIRST;
				}
			}
			// [이상] 수신 데이터가 Packet의 최소길이보다 긴 경우
			else
			{
				// printf("RX is not complete(1), first = %d, last = %d \r\n",g_Qfirst,g_Qlast);
				break;
			}
		}while(TRUE);
	}
	piUnlock(KEY_READ); // g_Qlast, ()buffer[]  
	return Result;
}

// 수신한 RX Data를 모두 화면에 출력
void Dump_RX_data()
{
	piLock(KEY_READ); // g_Qlast, ()buffer[]
	while (IsReadyToRX)	// g_Qlast == g_Qfirst
	{
		printf("[%02X]",buffer[g_Qfirst++]);
	}
	// printf("\r\n");
	piUnlock(KEY_READ); // g_Qlast, ()buffer[]  
	return;
}

// AX-12의 Register에 pTable의 Data를 Length만큼 넣음.
// Length는 pTable의 인자 개수임.
void WriteHex(int ID, unsigned char Addr, unsigned char * pTable, int Length)
{
	int i, Result;
	unsigned char pTxData[BUFFER_LENGTH];
	unsigned char pRxData[BUFFER_LENGTH];

	pTxData[0] = pTxData[1] = 0xFF;
	pTxData[2] = ID;
	pTxData[3] = Length + 3;
	pTxData[4] = INST_WRITE;
	pTxData[5] = Addr;
	// Parameter
	for (i=0;i<Length;i++)
	{
		pTxData[i+6] = pTable[i];
	}
	// Check Sum 계산.
	pTxData[Length + 6] = CalCheckSum(pTxData,Length + 6);

	// Data 송신.
	if (ID == BROADCASTING_ID)
	{
		SendHexStr_to_TxQueue(pTxData);		// 0 ~ i까지 데이터 송신
		printf("\r\n");
	}
	else
	{
		SET_RTS;					// 송신 모드로 변경
		write(serial_fd,pTxData, Length+7);
		RESET_RTS;					// 수신 모드로 변경
		printf("Send : ");	PrintHexStr(pTxData,Length+7);	printf("\r\n");
		
		delay(5);					// 10ms delay
		// 데이터 수신 후 기록
		Result = RX_data_check(pRxData);
		printf("Receive(%d) : ",Result); PrintHexStr(pRxData,Result); printf("\r\n");
	}

	return;
}

void ReadHex(int ID, unsigned char Addr, unsigned char * pTable, int Length)
{
	int i, Result;
	unsigned char pTxData[BUFFER_LENGTH];
	unsigned char pRxData[BUFFER_LENGTH];

	pTxData[0] = pTxData[1] = 0xFF;
	pTxData[2] = ID;
	pTxData[3] = Length + 3;
	pTxData[4] = INST_READ;
	pTxData[5] = Addr;
	// Parameter
	for (i=0;i<Length;i++)
	{
		pTxData[i+6] = pTable[i];
	}
	// Check Sum 계산.
	pTxData[Length + 6] = CalCheckSum(pTxData,Length + 6);

	// Data 송신.
	if (ID == BROADCASTING_ID)
	{
		SendHexStr_to_TxQueue(pTxData);		// 0 ~ i까지 데이터 송신
		printf("\r\n");
	}
	else
	{
		SET_RTS;					// 송신 모드로 변경
		write(serial_fd,pTxData, Length+7);
		RESET_RTS;					// 수신 모드로 변경
		printf("Send : ");	PrintHexStr(pTxData,Length+7);	printf("\r\n");
		
		delay(5);					// 10ms delay
		// 데이터 수신 후 기록
		Result = RX_data_check(pRxData);
		printf("Receive(%d) : ",Result); PrintHexStr(pRxData,Result);	printf("\r\n");
		printf("Error Code : [%02X]\r\n", pRxData[4]);
		printf("Rcv Data   : "); 		PrintHexStr(&pRxData[5],Result-6); 	printf("\r\n");
	}

	return;
}

void reset_RTS(void)
{
	int txemptystate;
	while(1)
	{
		ioctl(serial_fd,TIOCSERGETLSR, &txemptystate);
		if (txemptystate) break;
	}
	// tcdrain(serial_fd);	// 이건 안된다.
	digitalWrite(RTS,HIGH);
}
  
