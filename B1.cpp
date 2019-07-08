////////////////////////////////////////////////////////////////////
// 작성일 : 051231
// 제  목 : 타이머 인터럽트
// 내  용 : A_MOVE_1.cpp를 수정함.
////////////////////////////////////////////////////////////////////
#define null 0x00

#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#include <signal.h>
#include <stdio.h>
#include <stdint.h>			// uint8_t....
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "define.h"
#include "kinhead.h"
#include "serial.h"
#include "timer.h"
#include "debug.h"
#include "main.h"
// #include "MPU6050_6Axis_MotionApps20.h"
// 선언 순서에 주의!!!
#include "I2Cdev.h"
#include "helper_3dmath.h"

#define MPU6050_INCLUDE_DMP_MOTIONAPPS20
#include "MPU6050.h"

//////////////////////////////////////////////////
// Serial.CPP
//////////////////////////////////////////////////
// 데이터 저장을 위한 버퍼와 버퍼 정보 변수,
// g_Qfirst != g_Qlast이면 Read_From_Port를 호출한다.
extern unsigned char g_Qfirst;			// g_Qfirst : 지금 데이터를 읽어야 할 지점을 가리킴.
extern unsigned char g_Qlast;			// g_Qlast : 다음 입력할 버퍼의위치를 가리킴.
extern int serial_fd;
extern int socket_fd;
extern struct sockaddr_in servAddr;
extern bool serverReady;
extern bool	start_visualize;

//////////////////////////////////////////////////
// Timer.CPP
//////////////////////////////////////////////////
volatile extern unsigned int g_Send_Flag;
//////////////////////////////////////////////////
// Kin_M.CPP
//////////////////////////////////////////////////
extern int g_LEG_Position[2][6];	// AX-12의 위치로 저장.
extern int g_ARM_Position[2][4];	// AX-12의 위치로 저장.

//////////////////////////////////////////////////
// B1.CPP
//////////////////////////////////////////////////
int g_t1x_diff, g_t2x_diff;
int g_t1y_diff, g_t2y_diff;
int g_t1z_diff, g_t2z_diff;
int g_TE_diff;

int	g_Emer_Stop;
int select_speed;

double g_z_RIGHT_BASE, g_z_LEFT_BASE;

const int    LEG_ID[][6]     = {{   8,  10,  12,  14,  16,  18}, {   9,  11,  13,  15,  17,  19}};

//////////////////////////////////////////////////
// B1.CPP
//////////////////////////////////////////////////

extern MPU6050 mpu;

// MPU control/status vars
extern bool dmpReady;  // set true if DMP init was successful
extern uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
extern uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
extern uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
extern uint16_t fifoCount;     // count of all bytes currently in FIFO
extern uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
extern Quaternion q;           // [w, x, y, z]         quaternion container
extern VectorInt16 aa;         // [x, y, z]            accel sensor measurements
extern VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
extern VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
extern VectorFloat gravity;    // [x, y, z]            gravity vector
extern float euler[3];         // [psi, theta, phi]    Euler angle container
extern float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
extern uint8_t teapotPacket[14];

int main(void)
{
	char OneLineBuff[80];	// 명령 라인 버퍼
	char PreOneLineBuff[80];	// 명령 라인 버퍼_이전값

	setbuf(stdout, NULL); 	// 이게 없으면 \n을 만나기 전에는 출력하지 않는다!

	//////////////////////
	// GPIO 사용 Setup
	//////////////////////
	if (wiringPiSetup() == -1)
	{
		printf("Unable to start wiringPi: %s\n", strerror(errno));
		return 1;
	}
	printf("WiringPi start. \r\n");

	pinMode(RTS,OUTPUT);
	SET_RTS;  // 기본 송신모드
	
	//////////////////////
	// Serial 포트 열기
	// DATABIT8,STOPBIT1,PARITYNONE,BPS115200 (모터제어용)
	//////////////////////
	if ((serial_fd = serialOpen(PORT,BAUD_RATE)) < 0)
	{
		printf("Unable to open serial device: %s\n", strerror(errno));
		return 1;
	}
	printf("Serial Port Open \r\n");

	//////////////////////
	// Timer Interrupt 선언 (Raspberry Pi)
	//////////////////////
	signal(SIGALRM, timer_handler);
	SetTimer();
	
	//////////////////////
	// MPU-6050 초기화
	//////////////////////
	// initialize device
    printf("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    printf("Testing device connections...\n");
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // load and configure the DMP
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
    }

	//////////////////////
	// Thread 시작
	//////////////////////
	piThreadCreate(serial_t1);
	if (dmpReady == true) piThreadCreate(sensor_t1);
	// 이상하게... while(dmpReady)는 while(1)과 같은 역할을 하지 못한다.
	// 그래서, Thread안에서는 while(1)로 바꿨다.
	
	// 변수 초기화
	g_t1x_diff = g_t2x_diff = 0;
	g_t1y_diff = g_t2y_diff = 0;
	g_t1z_diff = g_t2z_diff = 0;
	g_TE_diff = 0;

	// g_z_RIGHT_BASE = -40;
	// g_z_LEFT_BASE  =  40;
	
	g_z_RIGHT_BASE = -50;
	g_z_LEFT_BASE  = 50;
	
	g_Send_Flag = 1;
	g_Emer_Stop = 0;

	get_g_Position();
	printf("Reading Current Position!!!\n");
	// 초기 프롬프트
    printf(">");

	// Main Routine
	while (1)
	{
		fgets(OneLineBuff,80,stdin);
		
		// for short cut...
		if (strncmp(OneLineBuff,"x",1) == 0) 	
			break;
		// 걷기 데모
		else if (strncmp(OneLineBuff,"dw",2) == 0)
			strcpy(OneLineBuff,"mw 50 120 30 20 -50 40");
		// 토크 끄기
		else if (strncmp(OneLineBuff,"t",1) == 0)
			strcpy(OneLineBuff,"wt a 0");
		// 이전 명령 반복
		else if (strncmp(OneLineBuff,"'",1) == 0)
		{
			strcpy(OneLineBuff,PreOneLineBuff);
			printf("%s\r\n",OneLineBuff);
		}
		
		// Blank가 아니면!!! 이전 명령 기억
		if (strlen(OneLineBuff) > 1)	strcpy(PreOneLineBuff,OneLineBuff);
		
		MonitorRoutine(OneLineBuff);
	}

	// serial_close(serial_fd);  // Serial Port Close 필요
	// Timer_Interrupt_Close(); // Timer 정지가 필요할까?
	// close(socket_fd);			// 이건 꼭 필요할 것 같은데...
	return 1;
}

//-----------------------------------------------------------------------------
//	Monitor Routinue
void MonitorRoutine(char* MonCmd)
{
	int Result;
    char *token[MAX_TOK];
	int ID, ADDR, param[MAX_TOK];
	int ArgLen = 0;
	unsigned char pRxData[BUFFER_LENGTH], pTable[BUFFER_LENGTH];

	// 입력 코멘드 분리 ex) mw 1 2 3 -> ArgLen = 4
    token[ArgLen] = strtok(MonCmd, " ");  // extracting the first token
    while (token[ArgLen] != NULL) {
        // printf("token(%d) = %s\n", ArgLen, token[ArgLen]);
        token[++ArgLen] = strtok(NULL, " ");	// 두번째 이후
    }

	// 명령 확인, 첫번째 명령
	switch (token[0][0])
	{
		case 'l':
			if (token[0][1] == 'o')	
			{	
				if (serverReady == false)	Socket_Open();
				else printf("Server was Opened already.\r\n");
				break;
			}
			if (token[0][1] == 'c')	
			{	
				if (serverReady == true)	Socket_Close();
				else printf("Server was Closed already.\r\n");
				break;
			}
			if (token[0][1] == 'i')	
			{	
				if (serverReady == true)	Send_Init_for_LAN();
				else printf("Server was Closed already.\r\n");
				break;
			}
			goto DispUsage;
		case 'd':
			if (token[0][1] == 'd')		{ Dump_RX_data(); printf("\r\n");	break; }
			goto DispUsage;
			
		case 'c':
			if (token[0][1] == 'p')		{ Result = RX_data_check(pRxData); PrintHexStr(pRxData, Result); break; }
			if (token[0][1] == 'c') 	{ Check_Mtx(token,ArgLen);			break; }	// 다리만 있을때...
			if (token[0][1] == 't') 	{ Check_All_Torque(token,ArgLen);	break; }
			goto DispUsage;
		case 's':
			if (token[0][1] == 's')		{ SET_RTS;  	break; }
			if (token[0][1] == 'r')		{ RESET_RTS;	break; }
			/*
			////////////////////////////////////////////////////////////////////////////////
			if (token[0][1] == '1')		// ID 1을 18번으로 변경
			{	ID = 1;		pTable[0] = 18;		WriteHex(ID, P_ID, pTable, 1);	break;	}
			if (token[0][1] == '2')		// ID를 18번을 1로 변경
			{	ID = 18;		pTable[0] = 1;		WriteHex(ID, P_ID, pTable, 1);	break;	}
			////////////////////////////////////////////////////////////////////////////////
			*/
			goto DispUsage;
		case 'w':
			Dump_RX_data();
			if (token[0][1] == 't')
			{
				// wt [ID][0|1]
				if (ArgLen < 3)	{printf("Not enough parameter. \r\n");	break; }
				ID = atoi(token[1]);
				if (ID == 0)	ID = BROADCASTING_ID;
				pTable[0] = (atoi(token[2]) == 0) ? 0 : 1 ;
				WriteHex(ID, P_TORQUE_ENABLE, pTable, 1);
				break;
			}

		 	if (token[0][1] == 'l')
			{
				// wl [ID][0|1]
				if (ArgLen < 3)	{printf("Not enough parameter. \r\n");	break; }
				ID = atoi(token[1]);
				if (ID == 0)	ID = BROADCASTING_ID;

				pTable[0] = (atoi(token[2]) == 0) ? 0 : 1 ;
				WriteHex(ID, P_LED, pTable, 1);
				break;
			}
			goto DispUsage;
		case 'm':
			if (token[0][1] == 'v') {	Move_Position(token,ArgLen);	break;   }
			if (token[0][1] == 'f') {	Move_For_Kin(token, ArgLen);   	break;   }
			// if (token[0][1] == 'f') {	Move_For_Kin_BODY(token, ArgLen);  	break;   }
			if (token[0][1] == 'i') {	Move_Inv_Kin(token, ArgLen);   	break;   }
			// if (token[0][1] == 'c') {	Check_Kin(token,ArgLen);    	break;   }
			if (token[0][1] == 'c') {	Check_Kin_Body(token,ArgLen);    	break;   }
			if (token[0][1] == 'z') {	Move_Zero();	Move_Zero_BODY();	break;   }
			if (token[0][1] == 's') {	Move_Sit_Down();				break;   }
			if (token[0][1] == 'b') {	Move_Zero_BODY();		    	break;   }
			if (token[0][1] == 'l') {	Move_Both_LEG(token,ArgLen);	break;   }
			if (token[0][1] == 'w') {	Move_COG_Walking(token,ArgLen);	break;   }
			goto DispUsage;
		case 'r':
			Dump_RX_data();
			if (token[0][1] == 'p') {	Get_Position(token, ArgLen);	break;   }
			if (token[0][1] == 'c') {	Get_COG_Data();	break;   }	// 다리만...
			if (token[0][1] == 'h')
			{ 
				if (ArgLen < 4)	{printf("Not enough parameter. \r\n");	break; }
				ID = atoi(token[1]);
				ADDR = atoi(token[2]);
				param[0] = atoi(token[3]);
				ArgLen = 4;
				if ((ID < 0x00) || (ID > 0xFF))	{printf("Out of range : ID = 0 ~ 255 \r\n");	break;}
				if ((ADDR < 0) || (ADDR > 49))	{printf("Out of range : Addr = 0 ~ 49 \r\n");	break;}
				if (param[0] > 50) 				{printf("Out of range \r\n");	break;}
				ReadHex(ID, ADDR, (unsigned char*)param, ArgLen);
				break; 
			}
			goto DispUsage;
		case 'h':
			printf(	"[Monitor Command List]\r\n\r\n"
					"[--- Short Cut ---------------------------------------------------------]\r\n"
					"x   -> Exit\r\n"	// 종료
					"t   -> Disable all Torque \r\n"	// 모든 Tarque 정비
					"dw  -> Demo Walking \r\n"			// Walking Demo
					"[--- Check Command -----------------------------------------------------]\r\n"
					"cc  -> Check COG (cc [X pos][Y pos][Z pos][LIFT(?)])\r\n"
					"cp  -> Check receive Packet \r\n"
					"ct  -> Check COG by Reading Currnet Position (ct) \r\n"
					"[--- Write Command -----------------------------------------------------]\r\n"
					"wt  -> Write Torque Enable to AX-12 register (wt [ID][0|1]) \r\n"
					"wl  -> Write Led status to AX-12 register (wl [ID][0|1]) \r\n"
					"[--- Read Command ------------------------------------------------------]\r\n"
					"rp  -> Read current Position (rp [ID])	\r\n"
					"rc  -> Read COG (rc)	\r\n"
					"rh  -> Read Control Table using Address (ra [ID][ADDR][BYTE to READ])	\r\n"
					"[--- Move Command ------------------------------------------------------]\r\n"
					"mv  -> MoVe AX-12 to POSITION (mv [ID][POSITION][SPEED])  \r\n"
					"mf  -> Move Forward kinematics (mf [J1][J2][J3][J4][J5][J6]) \r\n"
					"mz  -> Move to Zero(Stand Up) Position (mz)	\r\n"
					"ms  -> Move to Zero(Sit Down) Position (ms)	\r\n"
					"mi  -> Move Inverse kinematics (mi [X pos][Y pos][Z pos]) \r\n"
					"mc  -> Moving Check (Forward->Inverse) (mc [J2][J3][J4]) \r\n"
					"ml  -> Moving both LEG (ml [P_Rx][P_Ry][P_Rz][P_Lx][P_Ly][P_Lz]) \r\n"
					"mw  -> Moving by COG control (mw [t1][t2][xh][yf][yr][zp(zn)])\r\n"
					"[--- Others -----------------------------------------------------]\r\n"
					"lo  -> LAN socket Open \r\n"	// 192.168.0.3 / 5560 고정(define.h)
					"lc  -> LAN socket Close \r\n"
					"li  -> LAN communication Init \r\n"
					"dd  -> Dump receive Data \r\n"
					"\r\n");
			break;
		default:
			DispUsage:
			break;
	}
	printf(">");
}

void Get_Position(char* pArgument[MAX_TOK], int ArgLen)
{
	int ID, Result, Position;
	unsigned char TxData[BUFFER_LENGTH];
	unsigned char pRxData[BUFFER_LENGTH];
	
	int index, direction;
	float theta;
	int ARM_position[4], LEG_position[6];
	double ARM_theta[4], LEG_theta[6];

	// rp [ID]
	if (ArgLen < 2)	{printf("Not enough parameter. \r\n");	return; }
	ID = atoi(pArgument[1]);
	if ((ID < 0x00) || (ID > 0xFF))	{printf("Out of range : ID = 0 ~ 255 \r\n");	return;}

	// Make Packet
	// [FF][FF][ID][LEN][CMD][30][P_L][P_H][S_L][S_H][CHECKSUM]
	// Header
	TxData[0] = TxData[1] = 0xFF;
	TxData[2] = ID;
	TxData[3] = 4;
	TxData[4] = INST_READ;
	TxData[5] = P_PRESENT_POSITION_L;
	TxData[6] = 2;
	TxData[7] = CalCheckSum(TxData,7);
	
	printf("\n\n-----------------\nData Sending\n-----------------\n\n");
	// Data 송신.
	SET_RTS;					// 송신 모드로 변경
	write(serial_fd,TxData, 8);
	RESET_RTS;					// 수신 모드로 변경
	printf("Send Data\n");
	PrintHexStr(TxData,8);
	printf("\r\n");
	delay(5);					// 1ms delay
	// 데이터 수신
	printf("\n\n-----------------\nData Receiving\n-----------------\n\n");
	Result = RX_data_check(pRxData);
	
	printf("Receiving Data\n");
	PrintHexStr(pRxData,Result);
	printf("\r\n");
	Dump_RX_data();
	
	Position = pRxData[5]+(pRxData[6]<<8);

	// ARM_ID[][4]     = {{   1,   2,   5,   7}, {   1,  3,  4,  6}};
	if (ID <= 3)
	{
		index = ID / 2;
		direction = ID % 2;
		
		ARM_position[index] = Position;  
		ARM_P2D(ARM_position, ARM_theta, direction);
		theta = ARM_theta[index];
	}
	else if (ID <= 7)
	{
		index = ID / 2;
		direction = (ID - 1) % 2;
		
		ARM_position[index] = Position;  
		ARM_P2D(ARM_position, ARM_theta, direction);
		theta = ARM_theta[index];
	} 
	// LEG_ID[][6]     = {{   8,  10,  12,  14,  16,  18}, {   9,  11,  13,  15,  17,  19}};
	else
	{
		index = (ID - 8) / 2;
		direction = ID % 2;
		
		LEG_position[index] = Position;  
		LEG_P2D(LEG_position, LEG_theta, direction);
		theta = LEG_theta[index];
	}

	if (Result != 0)
		printf("Position = %4d, degree = %6.1f (%d) \r\n",Position, theta, Result);
	else
		printf("Try 'dd' command.\r\n");
		
	return;
}


// 다리
// 정기구학에 의한 위치로 이동(각도)
void Move_For_Kin(char* pArgument[MAX_TOK],int ArgLen)
{
	int i;
	int speed_ax[2][6], position_ax[2][6];
	double theta[2][6];		// degree
	double mtx[3][4];
	double abgxyz[6];
	double position[2][4][3];

	////////////////////////////////////
	// 각 관절의 목표 각도(deg) 구함.
	// 정기구학(?)
	////////////////////////////////////
	// theta 1 ... 6
	// mf [J1][J2][J3][J4][J5][J6]
	if (ArgLen < 7)	{printf("Not enough parameter. \r\n");	return; }
	// 관절각
	for (i=0;i<6;i++)	theta[RIGHT][i] = theta[LEFT][i] = atof(pArgument[i+1]);

	// 회전각.
	printf("Angle = ");
	for (i=0;i<6;i++)	printf("[%9.4f]",theta[RIGHT][i]);
	printf("\r\n");

	// 정기구학.
	T06_for_M2(theta[RIGHT], mtx);

	// ItoU
	ItouXyz(mtx, abgxyz);
	printf("U xyz = ");
	for (i=0;i<3;i++)	printf("[%9.4f]",abgxyz[i]*RADTODEG);
	for (i=3;i<6;i++)	printf("[%9.4f]",abgxyz[i]);
	printf("\r\n");

	// 오른쪽 다리
	// 위치로 변환.
	LEG_D2P(theta[RIGHT], position_ax[RIGHT_LEG], RIGHT_LEG);

	// AX-12의 위치
	printf("R_LEG = ");
	for (i=0;i<6;i++)	printf("[%9d]",position_ax[RIGHT_LEG][i]);
	printf("\r\n");

	// 왼쪽 다리
	// 위치로 변환.
	LEG_D2P(theta[LEFT], position_ax[LEFT_LEG], LEFT_LEG);

	// AX-12의 위치
	printf("L_LEG = ");
	for (i=0;i<6;i++)	printf("[%9d]",position_ax[LEFT_LEG][i]);
	printf("\r\n");

	// 발끝의 BC를 기준으로 한 X,Y,Z 좌표 계산
	Cal_LEG_Position(theta, position);
	printf("RF : %6.1f, %6.1f, %6.1f\r\n",
		position[RIGHT_LEG][P_BC_F][PX],position[RIGHT_LEG][P_BC_F][PY],position[RIGHT_LEG][P_BC_F][PZ]);
	printf("LF : %6.1f, %6.1f, %6.1f\r\n",
		position[LEFT_LEG][P_BC_F][PX] ,position[LEFT_LEG][P_BC_F][PY], position[LEFT_LEG][P_BC_F][PZ]);

	// 확인.
	printf("Press 'c' to cancel. Press other key to start\r\n");
	while(kbhit());
	if (getch() != 'c')
	{
		// Make & Send Packet
		Cal_LEG_Speed(position_ax, speed_ax, NORMAL);
		Sync_Write_LEG(position_ax, speed_ax);
		printf("\r\n");
	}
    return;
}

// 안기
// 정기구학에 의한 위치로 이동(각도)
void Move_Sit_Down(void)
{
	int i;
	int speed_ax[2][6], position_ax[2][6];
	int speed_ax_ARM[2][4], position_ax_ARM[2][4];
	// 실험에 의한 값(하체)
	// {8,10,12,14,16,18},{9,11,13,15,17,19}
	double theta[2][6] = {{0,0,-60,120,-60,0},{0,0,-60,120,-60,0}};		// degree
	// 실험에 의한 값(상체)
	// {1,2,5,7},{1,3,4,6}
	double theta_ARM[2][4] = {{0,0,-20,20},{0,0,20,-20}};;		// degree
	double mtx[3][4];
	double abgxyz[6];
	double position[2][4][3];

	////////////////////////////////////
	// 각 관절의 목표 각도(deg) 구함.
	// 하체
	////////////////////////////////////
	// 정기구학.
	T06_for_M2(theta[RIGHT], mtx);

	// ItoU
	ItouXyz(mtx, abgxyz);
	printf("U xyz = ");
	for (i=0;i<3;i++)	printf("[%9.4f]",abgxyz[i]*RADTODEG);
	for (i=3;i<6;i++)	printf("[%9.4f]",abgxyz[i]);
	printf("\r\n");

	// 오른쪽 다리
	// 위치로 변환.
	LEG_D2P(theta[RIGHT], position_ax[RIGHT_LEG], RIGHT_LEG);

	// AX-12의 위치
	printf("R_LEG = ");
	for (i=0;i<6;i++)	printf("[%9d]",position_ax[RIGHT_LEG][i]);
	printf("\r\n");

	// 왼쪽 다리
	// 위치로 변환.
	LEG_D2P(theta[LEFT], position_ax[LEFT_LEG], LEFT_LEG);

	// AX-12의 위치
	printf("L_LEG = ");
	for (i=0;i<6;i++)	printf("[%9d]",position_ax[LEFT_LEG][i]);
	printf("\r\n");

	// 발끝의 BC를 기준으로 한 X,Y,Z 좌표 계산
	Cal_LEG_Position(theta, position);
	printf("RF : %6.1f, %6.1f, %6.1f\r\n",
		position[RIGHT_LEG][P_BC_F][PX],position[RIGHT_LEG][P_BC_F][PY],position[RIGHT_LEG][P_BC_F][PZ]);
	printf("LF : %6.1f, %6.1f, %6.1f\r\n",
		position[LEFT_LEG][P_BC_F][PX] ,position[LEFT_LEG][P_BC_F][PY], position[LEFT_LEG][P_BC_F][PZ]);

	////////////////////////////////////
	// 각 관절의 목표 각도(deg) 구함.
	// 상체
	////////////////////////////////////
	
	// 위치로 변환.
	ARM_D2P(theta_ARM[RIGHT], position_ax_ARM[RIGHT], RIGHT);

	// 위치로 변환.
	ARM_D2P(theta_ARM[LEFT], position_ax_ARM[LEFT], LEFT);
	
	// Make & Send Packet
	Cal_LEG_Speed(position_ax, speed_ax, NORMAL);
	Cal_ARM_Speed(position_ax_ARM, speed_ax_ARM, NORMAL);
	
	Sync_Write_LEG(position_ax, speed_ax);
	Sync_Write_BODY(position_ax_ARM, speed_ax_ARM);
	printf("\r\n");

    return;
}

// 상체
// 정기구학에 의한 위치로 이동
void Move_For_Kin_BODY(char* pArgument[MAX_TOK],int ArgLen)
{
	int i;
	int speed_ax[2][4], position_ax[2][4];
	double theta[4];		// degree
	double mtx[3][4];
	double abgxyz[6];

	////////////////////////////////////
	// 각 관절의 목표 각도(deg) 구함.
	// (허리, 오른쪽팔)
	// 정기구학(?)
	////////////////////////////////////

	// mf (오른쪽)[J1][J2][J3][J4](왼쪽)[J2][J3][J4]
	if (ArgLen < 8)	{printf("Not enough parameter. \r\n");	return; }
	// 관절각, theta 1 ... 4
	for (i=0;i<4;i++)	theta[i] = atof(pArgument[i+1]);

	// 회전각.
	printf("Angle = ");
	for (i=0;i<4;i++)	printf("[%9.4f]",theta[i]);
	printf("\r\n");

	// 정기구학.
	T05_for_M2_BODY(theta, mtx);

	// ItoU
	ItouXyz(mtx, abgxyz);
	printf("U xyz = ");
	for (i=0;i<3;i++)	printf("[%9.4f]",abgxyz[i]*RADTODEG);
	for (i=3;i<6;i++)	printf("[%9.4f]",abgxyz[i]);
	printf("\r\n");

	// 허리와 오른쪽 팔
	// 위치로 변환.
	ARM_D2P(theta, position_ax[RIGHT], RIGHT);

	// AX-12의 위치
	printf("R_ARM = ");
	for (i=0;i<4;i++)	printf("[%9d]",position_ax[RIGHT][i]);
	printf("\r\n");

	////////////////////////////////////
	// 각 관절의 목표 각도(deg) 구함.
	// (왼쪽팔), -> 허리(theta[0])는 오른팔의 경우와 동일함.
	// 정기구학(?)
	////////////////////////////////////
	// 관절각, theta 2 ... 4
	for (i=0;i<3;i++)	theta[i+1] = atof(pArgument[i+5]);

	// 회전각.
	printf("Angle = ");
	for (i=0;i<4;i++)	printf("[%9.4f]",theta[i]);
	printf("\r\n");

	// 정기구학.
	T05_for_M2_BODY(theta, mtx);

	// ItoU
	ItouXyz(mtx, abgxyz);
	printf("U xyz = ");
	for (i=0;i<3;i++)	printf("[%9.4f]",abgxyz[i]*RADTODEG);
	for (i=3;i<6;i++)	printf("[%9.4f]",abgxyz[i]);
	printf("\r\n");

	// 왼쪽 팔
	// 위치로 변환.
	ARM_D2P(theta, position_ax[LEFT], LEFT);

	// AX-12의 위치
	printf("L_ARM = ");
	for (i=0;i<4;i++)	printf("[%9d]",position_ax[LEFT][i]);
	printf("\r\n");

	// 확인.
	printf("Press 'c' to cancel. Press other key to start\r\n");
	while(kbhit());
	if (getch() != 'c')
	{
		// Make & Send Packet

		// JHK 동작!!!!!!!!!!
		Cal_ARM_Speed(position_ax, speed_ax, NORMAL);
		Sync_Write_BODY(position_ax, speed_ax);
		printf("\r\n");
	}
    return;
}

// 19년에 왼발만 움직이도록 코드를 다시짰다.
// 06년 작성한 코드는 Move_Inv_Kin_temp이다.
void Move_Inv_Kin(char* pArgument[MAX_TOK],int ArgLen)
{
	int i;
	unsigned int state_to_run;
	double theta[2][6];		// degree
	double mtx[3][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0}};

	int speed_ax[2][6], position_ax[2][6];
	int sol_cnt, state[MAX_SOL];
	double jt_deg[MAX_SOL][6];
	
	double T06[3][4];

	////////////////////////////////////
	// X,Y,Z 축의 목표 위치를 구함.
	// 역기구학
	////////////////////////////////////
	// mi [X pos][Y pos][Z pos]
	if (ArgLen < 4)	{printf("Not enough parameter. \r\n");	return; }

	// 참고) mtx는 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)만 사용함.
	// 오른발의 위치
	for (i=0;i<3;i++)	mtx[i][3] = atof(pArgument[i+1]);
	
	// 이후 코드는 Move_COG_Working()에서 따왔다.
	// t=0에서의 초기값(최초 걸음을 시작할때)
	/*
	step = LEFT_LEG;		// 지탱하는 발.
	x[RIGHT_LEG] = X_BASE_POS;			x[LEFT_LEG] = X_BASE_POS;
	y[RIGHT_LEG] = yr;					y[LEFT_LEG] = yf;
	z[RIGHT_LEG] = zp + g_z_RIGHT_BASE;	z[LEFT_LEG] = zp + g_z_LEFT_BASE;
	*/
	Get_R_C_F(mtx);
	/*
	mtx[X][3] = x[k];
	mtx[Y][3] = y[k];
	mtx[Z][3] = z[k];
	*/
	// {BC}{F}T -> {0}{6}T로 변경.
	Cal_LEG_BASE(mtx, T06, RIGHT_LEG);

	// 역기구학의 계산.
	sol_cnt = Inverse_Kin_M2(T06, jt_deg, state);
	if (sol_cnt != 0)
	{
		// state_to_run = 최적해(경험상... 별 무리가 없더라...)
		state_to_run = 0;
		if ((state_to_run < MAX_SOL) && (state[state_to_run] == 1))
		{
			for (i=0;i<6;i++)	theta[RIGHT_LEG][i] = jt_deg[state_to_run][i];
			LEG_D2P(theta[RIGHT_LEG], position_ax[RIGHT_LEG], RIGHT_LEG);
		}
		else
		{
			printf("No Initial Solution\r\n");
			return;
		}
		// 왼발의 위치 (고정값 이용)
		position_ax[LEFT_LEG][0] = 511;
		position_ax[LEFT_LEG][1] = 511;
		position_ax[LEFT_LEG][2] = 474;
		position_ax[LEFT_LEG][3] = 745;
		position_ax[LEFT_LEG][4] = 548;
		position_ax[LEFT_LEG][5] = 511;
	
		// Make & Send Packet
		Cal_LEG_Speed(position_ax, speed_ax, NORMAL);
		Sync_Write_LEG(position_ax, speed_ax);
	}
	else
	{
		printf("No Solution\r\n");
	}
}

// 06년에 이렇게 만들었는데, 어떤 의미인지 모르겠다.
// 좌우 발 중 하나를 선택해서 움직이도록 코드를 다시짰다.
// 이건... 의미를 몰라서 일단 _temp를 붙여서 호출하지 않도록 막아놨다.
void Move_Inv_Kin_temp(char* pArgument[MAX_TOK],int ArgLen)
{
	char str[2];
	int i;
	unsigned int state_to_run;
	double theta[6];		// degree
	double mtx[3][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0}};

	int speed_ax[2][6], position_ax[2][6];
	int sol_cnt, state[MAX_SOL];
	double jt_deg[MAX_SOL][6];

	////////////////////////////////////
	// X,Y,Z 축의 목표 위치를 구함.
	// 역기구학
	////////////////////////////////////
	// mi [X pos][Y pos][Z pos]
	if (ArgLen < 4)	{printf("Not enough parameter. \r\n");	return; }

	// 참고) mtx는 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)만 사용함.
	for (i=0;i<3;i++)	mtx[i][3] = atof(pArgument[i+1]);

	// 역기구학의 계산.
	sol_cnt = Inverse_Kin_M2(mtx, jt_deg, state);

	// 계산된 역기구학의 확인.
	if (sol_cnt != 0)
	{
		printf("INV_T06 \r\n");
		for (i=0;i<MAX_SOL;i++)
		{
			if (state[i] == 1)
			{
				printf("%d : %9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf\r\n",
			           i,jt_deg[i][0],jt_deg[i][1],jt_deg[i][2],jt_deg[i][3],jt_deg[i][4],jt_deg[i][5]);
			}
		}
		// 확인.
		printf("Select Inv_Kin solution Number '0 ~ 3' \r\n");
		while(kbhit());
		str[0] = getch();
		str[1] = null;
		state_to_run = atoi(str);

		if ((state_to_run < MAX_SOL) && (state[state_to_run] == 1))
		{
			for (i=0;i<6;i++)	theta[i] = jt_deg[state_to_run][i];

			// 오른쪽 다리
			// 위치로 변환.
			LEG_D2P(theta, position_ax[RIGHT_LEG], RIGHT_LEG);

			// AX-12의 위치
			printf("R_LEG = ");
			for (i=0;i<6;i++)	printf("[%9d]",position_ax[RIGHT_LEG][i]);
			printf("\r\n");

			// 왼쪽 다리
			// 위치로 변환.
			LEG_D2P(theta, position_ax[LEFT_LEG], LEFT_LEG);

			// AX-12의 위치
			printf("L_LEG = ");
			for (i=0;i<6;i++)	printf("[%9d]",position_ax[LEFT_LEG][i]);
			printf("\r\n");

			// 확인.
			printf("Press 'c' to cancel. Press other key to start\r\n");
			while(kbhit());
			if (getch() != 'c')
			{
				// Make & Send Packet
				Cal_LEG_Speed(position_ax, speed_ax, NORMAL);
				Sync_Write_LEG(position_ax, speed_ax);
			}
		}
	}
	return;
}

// 현재 위치에서 전체의 무게중심을 확인
void Check_All_Torque(char* pArgument[MAX_TOK],int ArgLen)
{
	int i,j;
	double theta[2][6], arm_theta[2][4];		// degree
	int LEG_position[6], ARM_position[4];
	double Pcog[3], Torque[3];
	
	// 현재 Position을 읽음
	get_g_Position();
	
	// 다리의 Position을 각도로 변환 (RIGHT = 0, LEFT = 1)
	for(i=0;i<2;i++)
	{
		for(j=0;j<6;j++)	{	LEG_position[j] = g_LEG_Position[i][j];	}
		LEG_P2D(LEG_position, theta[i], i);
	}
	
	
	// 팔과 허리의 Position을 각도로 변환 (RIGHT = 0, LEFT = 1)
	for(i=0;i<2;i++)
	{
		for(j=0;j<4;j++)	{	ARM_position[j] = g_ARM_Position[i][j];	}
		ARM_P2D(ARM_position, arm_theta[i], i);
	}

	// 무게중심 확인.
	// 팔과 다리 위치 출력
	printf("RIGHT_LEG_Posit: ");
	for(i=0;i<6;i++)	printf(" %6d,", g_LEG_Position[RIGHT][i]);
	
	printf("\r\nLEFT_LEG_Posit:  ");
	for(i=0;i<6;i++)	printf(" %6d,", g_LEG_Position[LEFT][i]);
	
	printf("\r\nRIGHT_ARM_Posit: ");
	for(i=0;i<4;i++)	printf(" %6d,", g_ARM_Position[RIGHT][i]);
	
	printf("\r\nLEFT_ARM_Posit:  ");
	for(i=0;i<4;i++)	printf(" %6d,", g_ARM_Position[LEFT][i]);
	
	printf("\r\n");

	printf("\r\nRIGHT_LEG_Theta: ");
	for(i=0;i<6;i++)	printf(" %6.1f,", theta[RIGHT][i]);
	
	printf("\r\nLEFT_LEG_Theta:  ");
	for(i=0;i<6;i++)	printf(" %6.1f,", theta[LEFT][i]);

	printf("\r\nRIGHT_ARM_Theta: ");
	for(i=0;i<4;i++)	printf(" %6.1f,", arm_theta[RIGHT][i]);
	
	printf("\r\nLEFT_ARM_Theta:  ");
	for(i=0;i<4;i++)	printf(" %6.1f,", arm_theta[LEFT][i]);
	
	printf("\r\n");
	
	Cal_Torque_ALL(arm_theta, theta, Torque, Pcog, RIGHT_LEG);
	printf("\r\nCOG:              %6.1f, %6.1f, %6.1f \r\n",Pcog[0], Pcog[1], Pcog[2]);

	return;
}

// 몸통의 무게중심을 확인
void Check_BODY_Torque(char* pArgument[MAX_TOK],int ArgLen)
{
	int i;
	double theta_waist, arm_theta[2][4];		// degree
	double Pcog[3], Torque[3];

	////////////////////////////////////
	// X,Y,Z 축의 목표 위치를 구함.
	////////////////////////////////////

	/*****************************************/
	// 몸통과 양팔
	/*****************************************/

	// 허리의 각도
	// ct [허리](오른팔)[X][Y][Z](왼팔)[X][Y][Z], 8개
	if (ArgLen < 8) {printf("Not enough parameter. \r\n"); 	return;}

	arm_theta[RIGHT][0] = arm_theta[LEFT][0] = theta_waist = atof(pArgument[1]);

	// 오른쪽 팔의 각도
	for (i=0;i<3;i++)	arm_theta[RIGHT][i+1] = atof(pArgument[i+2]);

	// 왼쪽 팔의 각도
	for (i=0;i<3;i++)	arm_theta[LEFT][i] = atof(pArgument[i+5]);

	// 무게중심 확인.
	Cal_Torque_BODY(arm_theta, Torque, Pcog);
	printf("COG : %6.1f, %6.1f, %6.1f \r\n",Pcog[0], Pcog[1], Pcog[2]);

	return;
}

// 양쪽 발을 이동
void Move_Both_LEG(char* pArgument[MAX_TOK],int ArgLen)
{
	int i;
	unsigned int state_to_run;
	double theta[2][6];		// degree
	double mtx[3][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0}};
	double T06[3][4];
	double position[2][4][3];

	int speed_ax[2][6], position_ax[2][6];
	int sol_cnt, state[MAX_SOL];
	double jt_deg[MAX_SOL][6];

	double Pcog[3], Torque[3];

	////////////////////////////////////
	// X,Y,Z 축의 목표 위치를 구함.
	// 역기구학
	////////////////////////////////////
	/*****************************************/
	// 우측발
	/*****************************************/
	// 참고) mtx는 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)만 사용함.
	//       그러나... T06을 계산하기 위해 RBF가 필요.
	//       RBF는 항상 고정되어 있음.
	// 방향 : 중심에서 발끝의 회전 Matrix
	Get_R_C_F(mtx);

	// (ml [P_Rx][P_Ry][P_Rz][P_Lx][P_Ly][P_Lz])
	if (ArgLen < 7)	{printf("Not enough parameter. \r\n");	return; }

	// 위치
	// 참고) mtx는 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)만 사용함.
	for (i=0;i<3;i++)	mtx[i][3] = atof(pArgument[i+1]);

	// {BC}{F}T -> {0}{6}T로 변경.
	Cal_LEG_BASE(mtx, T06, RIGHT_LEG);

	// 역기구학의 계산.
	sol_cnt = Inverse_Kin_M2(T06, jt_deg, state);

	// 계산된 역기구학의 확인.
	if (sol_cnt != 0)
	{
		printf("INV_T06_RIGHT \r\n");
		for (i=0;i<MAX_SOL;i++)
		{
			if (state[i] == 1)
			{
				printf("%d : %9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf \r\n",
			           i,jt_deg[i][0],jt_deg[i][1],jt_deg[i][2],jt_deg[i][3],jt_deg[i][4],jt_deg[i][5]);
			}
		}
		// 확인.
		// printf("Run AX-12 for RIGHT_LEG, press '0 ~ 3' \r\n");
		// while(kbhit());
		// str[0] = getch();
		// str[1] = NULL;
		// state_to_run = atoi(str);
		state_to_run = 0;
		if ((state_to_run < MAX_SOL) && (state[state_to_run] == 1))
		{
			for (i=0;i<6;i++)	theta[RIGHT_LEG][i] = jt_deg[state_to_run][i];

			// 오른쪽 다리		// 위치로 변환.
			LEG_D2P(theta[RIGHT_LEG], position_ax[RIGHT_LEG], RIGHT_LEG);
		}
	}
	else
	{
		printf("Right LEG does not have solution \r\n");
		goto Exit;
	}

	/*****************************************/
	// 좌측발
	/*****************************************/
	// 참고) mtx는 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)만 사용함.
	//       그러나... T06을 계산하기 위해 RBF가 필요.
	//       RBF는 항상 고정되어 있음.

	// 방향 : 중심에서 발끝의 회전 Matrix
	Get_R_C_F(mtx);

	// 위치
	for (i=0;i<3;i++)	mtx[i][3] = atof(pArgument[i+4]);

	// {BC}{F}T -> {0}{6}T로 변경.
	Cal_LEG_BASE(mtx, T06, LEFT_LEG);

	// 역기구학의 계산.
	sol_cnt = Inverse_Kin_M2(T06, jt_deg, state);

	// 계산된 역기구학의 확인.
	if (sol_cnt != 0)
	{
		printf("INV_T06_LEFT \r\n");
		for (i=0;i<MAX_SOL;i++)
		{
			if (state[i] == 1)
			{
				printf("%d : %9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf \r\n",
			           i,jt_deg[i][0],jt_deg[i][1],jt_deg[i][2],jt_deg[i][3],jt_deg[i][4],jt_deg[i][5]);
			}
		}
		// 확인.
		// printf("Run AX-12 for LEFT LEG, press '0 ~ 3' \r\n");
		// while(kbhit());
		// str[0] = getch();
		// str[1] = NULL;a
		// state_to_run = atoi(str);
		state_to_run = 0;
		if ((state_to_run < MAX_SOL) && (state[state_to_run] == 1))
		{
			for (i=0;i<6;i++)	theta[LEFT_LEG][i] = jt_deg[state_to_run][i];

			// 왼쪽 다리	// 위치로 변환.
			LEG_D2P(theta[LEFT_LEG], position_ax[LEFT_LEG], LEFT_LEG);
		}
	}
	else
	{
		printf("Left LEG does not have solution \r\n");
		goto Exit;
	}
	
	// 위치에서 속도를 계산
	Cal_LEG_Speed(position_ax, speed_ax, NORMAL);
	
	// 좌우 관절의 위치를 position으로 출력
	printf("R_LEG[8...18]%4d, %4d, %4d, %4d, %4d, %4d\r\n",
				position_ax[RIGHT_LEG][0], position_ax[RIGHT_LEG][1], position_ax[RIGHT_LEG][2],
				position_ax[RIGHT_LEG][3], position_ax[RIGHT_LEG][4], position_ax[RIGHT_LEG][5]);
	printf("L_LEG[9...19]%4d, %4d, %4d, %4d, %4d, %4d\r\n",
				position_ax[LEFT_LEG][0], position_ax[LEFT_LEG][1], position_ax[LEFT_LEG][2],
				position_ax[LEFT_LEG][3], position_ax[LEFT_LEG][4], position_ax[LEFT_LEG][5]);
	/*
	printf("%4d, %4d, %4d, %4d, %4d, %4d\r\n",
				speed_ax[RIGHT_LEG][0], speed_ax[RIGHT_LEG][1], speed_ax[RIGHT_LEG][2],
				speed_ax[RIGHT_LEG][3], speed_ax[RIGHT_LEG][4], speed_ax[RIGHT_LEG][5]);
	*/
		
	// 무게중심 확인.
	Cal_Torque_LEG(theta, Torque, Pcog, LEFT_LEG);
	printf("COG : %6.1f, %6.1f, %6.1f \r\n",Pcog[0], Pcog[1], Pcog[2]);

	// 다리의 각 좌표계의 현재 위치 확인
	Cal_LEG_Position(theta, position);
/*
	printf("R0 : %6.1f, %6.1f, %6.1f\r\n",
		position[RIGHT_LEG][P_BC_0][PX],position[RIGHT_LEG][P_BC_0][PY],position[RIGHT_LEG][P_BC_0][PZ]);
	printf("L0 : %6.1f, %6.1f, %6.1f\r\n",
		position[LEFT_LEG][P_BC_0][PX] ,position[LEFT_LEG][P_BC_0][PY], position[LEFT_LEG][P_BC_0][PZ]);

	printf("R4 : %6.1f, %6.1f, %6.1f\r\n",
		position[RIGHT_LEG][P_BC_4][PX],position[RIGHT_LEG][P_BC_4][PY],position[RIGHT_LEG][P_BC_4][PZ]);
	printf("L4 : %6.1f, %6.1f, %6.1f\r\n",
		position[LEFT_LEG][P_BC_4][PX] ,position[LEFT_LEG][P_BC_4][PY], position[LEFT_LEG][P_BC_4][PZ]);

	printf("R6 : %6.1f, %6.1f, %6.1f\r\n",
		position[RIGHT_LEG][P_BC_6][PX],position[RIGHT_LEG][P_BC_6][PY],position[RIGHT_LEG][P_BC_6][PZ]);
	printf("L6 : %6.1f, %6.1f, %6.1f\r\n",
		position[LEFT_LEG][P_BC_6][PX] ,position[LEFT_LEG][P_BC_6][PY], position[LEFT_LEG][P_BC_6][PZ]);
*/
	printf("RF : %6.1f, %6.1f, %6.1f\r\n",
		position[RIGHT_LEG][P_BC_F][PX],position[RIGHT_LEG][P_BC_F][PY],position[RIGHT_LEG][P_BC_F][PZ]);
	printf("LF : %6.1f, %6.1f, %6.1f\r\n",
		position[LEFT_LEG][P_BC_F][PX] ,position[LEFT_LEG][P_BC_F][PY], position[LEFT_LEG][P_BC_F][PZ]);
	
	// 확인.
	printf("Press 'c' to cancel. Press other key to start\r\n");
	while(kbhit());
	// Make & Send Packet
	if (getch() != 'c')	Sync_Write_LEG(position_ax, speed_ax);

Exit:
	return;
}


#define REPETITION_COUNT	4
#define Z_COMP_VALUE 		0

// COG 제어에 의한 걸음.
void Move_COG_Walking(char* pArgument[MAX_TOK],int ArgLen)
{
	int i, j, k;
	unsigned int state_to_run;
	double theta[2][6];		// degree
	double theta_ARM[2][4];		// degree
	double mtx[3][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0}};
	double T06[3][4];

	int t, t1, t2, t1x, t2x, t1xm, t1y, t2y, t1z, TE, count;
	double xh, yf, yr, zp, zn;
	double x_Up_Speed, x_Dn_Speed, y_Speed, z_Speed;
	double x[2], y[2], z[2];
	double x_comp[2], y_comp[2], z_comp[2];

	int Arg[7];
	int step;

	int speed_ax[2][6], position_ax[2][6];
	int speed_ax_ARM[2][4], position_ax_ARM[2][4];
	int sol_cnt, state[MAX_SOL];
	double jt_deg[MAX_SOL][6];
	double best_sol[6];

	double Pcog[3], Torque[3];
	int	support_leg, swing_leg;

	char file_name[100];
	FILE *fp_out;
	char temp;

	x_comp[RIGHT_LEG] = x_comp[LEFT_LEG] = 0;
	y_comp[RIGHT_LEG] = y_comp[LEFT_LEG] = 0;
	z_comp[RIGHT_LEG] = z_comp[LEFT_LEG] = 0;

	//////////////////////////////
	// 인자 추출
	/////////////////////////////
	// mw [t1][t2][xh][yf][yr][zp(zn)]
	if (ArgLen < 7)	{printf("Not enough parameter. \r\n");	return; }

	// COG Data Log 기록 여부
	printf("Do you want to log the COG data? [y/n]");
	while (kbhit());
	temp = getch();
	if (temp == 'y' || temp == 'Y') 
	{
		printf("\r\nEnter a file name : ");
		fgets(file_name, sizeof(file_name),stdin);
		if (strlen(file_name) != 1)	// filename 입력이 된 경우
		{
			file_name[strlen(file_name)-1]='\0';
		}
		else	// filename 입력이 되지 않음 경우 (enter만 입력이 된 경우)
		{
			struct timeval ts;
			gettimeofday(&ts, NULL);
			sprintf(file_name,"COG_%ld.dat",ts.tv_sec);	// 시스템시간을 이용해 파일명 생성
		}
		printf("Filename to write the COG data: %s\n",file_name);
		
		// 파일 열기
		if((fp_out = fopen(file_name,"wt")) == NULL)
			printf("Can not open file %s\n",file_name);
		temp = 'y';
	}
	printf("\r\n");
	
	// t1, t2, xh, yf, yr, zp(zn), 0 ... 5
	for (i=0;i<6;i++)	Arg[i] = atof(pArgument[i+1]);
	// count, 6(Default 는 4회)
	if (ArgLen == 7) Arg[6] = REPETITION_COUNT;
	else Arg[6] = atof(pArgument[7]);

	// 발을 들기 시작하는 시간(t1), 완전히 내린 시간(t2)
	// 시간의 단위는 Cycle이다.
	t1 = (int)Arg[0];
	t2 = (int)Arg[1];
	
	// "g_xxx_diff"는 나중에 보상을 하려고 만든 것 같다. ('06년)
	// 그런데, 지금 다시보니 무슨 생각이었는지 기억이 안난다. ('19년)
	t1x = t1 + g_t1x_diff;
	t2x = t2 + g_t2x_diff;
	t1xm = (t1x+t2x)/2;

	t1y = t1 + g_t1y_diff;
	t2y = t2 + g_t2y_diff;

	t1z = t1 + g_t1z_diff;

	TE = t2 + g_TE_diff;

	// 발을 들어올리는 높이와 속도
	// xh: 들어올리는 높이, x_Up/Dn_Speed: 올리고 내리는 속도 (한 cycle동안 이만큼씩 증감한다)
	xh = Arg[2];
	x_Up_Speed = xh / (t1xm - t1x);
	x_Dn_Speed = xh / (t2x - t1xm);
	
	// 발을 전진하는 폭(yf)과 뒤로 미는 폭(yr)
	yf = Arg[3];
	yr = Arg[4];
	y_Speed = (yf - yr) / (t2y - t1y);

	// 좌우로 무게중심을 이동하는 거리
	// t = 0 ~ t1동안 이동한다.
	zp = Arg[5];
	zn = -Arg[5];
	z_Speed = (zp - zn) / (t1z);

	// 반복횟수
	count = Arg[6];
	/////////////////////
	// 상태 기록
	/////////////////////
	if (temp == 'y')
	{
		fprintf(fp_out,"------------------------------------------------------------------------------------------------------------\r\n");
		fprintf(fp_out,"--- Controlled by Center of Gravity ------------------------------------------------------------------------\r\n");
		fprintf(fp_out,"------------------------------------------------------------------------------------------------------------\r\n");
		fprintf(fp_out,"t1  = %4d, t2   = %4d\r\n",t1,t2);
		fprintf(fp_out,"t1x = %4d, t1xm = %4d, t2x = %4d\r\n",t1x,t1xm,t2x);
		fprintf(fp_out,"t1y = %4d, t2y  = %4d\r\n",t1y,t2y);
		fprintf(fp_out,"t1z = %4d\r\n",t1z);
		fprintf(fp_out,"------------------------------------------------------------------------------------------------------------\r\n");
		fprintf(fp_out,"xh  = %5.1f\r\n",xh);
		fprintf(fp_out,"yf  = %5.1f, yr = %5.1f\r\n",yf,yr);
		fprintf(fp_out,"zp  = %5.1f, zn = %5.1f\r\n",zp,zn);
		fprintf(fp_out,"------------------------------------------------------------------------------------------------------------\r\n");
		fprintf(fp_out,"x_Up_Speed = %5.1f, x_Dn_Speed = %5.1f\r\n",x_Up_Speed, x_Dn_Speed);
		fprintf(fp_out,"y_Speed    = %5.1f\r\n",y_Speed);
		fprintf(fp_out,"z_Speed    = %5.1f\r\n",z_Speed);
		fprintf(fp_out,"\r\n");
		fprintf(fp_out,"------------------------------------------------------------------------------------------------------------\r\n");
	  	fprintf(fp_out,"  tn,R_X_Pos,R_Y_Pos,R_Z_Pos,L_X_Pos,L_Y_Pos,L_Z_Pos,  COG_X,  COG_Y,  COG_Z, Torque_X, Torque_Y, Torque_Z \r\n");
		fprintf(fp_out,"------------------------------------------------------------------------------------------------------------\r\n");
	}
	//////////////////////////////
	// 초기 위치로 이동.
	//////////////////////////////
	// t=0에서의 초기값(최초 걸음을 시작할때)
	step = LEFT_LEG;		// 지탱하는 발.
	x[RIGHT_LEG] = X_BASE_POS;			x[LEFT_LEG] = X_BASE_POS;
	y[RIGHT_LEG] = yr;					y[LEFT_LEG] = yf;
	z[RIGHT_LEG] = zp + g_z_RIGHT_BASE;	z[LEFT_LEG] = zp + g_z_LEFT_BASE;
	// <<<<<<<< 하체 >>>>>>>>>
	// k = RIGHT_LEG(0), LEFT_LEG(1)
	for (k=0;k<2;k++)
	{
		Get_R_C_F(mtx);
		mtx[X][3] = x[k];
		mtx[Y][3] = y[k];
		mtx[Z][3] = z[k];

		// {BC}{F}T -> {0}{6}T로 변경.
		Cal_LEG_BASE(mtx, T06, k);

		// 역기구학의 계산.
		sol_cnt = Inverse_Kin_M2(T06, jt_deg, state);

		// state_to_run = 최적해(경험상... 별 무리가 없더라...)
		state_to_run = 0;
		if ((state_to_run < MAX_SOL) && (state[state_to_run] == 1))
		{
			for (i=0;i<6;i++)	theta[k][i] = jt_deg[state_to_run][i];
			LEG_D2P(theta[k], position_ax[k], k);
		}
		else
		{
			printf("No Initial Solution for %s\r\n",((k==RIGHT_LEG)?"RIGHT_LEG":"LEFT_LEG"));
			goto Exit;
		}

		// <<<보정>>> 경험상, 무릅 관절(14,15번)의 힘이 약해서 목적한 값보다 10정도 낮아진다.
		// 엉덩이가 처짐. 10정도는 올려줘야 정상적으로 걷는다.
		position_ax[RIGHT][3] += 10;
		position_ax[ LEFT][3] += 10;
	}

	// Make & Send Packet
	Cal_LEG_Speed(position_ax, speed_ax, NORMAL);
	Sync_Write_LEG(position_ax, speed_ax);

	// <<<<<<<< 상체 >>>>>>>>>
	// 허리와 오른쪽 팔
	// <<<보정>>> 경험상, 무게중심이 맞지 않는 것 같아 허리를 약간 앞으로 숙였다.
	theta_ARM[RIGHT][0] = 10;
	theta_ARM[RIGHT][1] = 0;
	theta_ARM[RIGHT][2] = -20;
	theta_ARM[RIGHT][3] = 20;
	
	// 위치로 변환.
	ARM_D2P(theta_ARM[RIGHT], position_ax_ARM[RIGHT], RIGHT);

	// 왼쪽 팔
	theta_ARM[LEFT][0] = 0;		// 허리와 같은 ID이나, 허리는 RIGHT ARM의 값을 이용한다.
	theta_ARM[LEFT][1] = 0;
	theta_ARM[LEFT][2] = 20;
	theta_ARM[LEFT][3] = -20;

	// 위치로 변환.
	ARM_D2P(theta_ARM[LEFT], position_ax_ARM[LEFT], LEFT);

	// Make & Send Packet
	Cal_ARM_Speed(position_ax_ARM, speed_ax_ARM, NORMAL);
	Sync_Write_BODY(position_ax_ARM, speed_ax_ARM);

	///////////////////////////
	// 이동 시작 확인.
	///////////////////////////
	printf("Press 'c' to cancel. Press other key to start\r\n");
	while(kbhit());
	if (getch() == 'c') goto Exit;

	//////////////////////////////
	// t에 따른 제어(??msec마다 제어)
	//////////////////////////////
	for (j = 0;j<count;j++)
	{
		printf("Supporting Leg(RIGHT = 0, LEFT = 1) = %d\r\n",step);
		
		for (t=0;t<TE;t++)
		{
			if (g_Emer_Stop)	goto Exit;
		//////////////////////////////
		// t에 따른 이동 위치 계산
		//////////////////////////////

			if (step == LEFT_LEG) 	{	support_leg = LEFT_LEG;		swing_leg = RIGHT_LEG;	}
			else					{	support_leg = RIGHT_LEG;	swing_leg = LEFT_LEG;	}

			
			
			// X축의 이동
			if (t<t1x)			{ x[swing_leg] = X_BASE_POS;	x[support_leg] = X_BASE_POS;}	// 두발 지지
			else if (t<t1xm)	{ x[swing_leg] += x_Up_Speed; 	x[support_leg] = X_BASE_POS;}	// 한발 지지, 들어올리는 중
			else if (t<t2x)		{ x[swing_leg] -= x_Dn_Speed; 	x[support_leg] = X_BASE_POS;}	// 한발 지지, 내리는 중
			else 				{ x[swing_leg] = X_BASE_POS;	x[support_leg] = X_BASE_POS;} 	// 두발 지지

			// Y축 방향의 이동
			if (t<t1y)			{ y[swing_leg] = yr;			y[support_leg] = yf; 		}	// 두발지지	
			else if (t<t2y)		{ y[swing_leg] += y_Speed;		y[support_leg] -= y_Speed; }	// 한발지지
			else				{ y[swing_leg] = yf;			y[support_leg] = yr; 		}	// 두발지지
			
			// Z축 방향의 이동
			if (step == LEFT_LEG)
			{
				if (t<t1z)	{ z[RIGHT_LEG] -= z_Speed;				z[LEFT_LEG] -= z_Speed; }
				else 		{ z[RIGHT_LEG] = zn + g_z_RIGHT_BASE;	z[LEFT_LEG] = zn + g_z_LEFT_BASE; }
			}
			else
			{
				if (t<t1z)	{ z[RIGHT_LEG] += z_Speed;				z[LEFT_LEG] += z_Speed; }
				else 		{ z[RIGHT_LEG] = zp + g_z_RIGHT_BASE;	z[LEFT_LEG] = zp + g_z_LEFT_BASE; }
			}
			// z값에 g_z_xxx_BASE(40)을 더하는 건 양 발의 간격을 조절하기 위함이다.

		//////////////////////////////
		// 발의 위치에 의한 theta 계산.
		//////////////////////////////
			// k = RIGHT_LEG(0), LEFT_LEG(1)
			for (k=0;k<2;k++)
			{
				Get_R_C_F(mtx);
				mtx[X][3] = x[k] + x_comp[k];
				mtx[Y][3] = y[k] + y_comp[k];
				mtx[Z][3] = z[k] + z_comp[k];

				// {BC}{F}T -> {0}{6}T로 변경.
				Cal_LEG_BASE(mtx, T06, k);

				// 역기구학의 계산.
				sol_cnt = Inverse_Kin_M2(T06, jt_deg, state);

				// 최적해 계산
				if (!sol_cnt)
				{
					printf("No Solution for %s\r\n",((k==RIGHT_LEG)?"RIGHT_LEG":"LEFT_LEG"));
					goto Exit;
				}
				BestSol_M(state, jt_deg, theta[k], best_sol);

				// 현재 위치 기억 (다음번의 best solution을 위해서)
				for (i=0;i<6;i++)	theta[k][i] = best_sol[i];

				// best solution을 이용하여 이동 위치 계산.
				LEG_D2P(best_sol, position_ax[k], k);
			}
			// <<<보정>>> 경험상, 무릅 관절(14,15번)의 힘이 약해서 목적한 값보다 10정도 낮아진다.
			position_ax[RIGHT][3] += 10;	
			position_ax[ LEFT][3] += 10;
			// <<<보정>>> 경험상, 고관절(10,11번)의 힘이 약해서 목적한 값보다 10정도 낮아진다. 
			/*
			if (support_leg == LEFT)	position_ax[LEFT][1] += 50;
			else 						position_ax[RIGHT][1] -= 50;
			*/

		//////////////////////////////////////////////////////////////////
		// 지지하는 발을 축으로 하는 토크 계산및 그에따른 보정.
		//////////////////////////////////////////////////////////////////
			// 발의 좌표계를 기준으로 계산함.
			// Cal_Torque_LEG(theta, Torque, Pcog, step);
			Cal_Torque_ALL(theta_ARM, theta, Torque, Pcog, step);

			// 몸체{BC}의 Z방향 Torque가 0이 되도록 보정함.
			if (Torque[Y] < 0)
			{
				z_comp[RIGHT_LEG] -= Z_COMP_VALUE;
				z_comp[ LEFT_LEG] -= Z_COMP_VALUE;
			}
			else if (Torque[Y] > 0)
			{
				z_comp[RIGHT_LEG] += Z_COMP_VALUE;
				z_comp[ LEFT_LEG] += Z_COMP_VALUE;
			}

		///////////////////////////
		// Make & Send Packet
		///////////////////////////
			Cal_LEG_Speed(position_ax, speed_ax, EXPERIENCE);
			// Sync_Write_LEG(position_ax, speed_ax);
			Sync_Write_ALL(position_ax, speed_ax, position_ax_ARM, speed_ax_ARM);

		///////////////////////////
		// 기록.
		///////////////////////////
			if (j<2 && temp == 'y')
			{
				fprintf(fp_out,"%4d, %6.1f, %6.1f, %6.1f, %6.1f, %6.1f, %6.1f, %6.1f, %6.1f, %6.1f, %8.5f, %8.5f, %8.5f \r\n",
						t,
						x[RIGHT_LEG]+x_comp[RIGHT_LEG], y[RIGHT_LEG]+y_comp[RIGHT_LEG], z[RIGHT_LEG]+z_comp[RIGHT_LEG],
						x[LEFT_LEG]+x_comp[RIGHT_LEG], y[LEFT_LEG]+y_comp[RIGHT_LEG], z[LEFT_LEG]+z_comp[LEFT_LEG],
						Pcog[X], Pcog[Y], Pcog[Z], Torque[X], Torque[Y], Torque[Z]);
			}
		}
		///////////////////////////////
		// 발 바꾸기.
		///////////////////////////////
		if (step == RIGHT_LEG) 	step = LEFT_LEG;
		else					step = RIGHT_LEG;
	}
Exit:
	if(temp == 'y')	fclose(fp_out);

	g_Send_Flag = 1;
	return;
}

// 다리의 기구학 검증.
void Check_Kin(char* pArgument,int ArgLen)
{
	char* pData;
	int Len_Sum,i;
	double theta[6];		// degree
	double mtx[3][4];

	int sol_cnt, state[MAX_SOL];
	double jt_deg[MAX_SOL][6];

	Len_Sum = 0;

	////////////////////////////////////
	// 각 관절의 목표 각도(deg) 구함.
	// 정기구학(?)
	////////////////////////////////////
	// theta 0 ... 6
	theta[0] = 0;
	theta[4] = 0;
        theta[5] = 0;
	for (i=1;i<4;i++)	// 1 ... 3
	{
		if (Len_Sum > ArgLen)
		{
			printf("Not enough parameter. \r\n");
			goto Exit;
		}

		pData = strtok(pArgument," ");
		theta[i] = atof(pData);

		Len_Sum = (int)(Len_Sum + strlen(pData) + (pData-pArgument) + 1);
		pArgument = pData + strlen(pData) + 1;
	}

	// 회전각.
	printf("Angle = ");
	for (i=0;i<6;i++)	printf("[%9.4f]",theta[i]);
	printf("\r\n");

	// 정기구학.
	T06_for_M2(theta, mtx);

	// 말단부 위치
	printf("X pos=[%9.4f],y pos=[%9.4f],z pos=[%9.4f] \r\n",mtx[0][3],mtx[1][3],mtx[2][3]);

	// 방향 성분을 모두 0으로 초기화 함.
	// (역기구학시 위치만을 이용하여 계산함.)
	for (i=0;i<2;i++)
	{
		mtx[i][0] = 0;
		mtx[i][1] = 0;
		mtx[i][2] = 0;
	}

	// 역기구학의 계산.
	sol_cnt = Inverse_Kin_M2(mtx, jt_deg, state);

	// 계산된 역기구학의 확인.
	printf("INV_T06 \r\n");
	for (i=0;i<MAX_SOL;i++)
	{
		if (state[i] == 1)
		{
			printf("%d : %9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf\r\n",
		           i,jt_deg[i][0],jt_deg[i][1],jt_deg[i][2],jt_deg[i][3],jt_deg[i][4],jt_deg[i][5]);
		}
		printf("solution = %d", sol_cnt);	// 유효한 솔루션 갯수
	}
Exit:
	return;
}

// 상체의 기구학 검증.
void Check_Kin_Body(char* pArgument[MAX_TOK],int ArgLen)
{
	int i;
	double theta[4];		// degree
	double mtx[3][4];

	// int sol_cnt, state[MAX_SOL];
	// double jt_deg[MAX_SOL][6];

	////////////////////////////////////
	// 각 관절의 목표 각도(deg) 구함.
	// 정기구학(?)
	////////////////////////////////////
	// mc [J1][J2][J3][J4]
	if (ArgLen < 5)	{printf("Not enough parameter. \r\n");	return; }

	// theta 0 ... 3
	for (i=0;i<4;i++)	theta[i] = atof(pArgument[i+1]);

	// 회전각.
	printf("Angle = ");
	for (i=0;i<4;i++)	printf("[%9.4f]",theta[i]);
	printf("\r\n");

	// 정기구학.
	T02_for_M2_BODY(theta, mtx);
	printf("X pos=[%9.4f],y pos=[%9.4f],z pos=[%9.4f] \r\n",mtx[0][3],mtx[1][3],mtx[2][3]);

	T03_for_M2_BODY(theta, mtx);
	printf("X pos=[%9.4f],y pos=[%9.4f],z pos=[%9.4f] \r\n",mtx[0][3],mtx[1][3],mtx[2][3]);

	T04_for_M2_BODY(theta, mtx);
	printf("X pos=[%9.4f],y pos=[%9.4f],z pos=[%9.4f] \r\n",mtx[0][3],mtx[1][3],mtx[2][3]);

	T05_for_M2_BODY(theta, mtx);
	printf("X pos=[%9.4f],y pos=[%9.4f],z pos=[%9.4f] \r\n",mtx[0][3],mtx[1][3],mtx[2][3]);

	/*
	// 방향 성분을 모두 0으로 초기화 함.
	// (역기구학시 위치만을 이용하여 계산함.)
	for (i=0;i<2;i++)
	{
		mtx[i][0] = 0;
		mtx[i][1] = 0;
		mtx[i][2] = 0;
	}

	// 역기구학의 계산.
	sol_cnt = Inverse_Kin_M2(mtx, jt_deg, state);
	sol_cnt = sol_cnt;

	// 계산된 역기구학의 확인.
	printf("INV_T06 \r\n");
	for (i=0;i<MAX_SOL;i++)
	{
		if (state[i] == 1)
		{
			printf("%d : %9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf,%9.4lf\r\n",
		           i,jt_deg[i][0],jt_deg[i][1],jt_deg[i][2],jt_deg[i][3],jt_deg[i][4],jt_deg[i][5]);
		}
	}
	*/

	return;
}

// Zero 위치로 이동.
void Move_Zero(void)
{
	int i;
	double theta[2][6];		// degree
	int speed_ax[2][6], position_ax[2][6];
	double mtx[3][4];
	double abgxyz[6];
	double position[2][4][3];

	////////////////////////////////////
	// 각 관절의 목표 각도(deg) 구함.
	// 정기구학(?)
	////////////////////////////////////
	// theta 0 ... 6
	for (i=0;i<6;i++)	// 0 ... 5
	{
		theta[RIGHT][i] = 0;
		theta[LEFT][i] = 0;
	}

	// 정기구학.
	T06_for_M2(theta[RIGHT], mtx);

	// ItoU
	ItouXyz(mtx, abgxyz);
	printf("U xyz = ");
	for (i=0;i<3;i++)	printf("[%9.4f]",abgxyz[i]*RADTODEG);
	for (i=3;i<6;i++)	printf("[%9.4f]",abgxyz[i]);
	printf("\r\n");

	// 오른쪽 다리 위치로 변환.
	LEG_D2P(theta[RIGHT], position_ax[RIGHT_LEG], RIGHT_LEG);

	// AX-12의 위치 확인
	printf("R_LEG = ");
	for (i=0;i<6;i++)	printf("[%9d]",position_ax[RIGHT_LEG][i]);
	printf("\r\n");

	// 왼쪽 다리 위치로 변환.
	LEG_D2P(theta[LEFT], position_ax[LEFT_LEG], LEFT_LEG);

	// AX-12의 위치 확인.
	printf("L_LEG = ");
	for (i=0;i<6;i++)	printf("[%9d]",position_ax[LEFT_LEG][i]);
	printf("\r\n");

	// 발끝의 BC를 기준으로 한 X,Y,Z 좌표 계산
	Cal_LEG_Position(theta, position);
	printf("RF : %6.1f, %6.1f, %6.1f\r\n",
		position[RIGHT_LEG][P_BC_F][PX],position[RIGHT_LEG][P_BC_F][PY],position[RIGHT_LEG][P_BC_F][PZ]);
	printf("LF : %6.1f, %6.1f, %6.1f\r\n",
		position[LEFT_LEG][P_BC_F][PX] ,position[LEFT_LEG][P_BC_F][PY], position[LEFT_LEG][P_BC_F][PZ]);

	// Make & Send Packet
	for(i = 0;i<6;i++)
	{
		speed_ax[RIGHT][i] = 100;
		speed_ax[LEFT][i] = 100;
	}
	
	for (i=0;i<6;i++)
	{
		// 최종 위치 저장.
		g_LEG_Position[RIGHT][i] = position_ax[RIGHT][i];
		g_LEG_Position[ LEFT][i] = position_ax[ LEFT][i];
	}
	
	Sync_Write_LEG(position_ax, speed_ax);

	printf("\r\n");
    return;
}

// Zero 위치로 이동.
void Move_Zero_BODY(void)
{
	int i,j;
	double theta[2][4];		// degree
	int speed_ax[2][4], position_ax[2][4];
	double mtx[3][4];
	double abgxyz[4];

	////////////////////////////////////
	// 각 관절의 목표 각도(deg) 구함.
	// 정기구학(?)
	////////////////////////////////////
	// theta 0 ... 3
	for (j=0;j<2;j++)
		for (i=0;i<4;i++) theta[j][i] = 0;
	
	// 정기구학.
	T05_for_M2_BODY(theta[RIGHT], mtx);

	// ItoU
	ItouXyz(mtx, abgxyz);
	printf("U xyz = ");
	for (i=0;i<3;i++)	printf("[%9.4f]",abgxyz[i]*RADTODEG);
	for (i=3;i<6;i++)	printf("[%9.4f]",abgxyz[i]);
	printf("\r\n");

	// 오른쪽 팔 위치로 변환.
	ARM_D2P(theta[RIGHT], position_ax[RIGHT], RIGHT);

	// AX-12의 위치 확인
	printf("R_ARM = ");
	for (i=0;i<6;i++)	printf("[%9d]",position_ax[RIGHT_LEG][i]);
	printf("\r\n");

	// 왼쪽 팔 위치로 변환.
	ARM_D2P(theta[LEFT], position_ax[LEFT], LEFT);

	// AX-12의 위치 확인.
	printf("L_ARM = ");
	for (i=0;i<6;i++)	printf("[%9d]",position_ax[LEFT_LEG][i]);
	printf("\r\n");
	
	// Make & Send Packet
	for(i = 0;i<4;i++)
	{
		speed_ax[RIGHT][i] = 100;
		speed_ax[LEFT][i] = 100;
	}
	
	for (i=0;i<4;i++)
	{
		// 최종 위치 저장.
		g_ARM_Position[RIGHT][i] = position_ax[RIGHT][i];
		g_ARM_Position[ LEFT][i] = position_ax[ LEFT][i];
	}

	Sync_Write_BODY(position_ax, speed_ax);

	printf("\r\n");
    return;
}

void Get_COG_Data(void)
{
	int i, k;
	double theta[2][6];		// degree
	int position_ax[2][6];
	double Pcog[3], Torque[3];;
    double mtx[3][4];
	double FootPosition[2][3];

	int Result;
	unsigned char TxData[BUFFER_LENGTH];
	unsigned char pRxData[BUFFER_LENGTH];

	// 현재  Rx 버퍼의 값을 버림
	DUMP_RX_DATA;

	///////////////////////////
	// 다리의 현재 위치 읽기

	///////////////////////////
	// Make Packet
	// [FF][FF][ID][LEN][CMD][30][P_L][P_H][S_L][S_H][CHECKSUM]
	// 공통부
	TxData[0] = TxData[1] = 0xFF;
	// TxData[2] = ID;
	TxData[3] = 4;
	TxData[4] = INST_READ;
	TxData[5] = P_PRESENT_POSITION_L;
	TxData[6] = 2;
	// TxData[7] = CalCheckSum(TxData,7);
	for (k=0;k<2;k++)
	{
		for (i=0;i<6;i++)
		{
			TxData[2] = LEG_ID[k][i];
			TxData[7] = CalCheckSum(TxData,7);

			// Data 송신.
			SET_RTS;					// 송신 모드로 변경
			write(serial_fd,TxData, 8);
			RESET_RTS;					// 수신 모드로 변경

            delay(5);                  // 1ms delay

			// 데이터 수신
			Result = RX_data_check(pRxData);

            // 060807 수정, ID 확인
            if ((Result == 8) && (pRxData[2] == TxData[2]))
			{
				position_ax[k][i] = pRxData[5]+(pRxData[6]<<8);
			}
			else
			{
                i--;			// 요청 명령을 다시 보낸다.

                // 060807 추가.
				delay(5);      // 1ms delay
				DUMP_RX_DATA;	// 현재  Rx 버퍼의 값을 버림

                // printf("Cannot get %s[%2d]'s position\r\n",(k==0)?"RIGHT_LEG":"LEFT_LEG",LEG_ID[k][i]);
                // goto Exit;
			}
		}
		// 현재 위치를 각도로 변환.
		LEG_P2D(position_ax[k], theta[k], k);
	}

	//////////////////////////////////////////
	// 현재 다리위치를 이용하여 COG 계산.
	//////////////////////////////////////////
        Cal_Torque_LEG(theta, Torque, Pcog, LEFT_LEG);

	//////////////////////////////////////////
	// 현재 다리위치를 이용하여 발의 위치 계산.
	//////////////////////////////////////////
	for (k=0;k<2;k++)
	{
        TCF_for_M2(theta[k], mtx, k);

		FootPosition[k][X] = mtx[X][3];
		FootPosition[k][Y] = mtx[Y][3];
		FootPosition[k][Z] = mtx[Z][3];
	}

	//////////////////////////////////////////
	// 결과 출력
	//////////////////////////////////////////
	printf("RIGHT_LEG_Theta \r\n");
	printf("   %6.1f, %6.1f, %6.1f, %6.1f, %6.1f, %6.1f\r\n",
			theta[RIGHT_LEG][0],theta[RIGHT_LEG][1],theta[RIGHT_LEG][2],
			theta[RIGHT_LEG][3],theta[RIGHT_LEG][4],theta[RIGHT_LEG][5]);
	printf("LEFT_LEG_Theta \r\n");
	printf("   %6.1f, %6.1f, %6.1f, %6.1f, %6.1f, %6.1f\r\n",
			theta[LEFT_LEG][0],theta[LEFT_LEG][1],theta[LEFT_LEG][2],
			theta[LEFT_LEG][3],theta[LEFT_LEG][4],theta[LEFT_LEG][5]);

	printf("RIGHT_Foot_Positon \r\n");
	printf("   %6.1f, %6.1f, %6.1f\r\n",
			FootPosition[RIGHT_LEG][X],FootPosition[RIGHT_LEG][Y],FootPosition[RIGHT_LEG][Z]);

	printf("LEFT_Foot_Positon \r\n");
	printf("   %6.1f, %6.1f, %6.1f\r\n",
			FootPosition[LEFT_LEG][X],FootPosition[LEFT_LEG][Y],FootPosition[LEFT_LEG][Z]);

	printf("COG \r\n");
	printf("   %6.1f, %6.1f, %6.1f\r\n",Pcog[X], Pcog[Y], Pcog[Z]);

	return;
}

void Change_Comp_Slope(char* pArgument[MAX_TOK],int ArgLen)
{
	int i;
	int Arg[2];

	int ID;
	unsigned char pTable[2];

	//////////////////////////////
	// 인자 추출
	/////////////////////////////
	// wc [ID][SlopeValue]
	if (ArgLen < 3)	{printf("Not enough parameter. \r\n");	return; }

	for (i=0;i<2;i++)	Arg[i] = atof(pArgument[i+1]);

	ID = (((int)Arg[0] == 0)? BROADCASTING_ID : (int)Arg[0]);
	pTable[0] = pTable[1] = (int)Arg[1];

	// AX-12로 데이터를 써 넣음.
	WriteHex(ID, P_CW_COMPLIANCE_SLOPE, pTable, 2);

	return;
}

// Position으로 이동.
// ID POSITION SPEED
// Speed가 없는 경우에는 100으로 함.
void Move_Position(char* pArgument[MAX_TOK],int ArgLen)
{
	int ID, Position, Speed, Result;
	unsigned char TxData[BUFFER_LENGTH];
	unsigned char RxData[BUFFER_LENGTH];
	

	// mv [ID][POSITION][SPEED]
	// speed 지정하지 않으면 default(100)으로 설정
	if (ArgLen < 3)	{printf("Not enough parameter. \r\n");	return; }
	// ID
	ID = atoi(pArgument[1]);
	if ((ID < 0x00) || (ID > 0xFF))
	{
		printf("Out of range : ID = 0 ~ 255 \r\n");
		return;
	}

	// Position
	Position = atoi(pArgument[2]);
	if ((Position < 0x00) || (Position > 0x3FF))
	{
		printf("Out of range : Position = 0 ~ 1023 \r\n");
		return;
	}

	// Speed
	if (ArgLen == 3)	Speed = 100;	// Default
	else			Speed = atoi(pArgument[3]);
	if ((Speed < 0x00) || (Speed > 0x3FF))
	{
		printf("Out of range : Speed = 0 ~ 1023 \r\n");
		return;
	}

	// Make Packet
	// [FF][FF][ID][LEN][CMD][P_L][P_H][S_L][S_H][CHECKSUM]
	// Header
	TxData[0] = TxData[1] = 0xFF;
	TxData[2] = ID;
	TxData[3] = 7;
	TxData[4] = INST_WRITE;
	TxData[5] = P_GOAL_POSITION_L;
	TxData[6] = Position & 0xFF;
	TxData[7] = (Position >> 8) & 0xFF;
	TxData[8] = Speed & 0xFF;
	TxData[9] = (Speed >> 8) & 0xFF;
	TxData[10] = CalCheckSum(TxData,10);

	// Data 송신.
	if (ID == BROADCASTING_ID)
	{
		// SendHexStr_to_TxQueue(TxData);		// 0 ~ i까지 데이터 송신
		printf("Select one AX-12. Broadcast is not allowed.");
		printf("\r\n");
	}
	else
	{
		SET_RTS;					// 송신 모드로 변경
		write(serial_fd,TxData, 11);
		RESET_RTS;					// 수신 모드로 변경
		printf("Send : ");	PrintHexStr(TxData,11);	printf("\r\n");
		
		delay(5);					// 1ms delay
		// 데이터 수신 후 기록
		Result = RX_data_check(RxData);
		printf("Receive(%d) : ",Result); PrintHexStr(RxData,Result); printf("\r\n");
	}

	printf("\r\n");
    return;
}

// 무게중심의 확인을 위한 함수
// R0, L0를 기준으로 한 좌표값의 입력으로 Theta 계산
// Theata에 따라 LF를 기준으로 COG 계산.
// #define 	LIFT	0	// 이 값 만큼 오른쪽 발을 들어 올린다.

void Check_Mtx(char* pArgument[MAX_TOK],int ArgLen)
{
	int i;
	double theta[2][6];		// degree
	double Pcog[3], Torque[3];

	double mtx[3][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0}};

	int speed_ax[2][6], position_ax[2][6];
	int state[MAX_SOL];
	double jt_deg[MAX_SOL][6];
	int LIFT;

	////////////////////////////////////
	// X,Y,Z 축의 목표 위치를 구함.
	// 역기구학
	////////////////////////////////////
	// cc [X pos][Y pos][Z pos], 4개 이하면 파라메터 부족
	if (ArgLen < 4)	{printf("Not enough parameter. \r\n");	return; }

	// 참고) mtx는 	mtx[0][3](x_pos), mtx[1][3](y_pos), mtx[2][3](z_pos)만 사용함.
	for (i=0;i<3;i++)	mtx[i][3] = atof(pArgument[i+1]);

	// Lift값(오른쪽 발을 들어올릴 정도)를 구함, 생략하면 0임.
	if (ArgLen == 4)	LIFT = 0;
	else				LIFT = atof(pArgument[4]);

	// 역기구학의 계산.(left)
	(void)Inverse_Kin_M2(mtx, jt_deg, state);

	if (state[0] == 0) {printf("No Solution \r\n");		return;}
	for (i=0;i<6;i++)       theta[LEFT_LEG][i] = jt_deg[0][i];

	// 역기구학의 계산.(right)
	// mtx[Z][POS] = mtx[Z][POS] - LIFT;
	mtx[Z][POS] = mtx[Z][POS] + LIFT;	// 060808 수정. 좌표계에 따르기 위함.
	(void)Inverse_Kin_M2(mtx, jt_deg, state);

	if (state[0] == 0)
	{
	        printf("No Solution \r\n");
	        return;
	}
	for (i=0;i<6;i++)       theta[RIGHT_LEG][i] = jt_deg[0][i];

	// 회전각.
	printf("Angle = ");
	for (i=0;i<6;i++)	printf("[%9.4f]",theta[0][i]);
	printf("\r\n");

	// 무게중심계산
	if (LIFT <= 0)
	{
		Cal_Torque_LEG(theta, Torque, Pcog, LEFT_LEG);
		DBG_Print_P((char*)"LF based COG ", Pcog);
	}
	else
	{
		Cal_Torque_LEG(theta, Torque, Pcog, RIGHT_LEG);
		DBG_Print_P((char*)"RF based COG ", Pcog);
	}

	// 이동 확인.
	printf("Press 'c' to cancel. Press other key to start\r\n");
	while(kbhit());
	if (getch() != 'c')
	{
		// 오른쪽 다리 위치로 변환.
		LEG_D2P(theta[RIGHT_LEG], position_ax[RIGHT_LEG], RIGHT_LEG);

		// 왼쪽 다리 위치로 변환.
		LEG_D2P(theta[LEFT_LEG], position_ax[LEFT_LEG], LEFT_LEG);

		// Make & Send Packet
		Cal_LEG_Speed(position_ax, speed_ax, NORMAL);
		Sync_Write_LEG(position_ax, speed_ax);
	}

	printf("\n");
	return;
}

void get_g_Position()
{
	int i, ID;
	int Result;
	int temp[20];
	unsigned char TxData[BUFFER_LENGTH];
	unsigned char pRxData[BUFFER_LENGTH];
	
	TxData[0] = TxData[1] = 0xFF;
	TxData[3] = 4;
	TxData[4] = INST_READ;
	TxData[5] = P_PRESENT_POSITION_L;
	TxData[6] = 2;
	
	for(ID=1;ID<20;ID++)
	{
		// Make Packet
		// [FF][FF][ID][LEN][CMD][30][P_L][P_H][S_L][S_H][CHECKSUM]
		// Header
		TxData[2] = ID;
		TxData[7] = CalCheckSum(TxData,7);
	
		// Data 송신.
		Dump_RX_data();
		SET_RTS;					// 송신 모드로 변경
		write(serial_fd,TxData, 8);
		RESET_RTS;					// 수신 모드로 변경
			
		delay(10);					// 1ms delay
		// 데이터 수신
		Result = RX_data_check(pRxData);
		temp[ID] = pRxData[5]+(pRxData[6]<<8);
		// printf("%2d[%d]=%4d,",ID, Result, temp[ID]);
		if (Result != 8)	
		{
			printf("Error while reading position [%d]\r\n",ID);
			ID = ID - 1;
		}
		for(i=0;i<10;i++)	{	pRxData[i] = 0;	}	// 좀 넉넉하게 Clear...
	}
	printf("\r\n");
	g_LEG_Position[RIGHT][0] = temp[ 8];
	g_LEG_Position[RIGHT][1] = temp[10];
	g_LEG_Position[RIGHT][2] = temp[12];
	g_LEG_Position[RIGHT][3] = temp[14];
	g_LEG_Position[RIGHT][4] = temp[16];
	g_LEG_Position[RIGHT][5] = temp[18];
	
	g_LEG_Position[ LEFT][0] = temp[ 9];
	g_LEG_Position[ LEFT][1] = temp[11];
	g_LEG_Position[ LEFT][2] = temp[13];
	g_LEG_Position[ LEFT][3] = temp[15];
	g_LEG_Position[ LEFT][4] = temp[17];
	g_LEG_Position[ LEFT][5] = temp[19];
	
	g_ARM_Position[RIGHT][0] = temp[1];
	g_ARM_Position[RIGHT][1] = temp[2];
	g_ARM_Position[RIGHT][2] = temp[5];
	g_ARM_Position[RIGHT][3] = temp[7];
	
	g_ARM_Position[ LEFT][0] = temp[1];
	g_ARM_Position[ LEFT][1] = temp[3];
	g_ARM_Position[ LEFT][2] = temp[4];
	g_ARM_Position[ LEFT][3] = temp[6];
}   
