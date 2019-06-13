/////////////////////////////////////////////
// Type 선언
/////////////////////////////////////////////
typedef unsigned char	INT8U;			//Unsigned  8 bit quantity
typedef signed	 char	INT8S;			//Signed    8 bit quantity
typedef unsigned int	INT16U; 		//Unsigned 16 bit quantity
typedef signed	 int	INT16S; 		//Signed   16 bit quantity
typedef unsigned long	INT32U; 		//Unsigned 32 bit quantity
typedef signed	 long	INT32S; 		//Signed   32 bit quantity
typedef float			FP32;			//Single precision floating point
typedef double			FP64;			//Double precision floating point

#define BYTE			INT8U
#define WORD			INT16U
#define LONG			INT32S
#define DWORD			INT32U
#define UINT			INT16U

typedef UINT		    WPARAM;
typedef LONG		    LPARAM;
typedef LONG		    LRESULT;
typedef UINT		    ATOM;

#define PASCAL		    _pascal
#define WINAPI		    _far _pascal
#define CALLBACK	    _far _pascal

#ifndef TRUE
#define FALSE		    0
#define TRUE		    1
#endif

#define FAR		    	_far
#define NEAR		    _near

//******************************************************/
// Kinhead.h
//******************************************************/
// 변환식...
#define 	DEGTORAD 	0.017453293
#define 	RADTODEG 	57.29577951
#define 	EPSI 		0.000001
#define 	PI 			3.141592654
#define 	PI_DEG 		180.0
#define 	LARGE 		100000.0

// Degree와 Position의 변환식.
#define CONV_D2P	3.41		// 1023/300 = 3.41
#define CONV_P2D	0.293255	// 300/1023 = 0.29325513196480938416422287390029

// DH Parameter
#define		DH_d2 	0
#define		DH_a3 	80.5
#define		DH_a4 	80.5
#define		DH_a6 	32.2

#define 	B_DH_a1	 73.4
#define 	B_DH_a2	-14.9
#define 	B_DH_a3	 68.0
#define 	B_DH_a4	 94.0

// [0] == 우, [1] == 좌
#define RIGHT_LEG	0
#define LEFT_LEG	1
#define RIGHT		0
#define LEFT		1

// for debugging
#define check 	printf("check\n"); getch();

// 역기구학의 최대 해의 개수
#define SOL_1	0
#define SOL_2	2
#define MAX_SOL	4

// AX-12의 속도 변환
#define NORMAL			 1.00	// 주기적인 제어를 하지 않는 경우
#define PERIOD_20MSEC	21.89	// 20msec마다 제어하는 경우
#define PERIOD_10MSEC	43.86	// 10msec마다 제어하는 경우.
// 중간 중간의 delay 요인(printf)를 없앴더니 속도변환 30으로는 제대로 처리되지 않는다???
#define EXPERIENCE		50.00	// 경험에 의한 값...

#define SPEED_MIN	100		// 처음엔 10이었다
#define SPEED_MAX	1023

// 각 관절의 무게, 단위 : g
#define G_C_MC	226	
#define G_C_M3	172
#define G_C_M4	86
#define G_C_M6	176

#define G_C_MA1	586
#define G_C_MA3	74
#define G_C_MA4	74

// 위치
#define P_BC_0	0
#define P_BC_4	1
#define P_BC_6	2
#define P_BC_F	3

#define PX	0
#define PY	1
#define PZ	2

#define X	0
#define Y	1
#define Z	2

#define POS	3

// 초기 X 위치. ( {BC}를 기준 좌표계로 함. )
#define X_BASE_POS      -220

// 각 관절의 위치 Index
#define R6 0
#define R4 1
#define R3 2
#define BC 3
#define L3 4
#define L4 5
#define L6 6
#define LEG_TORQUE 7

#define AR1	7
#define AR3	8
#define AR4	9
#define AL3	10
#define AL4	11
#define ALL_TORQUE 12

#define BR1	0
#define BR3	1
#define BR4	2
#define BL3	3
#define BL4	4
#define BODY_TORQUE 5

#define G_Acc 9.8

//******************************************************/
// Serial.h
//******************************************************/
// 데이터 저장을 위한 버퍼와 버퍼 정보 변수
// Queue Index를 증감할때 Rount되도록 하기 위함.
#define BUFFER_LENGTH 	256
#define PACKET_LENGTH	(BUFFER_LENGTH - 1)

#define TX_QUEUE_LENGTH	16

#ifndef FALSE
	#define FALSE 0
#endif
#ifndef TRUE
	#define TRUE 1
#endif

// GPIO Pin Setup
#define RTS	1	// wPi 1(GPIO_GEN1, pin#12)
// RTS 신호를 SET -> 송신모드
#define SET_RTS		digitalWrite(RTS,LOW)
// RTS 신호를 RESET -> 수신모드
#define RESET_RTS	reset_RTS();  // digitalWrite(RTS,HIGH);		// 아두이노: SET-LOW, RESET-HIGH

//*****************************************************************
//    매크로 선언
//*****************************************************************
// 송신 인터럽트와 수신 쓰레드의 변수 공유를 위한 Key
// piLock(int KeyNumber)와 piUnlock(int KeyNumber)에서 사용됨
// KeyNumber는 0 ~ 3까지 사용 가능
#define KEY_WRITE 0
#define KEY_READ  1
#define KEY_I2C	  2
	  
// ----------------------
// 수신 Queue
// ----------------------
// 읽을 데이터가 있는지 확인, First와 last가 같지 않으면 읽을 데이터가 있다.
// g_Qfirst : 지금 데이터를 읽어야 할 지점을 가리킴. 
// g_Qlast : 다음 입력할 버퍼의위치를 가리킴. 
// Queue가 가득차서 overflow되는것은 Buffer에 쓸때(인터럽트에서) 방지해야 한다.
#define IsReadyToRX	(g_Qfirst!=g_Qlast)

// 다음 읽을 위치로 이동.
// #define NEXT_FIRST  {g_Qfirst++; if (g_Qfirst==BUFFER_LENGTH)	g_Qfirst = 0;}
// Queue의 크기가 256개 이므로 그냥 하나 증가시키면 알아서 처리된다. 
#define NEXT_FIRST  g_Qfirst++; 

// 다음 쓸 위치로 이동.
// #define NEXT_LAST   {g_Qlast++;  if (g_Qlast ==BUFFER_LENGTH)	g_Qlast  = 0;}
// Queue의 크기가 256개 이므로 그냥 하나 증가시키면 알아서 처리된다. 
#define NEXT_LAST   g_Qlast++;


// ----------------------
// 송신 Packet Queue
// ----------------------
// 읽을 데이터가 있는지 확인, First와 last가 같지 않으면 읽을 데이터가 있다.
// g_Tx_Qfirst : 지금 데이터를 읽어야 할 지점을 가리킴. 
// g_Tx_Qlast : 다음 입력할 버퍼의위치를 가리킴. 
// Queue가 가득차서 overflow되는것은 Buffer에 쓸때(인터럽트에서) 방지해야 한다.
#define IsReadyToTX		(g_Tx_Qfirst!=g_Tx_Qlast)
// Queue가 가득 찻는지 확인.
#define IsTxQueueFull 	(((g_Tx_Qlast+1)&0x0F) == (g_Tx_Qfirst))
// Queue가 비어있는지 확인.
#define IsTxQueueEmpty 	(g_Tx_Qfirst == g_Tx_Qlast)

// 다음 읽을 위치로 이동.
// Queue의 크기가 16개 이므로 그냥 하나 증가시키고 하위 Nibble을 취하면 알아서 처리된다. 
#define NEXT_TX_FIRST  {g_Tx_Qfirst = (g_Tx_Qfirst+1)&0x0F;}

// 다음 쓸 위치로 이동.
// Queue의 크기가 16개 이므로 그냥 하나 증가시키고 하위 Nibble을 취하면 알아서 처리된다. 
#define NEXT_TX_LAST   {g_Tx_Qlast = (g_Tx_Qlast+1)&0x0F;}

//******************************************************/
// Main.h
//******************************************************/
///////////////////////////////////////////////
//--- AX-12 ---
//--- Control Table Address ---
///////////////////////////////////////////////
//EEPROM AREA
#define P_MODEL_NUMBER_L 			0
#define P_MODOEL_NUMBER_H 			1
#define P_VERSION 					2
#define P_ID 						3
#define P_BAUD_RATE 				4
#define P_RETURN_DELAY_TIME 		5
#define P_CW_ANGLE_LIMIT_L 			6
#define P_CW_ANGLE_LIMIT_H 			7
#define P_CCW_ANGLE_LIMIT_L 		8
#define P_CCW_ANGLE_LIMIT_H 		9
#define P_SYSTEM_DATA2 				10
#define P_LIMIT_TEMPERATURE 		11
#define P_DOWN_LIMIT_VOLTAGE 		12
#define P_UP_LIMIT_VOLTAGE 			13
#define P_MAX_TORQUE_L 				14
#define P_MAX_TORQUE_H 				15
#define P_RETURN_LEVEL 				16
#define P_ALARM_LED 				17
#define P_ALARM_SHUTDOWN 			18
#define P_OPERATING_MODE 			19
#define P_DOWN_CALIBRATION_L 		20
#define P_DOWN_CALIBRATION_H 		21
#define P_UP_CALIBRATION_L 			22
#define P_UP_CALIBRATION_H 			23

#define P_TORQUE_ENABLE 			(24)
#define P_LED 						(25)
#define P_CW_COMPLIANCE_MARGIN 		(26)
#define P_CCW_COMPLIANCE_MARGIN		(27)
#define P_CW_COMPLIANCE_SLOPE 		(28)
#define P_CCW_COMPLIANCE_SLOPE 		(29)
#define P_GOAL_POSITION_L 			(30)
#define P_GOAL_POSITION_H 			(31)
#define P_GOAL_SPEED_L 				(32)
#define P_GOAL_SPEED_H 				(33)
#define P_TORQUE_LIMIT_L 			(34)
#define P_TORQUE_LIMIT_H 			(35)
#define P_PRESENT_POSITION_L 		(36)
#define P_PRESENT_POSITION_H 		(37)
#define P_PRESENT_SPEED_L 			(38)
#define P_PRESENT_SPEED_H 			(39)
#define P_PRESENT_LOAD_L 			(40)
#define P_PRESENT_LOAD_H 			(41)
#define P_PRESENT_VOLTAGE 			(42)
#define P_PRESENT_TEMPERATURE 		(43)
#define P_REGISTERED_INSTRUCTION	(44)
#define P_PAUSE_TIME 				(45)
#define P_MOVING 					(46)
#define P_LOCK 						(47)
#define P_PUNCH_L 					(48)
#define P_PUNCH_H 					(49)

// AX-12 Instruction
#define INST_PING					0x01
#define INST_READ 					0x02
#define INST_WRITE 					0x03
#define INST_REG_WRITE 				0x04
#define INST_ACTION 				0x05
#define INST_RESET 					0x06
#define INST_DIGITAL_RESET 			0x07
#define INST_SYSTEM_READ 			0x0C
#define INST_SYSTEM_WRITE 			0x0D
#define INST_SYNC_WRITE 			0x83
#define INST_SYNC_REG_WRITE 		0x84

//////////////////////////////////////////
// 통신 Port 선언 
//////////////////////////////////////////
// Serial
#define PORT	"/dev/ttyS0"
// #define PORT	"/dev/ttyAMA0"

// LAN
#define SERVER_ADDR "192.168.0.3"
#define SERVER_PORT 5560
#define BUFSIZE 20000	// 통신 Buffer Size // 8byte * 9 * 2 * 100 = 14400 byte	// 500ms 동안의 Data 송신

// AX-12 ID
#define MAX_ID 	19
#define BROADCASTING_ID	0xFE

//////////////////////////////////////////
// MPU-6050 관련 선언 
//////////////////////////////////////////
// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
// #define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL
