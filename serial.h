// 화면 출력 없이 RX Data를 없앰.
#define DUMP_RX_DATA	{while (IsReadyToRX) g_Qfirst++;}

//*****************************************************************
//    함수선언
//*****************************************************************
void* serial_t1(void* );  // 이런 형 선언이 되나??? 안되는것 같다...

// LAN 통신
void Socket_Open(void);
void Socket_Close(void);
void Send_Init_for_LAN(void);

// Hex String을 화면으로 출력.
void PrintHexStr(unsigned char* bpBuffer, int bCount);

// 양쪽 LEG의 Serial Data 전송
void Sync_Write_LEG(int position_ax[2][6],int speed_ax[2][6]);
void Sync_Write_BODY(int position_ax[2][4],int speed_ax[2][4]);
void Sync_Write_ALL(int position_ax_LEG[2][6],int speed_ax_LEG[2][6],
					int position_ax_ARM[2][4],int speed_ax_ARM[2][4] );
// Serial Data의 checksum 계산.
unsigned char CalCheckSum(unsigned char* TxData, int Count);

// Packet을 TX Queue로 전송.
// TX Queue는 Timer Interrupt에서 보낸다.
void SendHexStr_to_TxQueue(unsigned char* bpBuffer);

int RX_data_check(unsigned char* pRxData);

void Dump_RX_data(void);

void WriteHex(int ID, unsigned char Addr, unsigned char * pTable, int Length);
void ReadHex(int ID, unsigned char Addr, unsigned char * pTable, int Length);

// 시리얼 데이터 송신 완료시까지 대기
void reset_RTS(void);

