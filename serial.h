// ȭ�� ��� ���� RX Data�� ����.
#define DUMP_RX_DATA	{while (IsReadyToRX) g_Qfirst++;}

//*****************************************************************
//    �Լ�����
//*****************************************************************
void* serial_t1(void* );  // �̷� �� ������ �ǳ�??? �ȵǴ°� ����...

// LAN ���
void Socket_Open(void);
void Socket_Close(void);
void Send_Init_for_LAN(void);

// Hex String�� ȭ������ ���.
void PrintHexStr(unsigned char* bpBuffer, int bCount);

// ���� LEG�� Serial Data ����
void Sync_Write_LEG(int position_ax[2][6],int speed_ax[2][6]);
void Sync_Write_BODY(int position_ax[2][4],int speed_ax[2][4]);
void Sync_Write_ALL(int position_ax_LEG[2][6],int speed_ax_LEG[2][6],
					int position_ax_ARM[2][4],int speed_ax_ARM[2][4] );
// Serial Data�� checksum ���.
unsigned char CalCheckSum(unsigned char* TxData, int Count);

// Packet�� TX Queue�� ����.
// TX Queue�� Timer Interrupt���� ������.
void SendHexStr_to_TxQueue(unsigned char* bpBuffer);

int RX_data_check(unsigned char* pRxData);

void Dump_RX_data(void);

void WriteHex(int ID, unsigned char Addr, unsigned char * pTable, int Length);
void ReadHex(int ID, unsigned char Addr, unsigned char * pTable, int Length);

// �ø��� ������ �۽� �Ϸ�ñ��� ���
void reset_RTS(void);

