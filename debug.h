void DBG_Print_T(char* Title, const double mtx[3][4]);
void DBG_Print_theta(char* Title, const double theta[6]);
void DBG_Print_P(char* Title, const double P[3]);
void DBG_LF(void);
void DBG_R_default(double mtx[3][4]);
void DBG_L_default(double mtx[3][4]);
// Sensor�� �д� Thread
void* sensor_t1(void* );
// Linux���� ����ϱ� ���� DOS �Լ� ����
int kbhit(void);
int getch(void);
int max(int a,int b);
int min(int a,int b);
// �������
