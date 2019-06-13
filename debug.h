void DBG_Print_T(char* Title, const double mtx[3][4]);
void DBG_Print_theta(char* Title, const double theta[6]);
void DBG_Print_P(char* Title, const double P[3]);
void DBG_LF(void);
void DBG_R_default(double mtx[3][4]);
void DBG_L_default(double mtx[3][4]);
// Sensor값 읽는 Thread
void* sensor_t1(void* );
// Linux에서 사용하기 위한 DOS 함수 구현
int kbhit(void);
int getch(void);
int max(int a,int b);
int min(int a,int b);
// 여기까지
