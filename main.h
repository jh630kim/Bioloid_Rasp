#define MAX_TOK 20

/////////////////////////////////////////
// ÇÔ¼ö Header
/////////////////////////////////////////
void get_g_Position(void);
void MonitorRoutine(char* MonCmd);
void Get_Position(char* pArgument[MAX_TOK], int ArgLen);
void Move_Position(char* pArgument[MAX_TOK],int ArgLen);
void Move_For_Kin(char* pArgument[MAX_TOK], int ArgLen);
void Move_For_Kin_BODY(char* pArgument[MAX_TOK],int ArgLen);
void Move_Inv_Kin(char* pArgument[MAX_TOK],int ArgLen);
void Move_Both_LEG(char* pArgument[MAX_TOK],int ArgLen);
void Check_Kin(char* pArgument[MAX_TOK],int ArgLen);
void Check_Kin_Body(char* pArgument[MAX_TOK],int ArgLen);
void Move_Zero(void);
void Move_Sit_Down(void);
void Move_Zero_BODY(void);
void Move_COG_Walking(char* pArgument[MAX_TOK],int ArgLen);
void Change_Comp_Slope(char* pArgument[MAX_TOK],int ArgLen);
void Get_COG_Data(void);

void Check_Mtx(char* pArgument[MAX_TOK],int ArgLen);
void Check_All_Torque(char* pArgument[MAX_TOK],int ArgLen);
void Check_BODY_Torque(char* pArgument[MAX_TOK],int ArgLen);
