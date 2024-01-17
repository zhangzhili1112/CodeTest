/****************************************
Created	by lgl	at 20181201
Modified	by	lgl	at 20181230
******************************************/
#include <math.h>
#include <absacc.h>
#include <intrins.h>
#ifndef __DEFINE_H__
#define __DEFINE_H__
#define oclock 11059200
#define myAdr 0x21		//温控仪子地址
#define BoardCastAdr 0xff	//广播子地址
#define	XDATASTART	0x00	//外部寄存器起始地址为0
#define	XDATALEN	0x8000	//外部寄存器长度(32KBytes)
#define	XBLOCKTOTAL	0x04	//外部寄存器块数
#define	XBLOCKLEN	0x2000	//外部寄存器每块长度(8KBytes)
#define HAVEDONE 0xEB	
#define CtrlPeriod	2000		//控制周期1000毫秒
/*****************控制地址表**********************************************/
#define ADDR_CJTD   	0xA000  	 /*采集通道设置地址*/
#define Sample_Total	55		/*采集信号总数*/
#define ADDR_IOSEL1 	0xA004  	 /*IOSEL1:加热通道14~20*/
#define ADDR_IOSEL2 	0xA008  	 /*IOSEL2:加热通道7~13*/
#define ADDR_IOSEL3 	0xA00C  	 /*IOSEL3:加热通道1~6*/
#define ADDR_IOSEL4 	0xA010  	 /*IOSEL4:加热通道34~40*/
#define ADDR_IOSEL5 	0xA014  	 /*IOSEL5:加热通道27~33*/
#define ADDR_IOSEL6 	0xA018  	 /*IOSEL6:加热通道21~26*/
#define IO_CMD1	0xFE  	 /*加热指令14~20*/
#define IO_CMD2	0xFE  	 /*加热指令7~13*/
#define IO_CMD3	0x7E  	 /*加热指令1~6*/
#define IO_CMD4	0xFE  	 /*加热指令34~40*/
#define IO_CMD5	0xFE  	 /*加热指令27~33*/
#define IO_CMD6	0x7E  	 /*加热指令21~26*/
/*****************使用的变量定义0x500-0xFDF*****************************************/
#define AnC_Buf 		0x900   	 /* 55个字--模拟量采集数据临时存储区from:0x1000            */
#define ArgAdr		    0xA00        /* 温控参数临时存放区 */
#define ReceiveBufStart 0x1000
#define SendBufStart	0x1080
#define HEAT_THRESHOLD    0x0300	 /* 加热器开关判据阈值:大于等于HEAT_THRESHOLD--开;否则--关*/
#define Sensor_Erk        0x03fa     /* 传感器判断开路阈值:-40*/
#define Sensor_Erd        0x001e     /* 传感器判断短路阈值:100*/
#define Error_THRESHOLD	0x33		/*最大误差值51*/
#define Base_Sensor		0x200		/*温度校准电压采样值512*/
#define Base_5V			0x200		/*+5V采样值512*/
#define Base_P12		0x4cd		/*+12V采样值1229*/
#define Base_S12		0x4cd		/*-12V采样值1229*/
#define SwitchMask	0xC0		//开关状态位
#define SwitchOn	0xC0		//开关开--接通
#define SwitchOff	0x00		//开关关--断开
/************命令定义*************/
#define SYN_CMD 0xEB		//广播同步命令--上电自检以后,空闲状态
#define SYN_CMD_L	0x04
#define SelfTest_CMD 0x11	//自检命令--上电自检以后,空闲状态
#define	SelfTest_CMD_L 0
#define SelfTest_ANSW 0x11	//自检命令应答
#define	TM_CMD 0xEE			//遥测查询命令--工作状态
#define	TM_CMD_L 0
#define TM_ANSW 0xEE		//遥测查询应答
#define WKStart_CMD 0xAA	//启动自动温控命令--空闲状态,内存、AD自检无错误
#define	WKStart_CMD_L 0
#define WKStart_ANSW 0xAA	//启动自动温控应答
#define WKorTestStop_CMD 0x55		//停止自动温控命令,与停止测试相同
#define WKStop_ANSW 0x5A	//停止自动温控应答
#define TestStart_CMD 0xBB	//启动测试命令--空闲状态,内存、AD自检无错误
#define	TestStart_CMD_L 0
#define TestStart_ANSW 0xBB	//启动测试应答
#define TestStop_ANSW 0xA5	//停止测试应答
#define	WKorTestStop_CMD_L 0
#define illCMD_ANSW 0x55	//其他状态收到停止命令应答
#define LCStart_CMD 0xB1	//启动巡检命令--空闲状态,内存、AD自检无错误
#define	LCStart_CMD_L 0
#define LCStart_ANSW 0x1B	//启动巡检应答
#define DisChannel_CMD 0xB0		//加热通道断开命令--测试状态
#define	DisChannel_CMD_L 0
#define DisChannel_ANSW 0x0B	//加热通道断开应答
#define EnChannel_CMD 0xBF		//加热通道接通命令--测试状态
#define	EnChannel_CMD_L 0
#define EnChannel_ANSW 0xFB		//加热通道接通应答
#define DEAChannel_CMD 0xBA		//按要求控制各加热通道命令--测试状态
#define	DEAChannel_CMD_L 0x5
#define DEAChannel_ANSW 0xAB	//按要求控制各加热通道应答
#define DESChannel_CMD 0xBC		//按要求控制单加热通道命令--测试状态
#define DESChannel_ANSW 0xCB	//按要求控制单加热通道应答
#define ArgLoad_CMD 0xFF		//参数装订命令--空闲状态,内存、AD自检无错误
#define ArgLoad_CMD_L 12
#define ArgLoad_ANSW 0xFF		//参数装订应答
#define XorErr 0xCA		//异或校验错
#define TimErr 0xCB		//时序错
#define CmdErr 0xCC		//命令错
#define FrameErr 0xCD		//帧结构(长度)错
#define DESChannel_Err 0xC8	//加热通道范围
#define OverTime	0x64	//收发超时100毫秒
#define Max_Frame_dL 40		//定义接收帧结构最大数据长度
/*********************定义工作状态*****************************/
#define	Auto_TempCtrl_Status	0x0F	//自动温控状态
#define	Test_Status	0xF0	//测试状态
#define	Test_Loop_Status	0xF1	//寻检状态
#define	Idel_Status	0x00	//空闲状态
#define Save_Arg_Status	0x01	//写参数
#define vaild_Status 0xFF	//故障状态,没有可用内存
/*********************定义1ms定时常数*****************************************/
#define	TH_1ms	0xFC		//计时约975.5微秒 
#define	TL_1ms	0x7D		//计时参数0xFC67，进入中断,约执行22个指令周期(响应中断约4个周期,保护现场12个周期--ACC、B、DPTR、PSW、设置寄存器组，记时器复位6个周期)
//定义数据结构
typedef struct {
	unsigned int LDT;	//其他控温动作下限
	unsigned int HDT;	//动作上限
	unsigned int CLDT;	//贮箱控温动作下限
	unsigned int LT;	//其他低温故障限
	unsigned int HT;	//高温故障限
	unsigned int QLT;	//气瓶低温故障限
} ArgStruct;
typedef	struct {
	unsigned char CLength;
	unsigned char CXor;
	unsigned char Cmd;
	unsigned char Length;
	unsigned int BuffAdr;
	unsigned int StartAdr;
} SeriBuff;
typedef union {
	unsigned char TS[4];
	unsigned long TA;
} TimeCount;
typedef union {
	unsigned long AncuLong;
	struct {
		unsigned int AncuIntL,AncuIntH;
	} AncuInt;
} TempleStr;
typedef union {
	unsigned int in;
	unsigned char ch[2];
} ICUnion;
/* 加热器地址列表 */
const unsigned int code HEAT_ADR[40] = 
{
	ADDR_IOSEL3, ADDR_IOSEL3, ADDR_IOSEL3, ADDR_IOSEL3, ADDR_IOSEL3, ADDR_IOSEL3,				/*0xA00C:01,02,03,04,05,06   */
	ADDR_IOSEL2, ADDR_IOSEL2, ADDR_IOSEL2, ADDR_IOSEL2, ADDR_IOSEL2, ADDR_IOSEL2, ADDR_IOSEL2,	/*0xA008:07,08,09,10,11,12,13*/
	ADDR_IOSEL1, ADDR_IOSEL1, ADDR_IOSEL1, ADDR_IOSEL1, ADDR_IOSEL1, ADDR_IOSEL1, ADDR_IOSEL1,	/*0xA004:14,15,16,17,18,19,20*/
	ADDR_IOSEL6, ADDR_IOSEL6, ADDR_IOSEL6, ADDR_IOSEL6, ADDR_IOSEL6, ADDR_IOSEL6,				/*0xA018:21,22,23,24,25,26   */ 
	ADDR_IOSEL5, ADDR_IOSEL5, ADDR_IOSEL5, ADDR_IOSEL5, ADDR_IOSEL5, ADDR_IOSEL5, ADDR_IOSEL5,	/*0xA014:27,28,29,30,31,32,33*/
	ADDR_IOSEL4, ADDR_IOSEL4, ADDR_IOSEL4, ADDR_IOSEL4, ADDR_IOSEL4, ADDR_IOSEL4, ADDR_IOSEL4	/*0xA010:34,35,36,37,38,39,40*/
};
/* 加热器操作指令列表 */
const unsigned char code OPEN_CMD[40] = 
{
/*  1.....................................................8*/
		0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 
		0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 
		0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02,
		0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 
		0x80, 0x40, 0x20, 0x10,0x08, 0x04, 0x02, 
		0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02
};
const unsigned char HEAT_INDEX[40] =
{/*      1       2       3       4       5       6       7       8*/
	2,2,2,2,2,2,	/*01,02,03,04,05,06   */
	1,1,1,1,1,1,1,	/*07,08,09,10,11,12,13*/
	0,0,0,0,0,0,0,	/*14,15,16,17,18,19,20*/
	5,5,5,5,5,5,	/*21,22,23,24,25,26   */
	4,4,4,4,4,4,4,	/*27,28,29,30,31,32,33*/
	3,3,3,3,3,3,3	/*34,35,36,37,38,39,40*/
};
const unsigned int code ptHL_T1_Cmd[6][5] = {
	/*SampleAdrstart,	HeatAdr-PP,	HeatCmd,	HeatAdr-PP,	HeatCmd*/
	{AnC_Buf+0x10,		2,			0x40,		2,			0x80},	//T01、T02; J01、J02:0xA00C,0x40,0xA00C,0x20
	{AnC_Buf+0x16,		1,			0x08,		1,			0x04},	//T07、T08; J11、J12:0xA008,0x08,0xA008,0x04
	{AnC_Buf+0x24,		4,			0x08,		4,			0x04},	//T21、T22;	J31、J32:0xA014,0x08,0xA014,0x04
	{AnC_Buf+0x1F,		5,			0x40,		5,			0x20},	//T16、T17; J21、J22:0xA018,0x40,0xA018,0x20
	{AnC_Buf+0x13,		2,			0x10,		2,			0x08},	//T04、T05;	J03、J04:0xA00C,0x10,0xA00C,0x08
	{AnC_Buf+0x19,		1,			0x02,		0,			0x08}	//T10、T11;	J13、J14:0xA008,0x02,0xA004,0x80
};
#endif
