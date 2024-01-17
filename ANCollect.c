#include "Extern.h"

extern void WRITE_EXTERNRAM(unsigned int Offset, unsigned int Data);/*中断程序中字写处理模块*/
extern unsigned int READ_EXTERNRAM(unsigned int Offset);			  /*中断程序中字读处理模块*/
const unsigned char TD_Index[Sample_Total] =
{
	0x00,0x10,0x20,0x30,0x40,0x50,0x70,0x80,0x90,	//一级通道=0;二级通道:0温度校准电压\1测温点01\2测温点17\3加热通道1状态\4加热通道17状态\5加热通道25状态\7+12V工作电压\8+5V工作电压\9-12V工作电压,共9路
		 0x11,0x21,0x31,0x41,0x51,					//一级通道=1;二级通道:              1测温点02\2测温点18\3加热通道2状态\4加热通道18状态\5加热通道26状态,共5路
		 0x12,0x22,0x32,							//一级通道=2;二级通道:              1测温点03\2测温点19\3加热通道3状态,共3路
		 0x13,0x23,0x33,							//一级通道=3;二级通道:              1测温点04\2测温点20\3加热通道4状态,共3路
		 0x14,0x24,0x34,0x44,						//一级通道=4;二级通道:              1测温点05\2测温点21\3加热通道5状态\4加热通道21状态,共4路
		 0x15,0x25,0x35,0x45,						//一级通道=5;二级通道:              1测温点06\2测温点22\3加热通道6状态\4加热通道22状态,共4路
		 0x16,0x26,     0x46,0x56,					//一级通道=6;二级通道:              1测温点07\2测温点23\               4加热通道23状态\5加热通道31状态,共4路
		 0x17,0x27,     0x47,0x57,					//一级通道=7;二级通道:              1测温点08\2测温点24\               4加热通道24状态\5加热通道32状态,共4路
		 0x18,               0x58,					//一级通道=8;二级通道:              1测温点09\                                         5加热通道33状态,共2路
		 0x19,               0x59,					//一级通道=9;二级通道:              1测温点10\                                         5加热通道34状态,共2路
		 0x1A,     0x3A,     0x5A,					//一级通道=A;二级通道:              1测温点11\          3加热通道11状态\               5加热通道35状态,共3路
		 0x1B,     0x3B,     0x5B,					//一级通道=B;二级通道:              1测温点12\          3加热通道12状态\               5加热通道36状态,共3路
		 0x1C,     0x3C,     0x5C,					//一级通道=C;二级通道:              1测温点13\          3加热通道13状态\               5加热通道37状态,共3路
		 0x1D,     0x3D,							//一级通道=D;二级通道:              1测温点14\          3加热通道14状态,共2路
		 0x1E,     0x3E,							//一级通道=E;二级通道:              1测温点15\          3加热通道15状态,共2路
		 0x1F,     0x3F,							//一级通道=F;二级通道:              1测温点16\          3加热通道16状态,共2路
};		//总共55路

void AnCollect(void)
{
	unsigned char register i,j,td;
	unsigned int register Rtmp,TotalRtmp;
	TotalRtmp = 0;
	AncMax = 0;
	AncMin = 0x7ff;
	td = TD_Index[TimerCount];
	XBYTE[ADDR_CJTD] = td & 0x0F;   						/*采集通道一级选择*/
	_nop_();	/*空等待*//*延时，消除干扰*/
	_nop_();	/*空等待*/
	XBYTE[ADDR_CJTD] = td;   		/*采集通道二级选择*/
	_nop_();	/*空等待*//*延时，消除干扰*/
	for (i = 0; i < 6; i++)/*采集6次,去高1去低1,剩余4个量求平均值*/
	{
		AD_CONVST = 0;  		   /*设置转换模式*/
		_nop_();                  /* 延时,消除干扰 */
		_nop_();
		AD_CONVST = 1;  		   /*设置转换模式*/                  
		_nop_();                  /* 延时,消除干扰，等待转换完成 */
		_nop_();
		AD_RFS = 0; 			   /*设置数据帧同步,开始读数据*/
		_nop_();                  /* 延时,消除干扰 */
		_nop_();
		for (Rtmp = 0,j = 0; j < 16; j++)	  /* 串行数据初始化,采集16位数据 */ 
		{
			Rtmp <<= 1;
			SDATA = 1;		/*读数据前数据输入位先置高*/
			SCLK = 0;   	    	 /* 下降沿读数 */
			if(SDATA)
			{
				Rtmp |=1;    
			}
			SCLK = 1;   			 /* 上升沿载数clock out */
		}
		AD_RFS = 1; 			   /*设置数据帧同步,读取数据结束*/
		_nop_();                  /* 延时,消除干扰 */
		if (0x800 == (0x800 & Rtmp))   //Tzh 判断是否溢出，文档中描述为负值。
		{
			Rtmp = 0;
		}
		TotalRtmp += (unsigned long)Rtmp;

		if (AncMax < Rtmp)  	  /*统计最大、最小值*/
		{
			AncMax = Rtmp;
		}
		if(AncMin > Rtmp)
		{
			AncMin = Rtmp;
		}
	}
	switch(td)	/* 存储数据重新排序[0-3]:JZ/5VTM/12VTM/12VTM*  [0x10-0x6f]:32路热敏电阻+40路加热器状态 */
	{
	case 0x70:
		td = 1;
		break;
	case 0x80:
		td = 2;
		break;
	case 0x90:
		td = 3;
		break;
	default:
		break;
	}
	/* 对采集到的模拟量进行滤波处理 */
	TotalRtmp = TotalRtmp - AncMax - AncMin + 2;		/*剔除最大值、最小值*///Tzh 为什么+2
	WRITE_EXTERNRAM((unsigned int) (AnC_Buf + td), (TotalRtmp >> 2));  /* 将采集数据存储到临时存储器 */
}
void PowerDetect(void)
{
	unsigned int register Rtmp;
	Rtmp = READ_EXTERNRAM(AnC_Buf + 1);		//+12V工作电压
	if(((Base_P12 - Error_THRESHOLD) > Rtmp) ||  ((Base_P12 + Error_THRESHOLD) < Rtmp))
	{
		Status |= 0x10;		//Status.4=1
	}
	else
	{
		Status &= 0x0f;		//Status.4=0
	}
	Rtmp = READ_EXTERNRAM(AnC_Buf + 2);		//+5V工作电压
	if(((Base_5V - Error_THRESHOLD) > Rtmp) ||  ((Base_5V + Error_THRESHOLD) < Rtmp))
	{
		Status |= 0x20;		//Status.5=1
	}
	else
	{
		Status &= 0x1f;		//Status.5=0
	}
	Rtmp = READ_EXTERNRAM(AnC_Buf + 3);		//-12V工作电压
	if(((Base_S12 - Error_THRESHOLD) > Rtmp) ||  ((Base_S12 + Error_THRESHOLD) < Rtmp))
	{
		Status |= 0x40		//Status.6=1
	}
	else
	{
		Status &= 0x3f		//Status.6=0
	}
	Rtmp = READ_EXTERNRAM(AnC_Buf + 0);		//温度校准电压
	if(((Base_Sensor - Error_THRESHOLD) > Rtmp) ||  ((Base_Sensor + Error_THRESHOLD) < Rtmp))
	{
		Status |= 0x80;		//Status.7=1
	}
	else
	{
		Status &= 0x7f;		//Status.7=0
	}
}										
/**************************************************************************
模块名称:	定时中断模块                
模块描述:   1.每毫秒中断1次
			2.对24路测温信号、27路加热状态反馈电压、4路工作电压进行采集，每毫秒对1路信号采集6次，负数取0。剔除最大值、最小值。其他取平均。
			3.采集完成后验证AD电路采集+5V（采样值512）、+12V（采样值1229）、-12V（采样值1229）工作电压及温度校准电压（采样值512）的正确性
			4.采集完成后，设置采集完成标志。
			5.对串口超时进行判断：结束发送和接收。
输入参数:	无
输出参数:	无
返回值:		无
修改记录:   
其他说明:
**************************************************************************/ 
void Timer0_INT(void) interrupt 1 using 1
{

	TR0 = 0;			  /*停止定时器0工作*/
	TH0 = TH_1ms;		  /*定时器0时间1ms初始化*/
	TL0 = TL_1ms; 		  
	TR0 = 1;			  /*启动定时器0工作*/
	synTime.TA ++;		  /*定时器计数累加*/
	if (ReceiveingFlag)
	{
		ReceiveCount ++;
		if (ReceiveCount > OverTime)
		{
			SM2 =1;
			ReceiveingFlag = 0;		//停止记时
			ReceiveCount = 0;
		}
	}
	if (SendingFlag)
	{
		SendCount ++;
		if (SendCount > OverTime)
		{
			CommuniationDisable = 1;		//断开通讯链路
			SendingFlag = 0;		//停止记时
			SendCount = 0;
		}
	}
	TimerCount++;		  /*内部定时器计数累加*/
	if (CtrlPeriod == TimerCount)
	{
		TimerCount = 0;

		TimerState_Cont ++;
	}
	if ((Sample_Total > TimerCount) && (wkStatus != vaild_Status))
	{
		AnCollect();
	}

	else if (Sample_Total == TimerCount)
	{
		PowerDetect();
	}
	else  if ((Sample_Total +1 ) == TimerCount)
	{
		ctrlFlag = 1；
	}
	else if((CtrlPeriod - 50) == TimerCount)
	{
		ctrlFlag = 0；
	}
	else
	{
		//空语句
	}
}
