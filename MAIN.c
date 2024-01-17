/****************************************
主要功能为：
1.采集数据一次为0x00   			   --ADC位置[0]
				0x10-0x2f[32路热敏电阻采集]--ADC位置[0x10-0x2f]
				0x30-0x43[1-20路加热器状态采集]--ADC位置[0x30-0x43]
				0x50-0x63[21-40路热敏电阻采集]--ADC位置[0x50-0x63]
				0x70--5VTM, 			   --ADC位置[1]
				0x80--12VTM,			   --ADC位置[2]
				0x90--12VTM*,   		   --ADC位置[3]
2.指令控制
  注入数据--子地址26-2个字--ADR高+ADR低+控制字
  注入数据--子地址24-8个字--阈值及预警值参数修改
  指令数据--子地址20-3个字--同步时间+指令(2a2a[复位];2b2b[加热器开];2c2c[加热器关];2d2d[全部加热器关];2e2e[加热自动巡检];1a1a[进入测试];1b1b[结束测试]
  自检结果--子地址10-24个字
  同步时间--子地址14-2个字
3.数据存储
使用1553B的RAM空间为数据RAM偏移地址:0x500-0xFDF
4.自动温度控制
根据预置温控策略进行加热控制

Created by Wangshw 20150822
Modified by Wangshw 20160603
******************************************/
#include "Global.h"

/*****************内部RAM存储地址表*******************************************/

/*****************函数映射表****************************************************/
void Exec_Loop(void);   			 /*循环执行模块*/
void ClearDog(void);				/*看门狗复位子模块*/
/*-------------初始化模块和其子模块----------------------------*/
void InitProgram(void); 			/*程序初始化子程序*/

extern void LoadArg(void);			/*装入参数子程序*/
extern void SaveArg(void);     		/*保存参数子程序*/

void WRITE_EXTERNRAM(unsigned int Offset, unsigned int Data);/*主程序中总线字写处理模块*/
unsigned int READ_EXTERNRAM(unsigned int Offset);	/*主程序中总线字读处理模块*/
/*-------------温控实现及功能检测模块------------------------- */
extern void TempCtrl_L1to6(unsigned char td);
extern void TempCtrl_L7(void);
extern void TempCtrl_L8(void);
extern void TempCtrl_L9(void);
extern void TempCtrl_L10(void);
extern void TempCtrl_L11(void);
char code reserve[8] _at_ 0x03;		//外部中断0--看门狗服务程序
/**************************************************************************
模块名称:	初始化和自检模块
模块描述:	1.对加热器状态进行初始设置;
			2.对RAM进行检测和清零;
			3.执行总线初始化设置;
			4.CPU状态初始化设置
扇入:		1-main()
输入参数:	无
输出参数:	无
返回值:		无
修改记录:	
其他说明:   	
**************************************************************************/ 
void InitProgram(void)
{
	unsigned char i,tChar;
	CommuniationDisable = 1;	//断开通讯链路
	ExternInt0_Enable = 0;		/*看门狗中断输入不允许*/
	AD_CONVST = 1;				/*AD不启动转换 */
	AD_RFS = 1;					/*设置数据帧不同步*/
	SCLK = 1;					/*AD时钟平时为高*/
	SDATA = 0;					/*AD不允许输入数据*/
	ICWRDisable = 1;			/*不允许读写I2C*/
	ICLK = 0;					/*I2C时钟平时为低*/
	ICDATA = 0;					/*I2C不允许输入数据*/
	/*--------------------加热器状态设置为关闭----------------------------*/ 
	pp[0] = 0;
	pp[1] = 0;
	pp[2] = 0;
	pp[3] = 0;
	pp[4] = 0;
	pp[5] = 0;
	XBYTE[ADDR_IOSEL1] = pp[0];  			 /*40加热器无输出*/ 
	XBYTE[ADDR_IOSEL2] = pp[1];  
	XBYTE[ADDR_IOSEL3] = pp[2];
	XBYTE[ADDR_IOSEL4] = pp[3];
	XBYTE[ADDR_IOSEL5] = pp[4];
	XBYTE[ADDR_IOSEL6] = pp[5];
	if(0x00 == (Status & 0x08)) 
	{
		for(tChar = RamFlag , RamStartAdr = XDATASTART , i = 0 ; i < XBLOCKTOTAL ; i++ , tChar >>= 1 , RamStartAdr + =XBLOCKLEN)
		{
			if((tChar & 0x01) == 0) break;
		}
	}
	else
	{
		RamStartAdr = 0;
		wkStatus = vaild_Status;
	}
	SendBuff.BuffAdr = RamStartAdr + SendBufStart;
	ReceiveBufStart.BuffAdr = RamStartAdr + ReceiveBufStart;
	if(HAVEDONE != RestoreFlag)//Tzh 代码中没有处理HAVEDONE的情况
	{
		if(0x00 == (Status & 0x08))
		{
			wkStatus = Idel_Status;
			WRITE_EXTERNRAM(AnC_Buf,(unsigned int)Base_Sensor);
			RetCode = 0xAA;
		}
		synTime.TA = 0;
	}
	TimerCount = CtrlPeriod -1 ; //进入中断开始采样
	ReceiveingFlag = 0;
	SendingFlag = 0;
	SentDone = 0;
	ctrlFlag = 0;
	LoadArg();
	RestoreFlag = HAVEDONE;        //Tzh RestoreFlag除HAVEDONE外，未置其它值？
	RestoreFlagB0 = HAVEDONE;
	RestoreFlagB1 = HAVEDONE;
}

/**************************************************************************
模块名称:	主程序模块
模块描述:	1.调用初始化模块对系统进行初始化;
			2.对加热自动测试以及温度控制的变量进行初始化；
			3.程序循环执行模块;
扇出:		3-InitProgram()/ClearDog()/Exec_Loop()；
扇入:		1-main()
输入参数:	无
输出参数:	无
返回值:		无
修改记录:	
其他说明:	
**************************************************************************/ 
void main(void)
{
	unsigned char i,tChar;
	IE = 0;	/*全局中断禁止*/ 
	TCON = 0;
	InitProgram();         /*程序初始化模块*/

	/*-----------------------------------------------------------------------*/ 
	TMOD = 0x21;					/*T1工作于方式2,T0工作方式1*/
	TH0 = 0xFF;						/*定时器启动后约17微秒进入定时中断*/
	TL0 = 0xF0; 		  
	IP = 3;							/*中断优先级:INTO(外部0)、INT1(时钟0)、INT4(串口)*/
	TR0 = 1;						/*启动ET0计数*/
	IE = 0x82;						/*允许ET0中断，开中断*/
	while(!ctrlFlag);				/*等待采集完一次数据*/ //Tzh 具体作用未知
	IE = 0;	/*全局中断禁止*/ 
 
	TH1 = 0xFD;						/*串口波特率9600*/
	TL1 = 0xFD;
	PCON = 0x7F;					/*波特率不加倍*/
	TR1 = 1;						/*启动ET1计数*/
	SCON = 0xD0;
	SM2 = 1;
	ClearDog();						/*看门狗复位*/
	ExternInt0_Enable = 1;			/*看门狗中断输入允许*/
	IT0 = 0;						/*中断(INT0)电平触发*/
	RI = 0;							/*清除串口中断标志*/
	TI = 0;
	IE = 0x93;						/*允许看门狗(EX0--INT0)中断、时钟(ET0--INT1)中断、串口中断(ES--INT4)，开中断*/
	while (1)
	{
		
		while(!ctrlFlag);
		switch(wkStatus)
		{
		case Auto_TempCtrl_Status:	/*温控程序循环执行模块*/
			Exec_wkLoop();
			break;
		case Test_Loop_Status:		/*寻检程序执行模块*/
			Exec_AutoProcess();
			Test_Loop_Status &= Test_Status;
			break;
		case Save_Arg_Status:		/*保存参数*/
			EA = 0;
			SaveArg();
			wkStatus = Idel_Status;
			EA = 1;
			break;
		default:
			break;
		}
		ClearDog(); 	  /*看门狗复位*/
	}
}

/**************************************************************************
模块名称:	循环执行模块
模块描述:	1.程序控制模块;
			2.传感器故障标志位清零;
			3.模拟量采集;
			4.当Mode_Flag为0时，进行自主温度控制;
			5.执行总线指令解析及执行;
			6.加热通道自动巡检子程序;
			7.进行自检结果数据组帧.
扇出:		5-TempCtrl_L1to6()/TempCtrl_L7()/TempCtrl_L8()/TempCtrl_L9()/TempCtrl_L10()/TempCtrl_L11()
扇入:		1-main()
输入参数:	无
输出参数:	无
返回值:		无
修改记录:	
其他说明:	
**************************************************************************/ 
void Exec_wkLoop(void)
{	
	unsigned char td; 	
	AncStandard = READ_EXTERNRAM(AnC_Buf);
	for(td = 0 ; td < 6 ; td ++)
	{
		if(ctrlFlag && (Auto_TempCtrl_Status == wkStatus))
		{
			TempCtrl_L1to6(td);
		}
		else
		{
			break;
		}
	}
	if(ctrlFlag && (Auto_TempCtrl_Status == wkStatus))
	{
		TempCtrl_L7();
	}
	if(ctrlFlag && (Auto_TempCtrl_Status == wkStatus))
	{
		TempCtrl_L8();
	}
	if(ctrlFlag && (Auto_TempCtrl_Status == wkStatus))
	{
		TempCtrl_L9();
	}
	if(ctrlFlag && (Auto_TempCtrl_Status == wkStatus))
	{
		TempCtrl_L10();
	}
	if(ctrlFlag && (Auto_TempCtrl_Status == wkStatus))
	{
		TempCtrl_L11();
	}
	if(ctrlFlag && (Auto_TempCtrl_Status == wkStatus))
	{
		XBYTE[ADDR_IOSEL1] = pp[0];                          /* ADDR_IOSEL1 加热控制输出 */
		XBYTE[ADDR_IOSEL2] = pp[1];                          /* ADDR_IOSEL2 加热控制输出 */
		XBYTE[ADDR_IOSEL3] = pp[2];                          /* ADDR_IOSEL3 加热控制输出 */
		XBYTE[ADDR_IOSEL4] = pp[3];                          /* ADDR_IOSEL4 加热控制输出 */
		XBYTE[ADDR_IOSEL5] = pp[4];                          /* ADDR_IOSEL5 加热控制输出 */
		XBYTE[ADDR_IOSEL6] = pp[5];                          /* ADDR_IOSEL6 加热控制输出 */
	}
}


/*************************************************************************
模块名称: 加热器自检循环函数程序
模块描述: 1.对40路加热通道进行循环检测
		  2.每个加热通道打开时间1s,关闭。如果退出测试，停止循检
扇入:	  0-无	
输出参数: 无	
返回值:	  无	
修改记录:	
其他说明:	
**************************************************************************/
void Exec_AutoProcess(void)
{
	unsigned int i;
	XBYTE[ADDR_IOSEL1] = 0x00;  			 /*40加热器无输出*/ 
	XBYTE[ADDR_IOSEL2] = 0x00;  
	XBYTE[ADDR_IOSEL3] = 0x00;
	XBYTE[ADDR_IOSEL4] = 0x00;
	XBYTE[ADDR_IOSEL5] = 0x00;
	XBYTE[ADDR_IOSEL6] = 0x00;
	/*由于1秒下降沿有延迟,调整为1秒上升沿开始*/
	while(ctrlFlag);		//等到下降沿
	while(!ctrlFlag);		//等到上升沿
	for(i = 0 ; i < 40 ; i++)
	{
		XBYTE[HEAT_ADR[i]] = OPEN_CMD[i];
		ClearDog(); 	  /*可能等待近1秒,看门狗复位*/
		while(ctrlFlag);
		while(!ctrlFlag);
		XBYTE[HEAT_ADR[i]] = 0;
		if(Test_Loop_Status != wkStatus)	break;
	}
}


/*******************************************************************************
模块名称:  写处理子模块
模块描述:  总线字写处理
扇出:	   0-无	
输出参数:  无	
返回值:	   无	
修改记录:	
其他说明:	
*******************************************************************************/
void WRITE_EXTERNRAM(unsigned int Offset, unsigned int Data)
{
	XBYTE[RamStartAdr + (Offset * 2)] = (unsigned char) Data;		/*写低字节 */
	XBYTE[RamStartAdr + (Offset * 2) + 1] = (unsigned char) (Data >> 8);/*写高字节 */
}
/*******************************************************************************
模块名称:  读处理子模块
模块描述:  总线字读处理
扇出:	   0-	
扇入:	   3-BusInitial/BusRamTest/CheckBusStatus	
输入参数:  Offset	
输出参数:  无	
返回值:	   无	
修改记录:	
其他说明:	
*******************************************************************************/
unsigned int READ_EXTERNRAM(unsigned int Offset)
{
	unsigned char LByte;
	unsigned char HByte;
	HByte = XBYTE[RamStartAdr + (Offset * 2) + 1]; /*读出高字节*/
	LByte = XBYTE[RamStartAdr + (Offset * 2)];	 /*读出低字节*/
	return (unsigned int) ((HByte << 8) + LByte);	/*返回读出的字*/
}
/**************************************************************************
模块名称: 看门狗复位子模块
模块描述: 将看门狗计数器复位；
扇出:	  1_nop_() 
扇入:	  1)/main()/
输入参数: 无			
输出参数: 无	
返回值:	  无	
修改记录: ；	
其他说明:
**************************************************************************/
void ClearDog(void)
{
	ADDR_DOG = 0;		/*看门狗复位*/
	_nop_();			/*延时,至少保持2个指令周期*/
	_nop_();			
	ADDR_DOG = 1;		/*允许看门狗计时、产生中断*/
}
