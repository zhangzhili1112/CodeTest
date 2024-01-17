/***************************
Created by Wangshw 20150822
Modified by Wangshw 20160603
***************************/
#include "Extern.h"
extern unsigned int READ_EXTERNRAM(unsigned int Offset);	/*主程序中总线字读处理模块*/
void TempCtrl_L1to6(unsigned char td);
void TempCtrl_L7(void);
void TempCtrl_L8(void);
void TempCtrl_L9(void);
void TempCtrl_L10(void);
void TempCtrl_L11(void);
/**************************************************************************
模块名称:	正常温控策略实现模块
模块描述:	1.将采集温度值进行判断和比较;
			2.实现温度控制策略，输出加热控制指令;
			3.对故障进行判断，对自检结果进行赋值。
输入参数:	无
输出参数:	P[0~5]
返回值:		无
修改记录:	20150822创建，对应任务书要求实现24路测温对23路加热的控制
            20160603修改，对应摸底状态任务书要求实现24路测温对27路加热的控制
其他说明:	
**************************************************************************/
/*********回路1:T1,T2;H1,H2*************/  
void TempCtrl_L1to6(unsigned char td)
{
	unsigned int Temp_value1, Temp_value2, T_max, T_min;

	/* 传感器故障判断,求回路最大、最小温度 */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM( ptHL_T1_Cmd[td][0]); 	/*   T01,T07,T21,T16,T04,T10  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value1 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* 校正系数计算 512对应5V  */

	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM( ptHL_T1_Cmd[td][0] + 1); 	/*   T02,T08,T22,T17,T05,T11  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value2 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* 校正系数计算 512对应5V  */

	HeatState = 1;                  /* 存在正确回路 */
	if((Temp_value1 > Sensor_Erk) || (Temp_value1 < Sensor_Erd))
	{
		if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))		/* 回路均开路或短路:控温策略1 */
		{
			HeatState = 0;                  
			RetCode = 0x55;           /* 温控器状态编码赋值 0x55*/
			pp[ptHL_T1_Cmd[td][1]] &= (unsigned char) ~ptHL_T1_Cmd[td][2];	/* 关闭加热*/
			pp[ptHL_T1_Cmd[td][3]] &= (unsigned char) ~ptHL_T1_Cmd[td][4];	/* 关闭加热*/
		}
		else	/* 回路异常:存在开路或短路,控温策略2 */
		{
			T_max = Temp_value2;   /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
			T_min = Temp_value2; 
			if(RetCode > 0x80)
			{
				RetCode = 0x80;		/* 回路中有传感器故障温控器状态编码赋值 */
			}
		}
	}
	else if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))	/* 回路异常:存在开路或短路,控温策略2 */
	{
		T_max = Temp_value1;	/* 温度最值判断 负温度系数,T_max为模拟量极小值 */
		T_min = Temp_value1; 
		if(RetCode > 0x80)
		{
			RetCode = 0x80;		/* 回路中有传感器故障温控器状态编码赋值 */
		}
	}
	else		/* 回路正常:不存在开路或短路,控温策略3~11 */
	{
		T_max = (Temp_value1 < Temp_value2) ? Temp_value1 : Temp_value2;   /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
		T_min = (Temp_value1 > Temp_value2) ? Temp_value1 : Temp_value2; 
	}

	/* 加热控制   加热1 加热2  */
	if(HeatState)	/* 判断加热回路状态 */
	{
		if((T_max < Argument.HT))		/* 温度超过高温故障限,控温策略4 */
		{
			RetCode = 0x55;				/* 温控器状态编码赋值0x55 */
			pp[ptHL_T1_Cmd[td][1]] &= (unsigned char) ~ptHL_T1_Cmd[td][2];	/* 关闭加热*/
			pp[ptHL_T1_Cmd[td][3]] &= (unsigned char) ~ptHL_T1_Cmd[td][4];	/* 关闭加热*/
		}
		else	/* 温度不超过高温故障限*/
		{
			if(T_min >= Argument.LDT)		/* 温度低于动作下限 */
			{
				pp[ptHL_T1_Cmd[td][1]] |= (unsigned char) ptHL_T1_Cmd[td][2];	/* 打开加热 */
				pp[ptHL_T1_Cmd[td][3]] |= (unsigned char) ptHL_T1_Cmd[td][4];	/* 打开加热*/
			
				if(T_min > Argument.LT)		/* 温度低于低温故障限*/
				{	
					if (TimerState_Cont > 600)
					{
						RetCode = 0x55;    /*开机超过10分钟内:上报错误,控温策略4 */	
					}
				}
				else if (T_max <= Argument.HDT)		/* 低温度低于动作下限、高温高于动作上限:控温策略6*/
				{
					if(RetCode > 0x60)
					{
						RetCode = 0x60;    /* 处于高低温异常状态，自检结果赋值 */	
					}
				}
				else
				{
				    ;    /* 无需处理:控温策略5*/
				}	
			}
			else	/* 温度高于动作下限 */
			{	
				if(T_max <= Argument.HDT)	/* 温度高于动作上限:控温策略8 */
				{	
					pp[ptHL_T1_Cmd[td][1]] &= (unsigned char) ~ptHL_T1_Cmd[td][2];	/* 关闭加热*/
					pp[ptHL_T1_Cmd[td][3]] &= (unsigned char) ~ptHL_T1_Cmd[td][4];	/* 关闭加热*/
				}
				// /* 温度低于动作上限:回路状态不变,控温策略7 */
			}
		}
	}
}

void TempCtrl_L7(void)
{
	unsigned int Temp_value1, Temp_value2, Temp_value3, T_max, T_min;
	/* T3,T9,T13 控控制 H5,H6(加热5，加热6)*/
	/* 传感器异常判断 */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x12); 	/*   T03  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value1 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* 校正系数计算 512对应5V  */

	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x18); 	/*   T09  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value2 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* 校正系数计算 512对应5V  */

	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x1c); 	/*   T13  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value3 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* 校正系数计算 512对应5V  */

	HeatState = 1;                  /* 存在正确回路 */
	if((Temp_value1 > Sensor_Erk) || (Temp_value1 < Sensor_Erd))
	{
		if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
		{
			if((Temp_value3 > Sensor_Erk) || (Temp_value3 < Sensor_Erd))
			{
				HeatState = 0;		/* 回路异常 */
				RetCode = 0x55;		/* 温控器状态编码赋值 */
				pp[2] &= (~0x06);	/* 关闭加热 加热5，加热6*/                                  
			}
			else
			{
				T_max = Temp_value3 ;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
				T_min = Temp_value3 ; 
			}
		}
		else if((Temp_value3 > Sensor_Erk) || (Temp_value3 < Sensor_Erd))
		{
			T_max = Temp_value2 ;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
			T_min = Temp_value2 ; 
		}
		else
		{
			T_max = (Temp_value2 < Temp_value3) ? Temp_value2 : Temp_value3;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
			T_min = (Temp_value2> Temp_value3) ? Temp_value2 : Temp_value3; 
		}
		if(RetCode > 0x80)
		{
			RetCode = 0x80;        /* 回路中有传感器故障温控器状态编码赋值 */
		}
	}
	else if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
	{
		if((Temp_value3 > Sensor_Erk) || (Temp_value3 < Sensor_Erd))
		{
			T_max = Temp_value1 ;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
			T_min = Temp_value1 ; 
		}
		else
		{
			T_max = (Temp_value1 < Temp_value3) ? Temp_value1 : Temp_value3;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
			T_min = (Temp_value1 > Temp_value3) ? Temp_value1 : Temp_value3; 
			if(RetCode > 0x80)
			{
				RetCode = 0x80;     /* 回路中有传感器故障温控器状态编码赋值 */
			}
		}
	}
	else if(Temp_value3 > Sensor_Erk)
	{
		T_max = (Temp_value1 < Temp_value2) ? Temp_value1 : Temp_value2;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
		T_min = (Temp_value1 > Temp_value2) ? Temp_value1 : Temp_value2; 
		if(RetCode > 0x80)
		{
			RetCode = 0x80;     /* 回路中有传感器故障温控器状态编码赋值 */
		}
	}
	else
	{
		T_max = (Temp_value1 < Temp_value2) ? Temp_value1 : Temp_value2;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
		T_min = (Temp_value1 > Temp_value2) ? Temp_value1 : Temp_value2; 

		T_max = (T_max < Temp_value3) ? T_max : Temp_value3;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
		T_min = (T_min > Temp_value3) ? T_min : Temp_value3; 
	}
	
	/* 加热控制  加热5，加热6 */
	if(HeatState)	/* 判断加热回路状态 */
	{
		if((T_max < Argument.HT))	/* 温度超过高温故障限 */
		{
			RetCode = 0x55;			/* 温控器状态编码赋值 */
			pp[2] &= (~0x06);		/* 关闭加热 加热5，加热6*/
		}
		else
		{
			if(T_min >= Argument.LDT)	/* 温度低于动作下限 */
			{
				pp[2] |= 0x06;			/* 打开加热 加热5，加热6*/	

				if(T_min > Argument.LT)	 /* 开机10分钟内,不上报错误*/
				{	
					if(TimerState_Cont > 600)
					{
						RetCode = 0x55;    /* 温度低于低温故障限*/	
					}
				}
				else if (T_max <= Argument.HDT)
				{
					if(RetCode > 0x60)
					{
						RetCode = 0x60;    /* 处于高低温异常状态，自检结果赋值 */	
					}
				}
				else
				{
				    ;    /* 无需处理 */
				}	
			}
			else
			{	
				if(T_max <= Argument.HDT)	/* 温度高于动作上限 */
				{	
					pp[2] &= (~0x06);		/* 关闭加热 加热5，加热6*/
				}
			}
		}
	}
}
void TempCtrl_L8(void)
{
	unsigned int Temp_value1, Temp_value2,  T_max, T_min;
	/* T15,T20,控控制 H37(加热37)*/
	/* 传感器异常判断 */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x1e); 	/*   T15  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value1 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* 校正系数计算 512对应5V  */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x23); 	/*   T20  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value2 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* 校正系数计算 512对应5V  */
	HeatState = 1;                  /* 存在正确回路 */
	if((Temp_value1 > Sensor_Erk) || (Temp_value1 < Sensor_Erd))
	{
		if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
		{
				HeatState = 0;			/* 回路异常 */
				RetCode = 0x55;			/* 温控器状态编码赋值 */
				pp[3] &= (~0x10);		/* 关闭加热 加热37*/                                  
		}
		else
		{
			T_max = Temp_value2;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
			T_min = Temp_value2; 
			if(RetCode > 0x80)
			{
				RetCode = 0x80;        /* 回路中有传感器故障温控器状态编码赋值 */
			}
		}
	}
	else if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
	{
		T_max = Temp_value1;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
		T_min = Temp_value1; 
		if(RetCode > 0x80)
		{
			RetCode = 0x80;     /* 回路中有传感器故障温控器状态编码赋值 */
		}
	}
	else
	{
		T_max = (Temp_value1 < Temp_value2) ? Temp_value1 : Temp_value2;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
		T_min = (Temp_value1 > Temp_value2) ? Temp_value1 : Temp_value2; 
	}
	
	/* 加热控制  加热5，加热6 */
	if(HeatState)	/* 判断加热回路状态 */
	{
		
		if((T_max < Argument.HT))		/* 温度超过高温故障限 */
		{
			RetCode = 0x55;				/* 温控器状态编码赋值 */
			pp[3] &= (~0x10);			/* 关闭加热 加热37*/
		}
		else
		{
			if(T_min >= Argument.LDT)		/* 温度低于动作下限 */
			{
				pp[3] |= 0x10;				/* 打开加热 加热5，加热6*/	

				if(T_min > Argument.QLT)	 /* 开机10分钟内,不上报错误*/
				{
					if(TimerState_Cont > 600)
					{
						RetCode = 0x55;    /* 温度低于低温故障限*/	
					}
				}
				else if (T_max <= Argument.HDT)
				{
					if(RetCode > 0x60)
					{
						RetCode = 0x60;    /* 处于高低温异常状态，自检结果赋值 */	
					}
				}
				else
				{
				    ;    /* 无需处理 */
				}	
			}
			else
			{	
				if(T_max <= Argument.HDT)	/* 温度高于动作上限 */
				{	
					pp[3] &= (~0x10);		/* 关闭加热 加热37*/
				}
			}
		}
	}
}
void TempCtrl_L9(void)
{
	unsigned int Temp_value1, Temp_value2,  T_max, T_min;
	/* T14,T18,控控制 H23、H24、H25、H26、H33、H34、H35、H36(加热23、24、25、26、33、34、35、36)*/
	/* 传感器异常判断 */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x1d); 	/*   T15  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value1 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* 校正系数计算 512对应5V  */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x21); 	/*   T20  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value2 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* 校正系数计算 512对应5V  */
	HeatState = 1;                  /* 存在正确回路 */
	if((Temp_value1 > Sensor_Erk) || (Temp_value1 < Sensor_Erd))
	{
		if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
		{
			HeatState = 0;				/* 回路异常 */
			RetCode = 0x55;				/* 温控器状态编码赋值 */
			pp[5] &= (~0x1e);			/* 关闭加热 加热23,加热24,加热25,加热26  */
			pp[4] &= (~0x02);			/* 关闭加热 加热33 */
			pp[3] &= (~0xe0);			/* 关闭加热 加热34,加热35，加热36 */
		}
		else
		{
			T_max = Temp_value2;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
			T_min = Temp_value2; 
			if(RetCode > 0x80)
			{
				RetCode = 0x80;        /* 回路中有传感器故障温控器状态编码赋值 */
			}
		}
	}
	else if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
	{
		T_max = Temp_value1;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
		T_min = Temp_value1; 
		if(RetCode > 0x80)
		{
			RetCode = 0x80;     /* 回路中有传感器故障温控器状态编码赋值 */
		}
	}
	else
	{
		T_max = (Temp_value1 < Temp_value2) ? Temp_value1 : Temp_value2;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
		T_min = (Temp_value1 > Temp_value2) ? Temp_value1 : Temp_value2; 
	}
	
	/* 加热控制 */
	if(HeatState)	/* 判断加热回路状态 */
	{
		
		if((T_max < Argument.HT))		/* 温度超过高温故障限 */
		{
			RetCode = 0x55;				/* 温控器状态编码赋值 */
			pp[5] &= (~0x1e);			/* 关闭加热 加热23,加热24,加热25,加热26  */
			pp[4] &= (~0x02);			/* 关闭加热 加热33 */
			pp[3] &= (~0xe0);			/* 关闭加热 加热34,加热35，加热36 */
		}
		else
		{

			if(T_min >= Argument.CLDT)	/* 温度低于动作下限 */
			{
				pp[5] |= 0x1e;			/* 加热 加热23,加热24,加热25,加热26  */
				pp[4] |= 0x02;			/* 加热 加热33 */
				pp[3] |= 0xe0;			/* 加热 加热34,加热35，加热36 */

				if(T_min > Argument.LT)	 /* 开机10分钟内,不上报错误*/
				{
					if(TimerState_Cont > 600)
					{
						RetCode = 0x55;    /* 温度低于低温故障限*/	
					}
				}
				else if (T_max <= Argument.HDT)
				{
					if(RetCode > 0x60)
					{
						RetCode = 0x60;    /* 处于高低温异常状态，自检结果赋值 */	
					}
				}
				else
				{
				    ;    /* 无需处理 */
				}	
			}
			else
			{	
				if(T_max <= Argument.HDT)		/* 温度高于动作上限 */
				{	
					pp[5] &= (~0x1e);			/* 关闭加热 加热23,加热24,加热25,加热26  */
					pp[4] &= (~0x02);			/* 关闭加热 加热33 */
					pp[3] &= (~0xe0);			/* 关闭加热 加热34,加热35，加热36 */
				}
			}
		}
	}
}

void TempCtrl_L10(void)
{
	unsigned int Temp_value1, Temp_value2, Temp_value3, T_max, T_min;
	/* T6,T12,T24 控控制 H15,H18(加热15，加热18)*/
	/* 传感器异常判断 */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x15); 	/*   T06  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value1 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* 校正系数计算 512对应5V  */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x1b); 	/*   T12  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value2 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* 校正系数计算 512对应5V  */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x27); 	/*   T24  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value3 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* 校正系数计算 512对应5V  */
	HeatState = 1;                  /* 存在正确回路 */
	if((Temp_value1 > Sensor_Erk) || (Temp_value1 < Sensor_Erd))
	{
		if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
		{
			if((Temp_value3 > Sensor_Erk) || (Temp_value3 < Sensor_Erd))
			{
				HeatState = 0;			/* 回路异常 */
				RetCode = 0x55;			/* 温控器状态编码赋值 */
				pp[0] &= (~0x48);		/* 关闭加热 加热15，加热18*/                                  
			}
			else
			{
				T_max = Temp_value3;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
				T_min = Temp_value3; 
			}
		}
		else if((Temp_value3 > Sensor_Erk) || (Temp_value3 < Sensor_Erd))
		{
			T_max = Temp_value2;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
			T_min = Temp_value2; 
		}
		else
		{
			Temp_value1 = Temp_value2;		/* 异常传感器数据处理 */
		}
		if(RetCode > 0x80)
		{
			RetCode = 0x80;        /* 回路中有传感器故障温控器状态编码赋值 */
		}
	}
	else if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
	{
		if((Temp_value3 > Sensor_Erk) || (Temp_value3 < Sensor_Erd))
		{
			T_max = Temp_value1;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
			T_min = Temp_value1; 
		}
		else
		{
			T_max = (Temp_value1 < Temp_value3) ? Temp_value1 : Temp_value3;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
			T_min = (Temp_value1 > Temp_value3) ? Temp_value1 : Temp_value3; 
			if(RetCode > 0x80)
			{
				RetCode = 0x80;     /* 回路中有传感器故障温控器状态编码赋值 */
			}
		}
	}
	else if(Temp_value3 > Sensor_Erk)
	{
		T_max = (Temp_value1 < Temp_value2) ? Temp_value1 : Temp_value2;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
		T_min = (Temp_value1 > Temp_value2) ? Temp_value1 : Temp_value2; 
		if(RetCode > 0x80)
		{
			RetCode = 0x80;     /* 回路中有传感器故障温控器状态编码赋值 */
		}
	}
	else
	{
		T_max = (Temp_value1 < Temp_value2) ? Temp_value1 : Temp_value2;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
		T_min = (Temp_value1 > Temp_value2) ? Temp_value1 : Temp_value2; 

		T_max = (T_max < Temp_value3) ? T_max : Temp_value3;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
		T_min = (T_min > Temp_value3) ? T_min : Temp_value3; 
	}
	
	/* 加热控制  加热15，加热18 */
	if(HeatState)	/* 判断加热回路状态 */
	{
		
		if((T_max < Argument.HT))		/* 温度超过高温故障限 */
		{
			RetCode = 0x55;				/* 温控器状态编码赋值 */
			pp[0] &= (~0x48);			/* 关闭加热 加热15，加热18*/
		}
		else
		{
			if(T_min >= Argument.LDT)		/* 温度低于动作下限 */
			{
				pp[0] |= 0x48;				/* 打开加热 加热15，加热18*/	

				if(T_min > Argument.LT)
				{	
					if(TimerState_Cont > 600)
					{
						RetCode = 0x55;    /* 温度低于低温故障限*/	
					}
				}
				else if (T_max <= Argument.HDT)
				{
					if(RetCode > 0x60)
					{
						RetCode = 0x60;    /* 处于高低温异常状态，自检结果赋值 */
					}
				}
				else
				{
				    ;    /* 无需处理 */
				}	
			}
			else
			{	
				if(T_max <= Argument.HDT)		/* 温度高于动作上限 */
				{	
					pp[0] &= (~0x48);			/* 关闭加热 加热15，加热18*/
				}
			}
		}
	}
}
void TempCtrl_L11(void)
{
	unsigned int Temp_value1, Temp_value2,  T_max, T_min;
	/* T19,T23,控控制 H16、 H17(加热16、加热17)*/
	/* 传感器异常判断 */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x22); 	/*   T19  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value1 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* 校正系数计算 512对应5V  */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x26); 	/*   T23  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value2 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* 校正系数计算 512对应5V  */
	HeatState = 1;                  /* 存在正确回路 */
	if((Temp_value1 > Sensor_Erk) || (Temp_value1 < Sensor_Erd))
	{
		if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
		{
				HeatState = 0;                  /* 回路异常 */
				RetCode = 0x55;           /* 温控器状态编码赋值 */
				pp[0] &= (~0x30);                                           /* 关闭加热 加热16、加热17*/                                  
		}
		else
		{
			T_max = Temp_value2;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
			T_min = Temp_value2; 
			if(RetCode > 0x80)
			{
				RetCode = 0x80;        /* 回路中有传感器故障温控器状态编码赋值 */
			}
		}
	}
	else if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
	{
		T_max = Temp_value1;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
		T_min = Temp_value1; 
		if(RetCode > 0x80)
		{
			RetCode = 0x80;     /* 回路中有传感器故障温控器状态编码赋值 */
		}
	}
	else
	{
		T_max = (Temp_value1 < Temp_value2) ? Temp_value1 : Temp_value2;	  /* 温度最值判断 负温度系数,T_max为模拟量极小值 */
		T_min = (Temp_value1 > Temp_value2) ? Temp_value1 : Temp_value2; 
	}
	
	/* 加热控制  加热5，加热6 */
	if(HeatState)	/* 判断加热回路状态 */
	{
		
		if((T_max < Argument.HT))                                      /* 温度超过高温故障限 */
		{
			RetCode = 0x55;           /* 温控器状态编码赋值 */
			pp[0] &= (~0x30);                                           /* 关闭加热 加热16、加热17*/
		}
		else
		{
			if(T_min >= Argument.LDT)                                    /* 温度低于动作下限 */
			{
				pp[0] |= 0x30;                                          /* 打开加热 加热16、加热17*/	

				if(T_min > Argument.QLT)	 /* 开机10分钟内,不上报错误*/
				{
					if(TimerState_Cont > 600)
					{
						RetCode = 0x55;    /* 温度低于低温故障限*/	
					}
				}
				else if (T_max <= Argument.HDT)
				{
					if(RetCode > 0x60)
					{
						RetCode = 0x60;    /* 处于高低温异常状态，自检结果赋值 */	
					}
				}
				else
				{
				    ;    /* 无需处理 */
				}	
			}
			else
			{	
				if(T_max <= Argument.HDT)                               /* 温度高于动作上限 */
				{	
					pp[0] &= (~0x30);                                   /* 关闭加热 加热16、加热17*/
				}
			}
		}
	}
}
