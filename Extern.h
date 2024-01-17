/****************************************
Created	by lgl	at 20181201
Modified	by	lgl	at 20181230
******************************************/
#include <reg51.h>
#include "Define.h"
#ifndef __Extern_H__
#define __Extern_H__
/*****************P口定义****************************************************/
extern sbit ExternInt0_Enable = P3^2;				/*外部中断(看门狗)输入*/
extern sbit AD_CONVST = P1 ^ 2;					/*AD转换启动[0->1－AD启动转换,1-转换过程中]*/
extern sbit AD_RFS = P1 ^ 3;						/*AD数据读启动[0-帧同步]*/
extern sbit SCLK = P1 ^ 4;							/*AD读取串行数据时钟SCLK*/
extern sbit ICLK = P1 ^ 5;							/*I2C时钟*/
extern sbit ICWRDisable = P1 ^ 6;					/*I2C不允许读写*/
extern sbit ADDR_DOG = P1 ^ 7;						/*看门狗控制*/
extern sbit SDATA = P3 ^ 4;						/*AD读取的串行SDATA*/
extern sbit ICDATA = P3 ^ 5;						/*I2C数据位*/
extern sbit CommuniationDisable = P3 ^ 7;			/*串性通讯链路断开*/	
/********************************自定义变量**********************************/
extern unsigned char idata Status _at_ 0x20;
extern unsigned char idata RetCode _at_ 0x21;
extern unsigned char idata RamFlag _at_ 0x22;
extern unsigned char idata wkStatus _at_ 0x23;
extern unsigned int idata TimerState_Cont _at_ 0x24;
extern unsigned char idata pp[6] _at_ 0x26 = {0, 0, 0, 0, 0, 0};  /*控制指令输出状态记录       */
extern unsigned char bdata iBase _at_ 0x2c;
extern unsigned char idata RestoreFlag _at_ 0x2d;
extern unsigned char idata RestoreFlagB0 _at_ 0x2e;
extern unsigned char idata RestoreFlagB1 _at_ 0x2f;
extern unsigned int idata RamStartAdr _at_ 0x30;	/*可用内存起始地址*/
extern unsigned int idata TimerCount _at_ 0x32;       /* 定时器计数累加值                 */
extern unsigned char idata ReceiveCount _at_0x34;
extern unsigned char idata SendCount _at_0x35;
extern unsigned int idata AncMax _at_ 0x36;
extern unsigned int idata AncMin _at_ 0x38;
extern unsigned int idata AncStandard _at_ 0x3a;
extern unsigned TempleStr idata AncTotal _at_ 0x3c;

extern rgStruct idata Argument _at_ 0x40;
extern TimeCount idata synTime _at_ 0x4C;

extern SeriBuff idata SendBuff _at_ 0x50;
extern SeriBuff idata ReceiveBuff _at_ 0x58;

extern sbit ReceiveingFlag = iBase ^ 0;
extern sbit SendingFlag = iBase ^ 1;
extern sbit SentDone = iBase ^ 2;
extern sbit BoardcastFlag = iBase ^ 3;
extern sbit rCmdErr = iBase ^ 4;
extern sbit HeatState = iBase ^ 5;
extern sbit ctrlFlag = iBase ^ 7;
#endif
