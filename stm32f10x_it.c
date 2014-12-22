/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "function.h"
#include "motion.h"
#include "i2c_ee.h"
#include "lcd.h"
#define ABS(A) (((A)>0)?(A):(-(A)))
u8	status,flag_run=0,upperStop,upperCount;
u8	CfTimes=0;
u8	m_bRequestCf=0;
u8	flickerTimes,keypressTime;
u8	checkChannel1;
const unsigned int Speedup_moto[162]=
{2048,2033,2025,2017,2007,1996,1983,1970,1956,1940,1924,1907,1889,1870,1851,1832,1811,1791,1770,1749,1728,1706,1685,1663,1642,1620,1599,1577,1556,1535,1514,1493,1472,1452,1432,1412,1392,1373,1354,1335,1316,1298,1280,1262,1245,1227,1210,1194,1177,1161,1145,1129,1114,1099,1084,1069,1054,1040,1026,1012,998,985,971,958,945,933,920,907,895,883,871,859,847,835,823,812,800,789,777,766,755,743,732,720,709,697,687,677,668,661,654,649,644,639,634,630,627,623,620,617,614,611,608,606,603,601,599,597,595,593,591,589,588,586,585,583,582,580,579,578,577,575,574,573,572,571,570,569,569,568,567,566,565,565,564,563,563,562,562,561,561,560,560,559,559,558,558,558,557,557,557,557,556,556,556,556,556,556,556,556,556,556};
const unsigned int moto_ac[147]=
	{2048,2047,2047,2046,2045,2044,2042,2040,2038,2035,2032,2029,2026,2022,2018,2014,2010,2005,2000,1995,1990,1984,1978,1972,1966,1959,1952,1946,1939,1931,1924,1916,1909,1901,1893,1885,1877,1868,1860,1852,1843,1834,1825,1817,1808,1799,1790,1781,1772,1763,1753,1744,1735,1726,1716,1707,1698,1689,1679,1670,1661,1652,1643,1633,1624,1615,1606,1597,1588,1579,1570,1561,1552,1543,1535,1526,1517,1508,1500,1491,1483,1474,1466,1458,1449,1441,1433,1425,1417,1409,1401,1393,1385,1377,1370,1362,1355,1347,1340,1332,1325,1318,1310,1303,1296,1289,1282,1275,1268,1262,1258,1252,1246,1240,1235,1229,1224,1220,1215,1211,1207,1203,1199,1195,1192,1189,1186,1183,1180,1178,1176,1173,1171,1169,1168,1166,1165,1163,1162,1161,1160,1159,1159,1158,1158,1158,1157};

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void TIM6_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
}

//切光片电机使用TIM1中断
void TIM1_UP_IRQHandler(void)
{
	if(Step_LM<9000)	//加速过程
	{
		//清除中断标志
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		TIM1->ARR=(Speedup[(u8)((Step_LM++)/45)]<<1)-1;
	}
	else
	{					
		//转速达到启动PWM
		TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
		TIM1->ARR=3071;
		TIM1->CCER=0x000B;
		TIM1->CNT=0x08B3;
	}	 	
}
/*******************************************************************************
* Function Name  : NMIException
* Description    : This function handles NMI exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NMIException(void)
{
}

/*******************************************************************************
* Function Name  : HardFaultException
* Description    : This function handles Hard Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HardFaultException(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
		return;
	}
}

/*******************************************************************************
* Function Name  : MemManageException
* Description    : This function handles Memory Manage exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MemManageException(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/*******************************************************************************
* Function Name  : BusFaultException
* Description    : This function handles Bus Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BusFaultException(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/*******************************************************************************
* Function Name  : UsageFaultException
* Description    : This function handles Usage Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsageFaultException(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/*******************************************************************************
* Function Name  : DebugMonitor
* Description    : This function handles Debug Monitor exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DebugMonitor(void)
{
}

/*******************************************************************************
* Function Name  : SVCHandler
* Description    : This function handles SVCall exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVCHandler(void)
{
}

/*******************************************************************************
* Function Name  : PendSVC
* Description    : This function handles PendSVC exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PendSVC(void)
{
}

/*******************************************************************************
* Function Name  : SysTickHandler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTickHandler(void)
{
	if(TimingDelay>0)
		TimingDelay--;
}

/*******************************************************************************
* Function Name  : WWDG_IRQHandler
* Description    : This function handles WWDG interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WWDG_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : PVD_IRQHandler
* Description    : This function handles PVD interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PVD_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TAMPER_IRQHandler
* Description    : This function handles Tamper interrupt request. 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TAMPER_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : RTC_IRQHandler
* Description    : This function handles RTC global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : FLASH_IRQHandler
* Description    : This function handles Flash interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : RCC_IRQHandler
* Description    : This function handles RCC interrupt request. 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : EXTI0_IRQHandler
* Description    : This function handles External interrupt Line 0 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI0_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : EXTI1_IRQHandler
* Description    : This function handles External interrupt Line 1 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI1_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : EXTI2_IRQHandler
* Description    : This function handles External interrupt Line 2 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI2_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : EXTI3_IRQHandler
* Description    : This function handles External interrupt Line 3 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : EXTI4_IRQHandler
* Description    : This function handles External interrupt Line 4 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI4_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel1_IRQHandler
* Description    : This function handles DMA1 Channel 1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel2_IRQHandler
* Description    : This function handles DMA1 Channel 2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel2_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel3_IRQHandler
* Description    : This function handles DMA1 Channel 3 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel4_IRQHandler
* Description    : This function handles DMA1 Channel 4 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel5_IRQHandler
* Description    : This function handles DMA1 Channel 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel5_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel6_IRQHandler
* Description    : This function handles DMA1 Channel 6 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel6_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA1_Channel7_IRQHandler
* Description    : This function handles DMA1 Channel 7 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : ADC1_2_IRQHandler
* Description    : This function handles ADC1 and ADC2 global interrupts requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC1_2_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : USB_HP_CAN_TX_IRQHandler
* Description    : This function handles USB High Priority or CAN TX interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_HP_CAN_TX_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : USB_LP_CAN_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN_RX0_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : CAN_RX1_IRQHandler
* Description    : This function handles CAN RX1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN_RX1_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : CAN_SCE_IRQHandler
* Description    : This function handles CAN SCE interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN_SCE_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : EXTI9_5_IRQHandler
* Description    : This function handles External lines 9 to 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI9_5_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM1_BRK_IRQHandler
* Description    : This function handles TIM1 Break interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_BRK_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM1_TRG_COM_IRQHandler
* Description    : This function handles TIM1 Trigger and commutation interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_TRG_COM_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM1_CC_IRQHandler
* Description    : This function handles TIM1 capture compare interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_CC_IRQHandler(void)
{
}

/*******************************************************************************
Timer2 用于汽缸电机中断
*******************************************************************************/
void TIM2_IRQHandler(void)
{ 
	static u8 pulsetype;
	if(TIM_GetFlagStatus(TIM2,TIM_FLAG_Update)!=RESET)
	{
		GPIO_WriteBit(GPIO_M1CL, GPIO_Pin_M1CL,(BitAction)pulsetype);
		pulsetype=~pulsetype;
		
		//如果脉冲够数
		if(PulseCount>=PulseNum+AddedNum)
		{
			//关闭中断及timer2
			TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM2,DISABLE);
		}
		//加压有加减速
		if(flagPress)
		{
			//如果出于匀速运行阶段
			if(PulseCount>294&&PulseCount<=PulseNum-294)
				TIM2->ARR=1157;
			//如果处于加速阶段
			else if(PulseCount<294)
				TIM2->ARR=moto_ac[PulseCount/2];
			//如果出于减速阶段
			else if((PulseNum-PulseCount)<294&&(PulseNum-PulseCount)>0)
				TIM2->ARR=moto_ac[(PulseNum-PulseCount)/2];
			//如果是AddedNum 部分，最低速度运行
			else
				TIM2->ARR=moto_ac[0];				
		}
		else
		{	//匀速
			if(StartSpeed==TargeSpeed)
			{
				TIM2->ARR=1157;
			}
			else
			{
				//不加压有加减速
				//如果出于匀速运行阶段
				if(PulseCount>324&&PulseCount<=PulseNum-324)
					TIM2->ARR=556;
				//如果处于加速阶段
				else if(PulseCount<324)
					TIM2->ARR=Speedup_moto[PulseCount/2];
				//如果出于减速阶段
				else if((PulseNum-PulseCount)<324&&(PulseNum-PulseCount)>0)
					TIM2->ARR=Speedup_moto[(PulseNum-PulseCount)/2];
				//如果是AddedNum 部分，最低速度运行
				else
					TIM2->ARR=Speedup_moto[0];	
			}
		}
		PulseCount++;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	
	}
}

/*******************************************************************************
* Function Name  : TIM3_IRQHandler
* Description    : TIM3用于系统时间中断100ms中断一次
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM3_IRQHandler(void)
{
	if(TIM_GetFlagStatus(TIM3,TIM_FLAG_Update)!=RESET)
	{
		flickerTimes++;
		if(flickerTimes>10)
		{
			status=!status;
			flickerTimes=0;
			//collin add20141125
			switch(ledChannel)
			{
			case 1:LED(1,status);
			break;
			case 2:	LED(2,status);
			break;
			default:
			break;
			}

			//end
			switch(checkChannel)
			{
			/*
			case 1:LED(1,status);
			//collin add20141027
			//LED(2,status);
			//end
				break;
			//collin mask20141027
			
			case 2:LED(2,status);
				break;
			  */
			case 3:LED(3,status);
				break;
			case 4:LED(4,status);
				break;
			case 5:LED(5,status);
				break;
			case 6:LED(6,status);
				break;	 
			default:
				break;
			}
		} 
		Second_1++;
		//collin add20141013
		Second_2++;
		//end
		if((!(TIM2->CR1))&&(!SPI_I2S_GetITStatus(SPI2, SPI_I2S_FLAG_RXNE)))
		{
			u8	i;
			u32	temp;
 			
			ADCResult[ADCIndex]		= ADC(ADC_T_10A);
			ADCResult[ADCIndex+6]	= ADC(ADC_T_10B);
			ADCResult[ADCIndex+12]	= ADC(ADC_T18);
			ADCResult[ADCIndex+18]	= ADC(ADC_T42);
//			ADCResult[ADCIndex+24]	= ADC(ADC_ROOM);
//			ADCResult[ADCIndex+30]	= ADC(ADC_Humidity);

			for(i=0;i<6;i++)
			{
				temp = 197*ADCResult[0+i*6] + 1324*ADCResult[1+i*6] + 3479*ADCResult[2+i*6]
					 + 197*ADCResult[5+i*6] + 1324*ADCResult[4+i*6] + 3479*ADCResult[3+i*6];
				Temperature[i]	= (u16)(temp/10000);
			}
			ADCIndex++;
			if(ADCIndex>6)
				ADCIndex	= 0;
			
			if(Second_2>40)
			{
				Second_2	= 0;
				SendDataTemperature();
			}			
  	   	}
 		if(m_bRequestCf>0)
 		{
 			int k;

 			//发送标准曲线
 			QueryFlag	= 1;
 			//加入帧头									
 			Inqueue(0x02);
 			//加入命令标志
 			Inqueue(0xFF);
 			Inqueue(0x00);
 			Inqueue(0x43);
 			Inqueue(CfTimes);
 			for(k=0;k<8;k++)
 				Inqueue(Cf[k+CfTimes*8]);
 			for(k=0;k<FrameLength-14;k++)
 				Inqueue(0);
 			Inqueue(0x03);
 			//Delay5ms(10);
			//发送标准曲线
 			QueryFlag_uart1	= 1;
 			//加入帧头									
 			Inqueue_uart1(0x02);
 			//加入命令标志
 			Inqueue_uart1(0xFF);
 			Inqueue_uart1(0x00);
 			Inqueue_uart1(0x43);
 			Inqueue_uart1(CfTimes);
 			for(k=0;k<8;k++)
 				Inqueue_uart1(Cf[k+CfTimes*8]);
 			for(k=0;k<FrameLength-14;k++)
 				Inqueue_uart1(0);
 			Inqueue_uart1(0x03); 
 			CfTimes++;
 			
			if(CfTimes>=m_bRequestCf&&m_bRequestCf==1) 
			{
				CfTimes=0;
 				m_bRequestCf=0;
			} 
			else if(CfTimes>=m_bRequestCf&&m_bRequestCf>1)
 			{
 				CfTimes=0;
 				m_bRequestCf=1;
 			}
 		}
		if(flag_count==0&&m_bChopper==1)
			chopperDelaytimes++;
		else
			chopperDelaytimes=0;

		//检查发送队列，在批量传输AD数据时 QueryFlag=0
		//USART1->CR1&0x0080，表明串口发送中断开启
		
		if(QueryFlag && !(USART1->CR1&0x0080))
		{
			//如果循环队列非空且超过一帧数据
			if(QLength_uart1()!=0 &&QLength_uart1()/FrameLength>0)
			{
				//计算发送个数
				SendLength_uart1=((unsigned int)(QLength_uart1()/FrameLength))*FrameLength;
				//发送计数为0
				SendCounter_uart1=0;
				//启动串口发送中断
				USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
			}
		}
			//检查发送队列，在批量传输AD数据时 QueryFlag=0
		//USART3->CR1&0x0080，表明串口发送中断开启
		if(QueryFlag && !(USART3->CR1&0x0080))
		{
			//如果循环队列非空且超过一帧数据
			if(QLength()!=0 &&QLength()/FrameLength>0)
			{
				//计算发送个数
				SendLength=((unsigned int)(QLength()/FrameLength))*FrameLength;
				//发送计数为0
				SendCounter=0;
				//启动串口发送中断
				USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
			}
		}
			
		//维护日常时钟
		if(Second_1>=10)
		{
			QualifyTimes++;
			Second_1=0;
			worktime++;
			if(sleeptime<180&&flag_count==0)sleeptime++;
			if(SelfCheckCount>0)SelfCheckCount--;
		}
		SamplePosition();
		if(flag_count==1&&m_bMeasureBreak==0&&m_mesureType!=8) //停止键
		{  
			if(upperStop==1)
			{
					m_bMeasureBreak=1;
					LED(checkChannel,1);
					checkChannel=0;
					flag_count=0;
					upperStop=0;
					ch_selected=0;
			}
		}		
	}	
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

/*******************************************************************************
* Function Name  : TIM4_IRQHandler
* Description    : This function handles TIM4 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM4_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : I2C1_EV_IRQHandler
* Description    : This function handles I2C1 Event interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C1_EV_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : I2C1_ER_IRQHandler
* Description    : This function handles I2C1 Error interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C1_ER_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : I2C2_EV_IRQHandler
* Description    : This function handles I2C2 Event interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C2_EV_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : I2C2_ER_IRQHandler
* Description    : This function handles I2C2 Error interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C2_ER_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : SPI1_IRQHandler
* Description    : This function handles SPI1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI1_IRQHandler(void)
{
}


/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
	unsigned int i	= 0;
	unsigned int Position=0;
	u8 	temp; //吸气
//		u8 	FX=0; //吸气
//		u16	PNum=4500;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		//进入比较队列
		for(i=0;i<33;i++)
			ReadBuffer_uart1[i]=ReadBuffer_uart1[i+1];
		ReadBuffer_uart1[33]=USART_ReceiveData(USART1);
		if(ReadBuffer_uart1[10]==0x0A && ReadBuffer_uart1[33]==0x0A)
		{
 			//加入帧头
			Inqueue_uart1(0x02);
			//加入命令标志
			Inqueue_uart1(0xFF);
			Inqueue_uart1(0x08);
			Inqueue_uart1(0x08);	
			for(i=0;i<FrameLength-5;i++)
				Inqueue_uart1(0x08);
			Inqueue_uart1(0x08);
		}
		if(ReadBuffer_uart1[10]==0xA1 &&ReadBuffer_uart1[11]==0xA1 && ReadBuffer_uart1[33]==0xA1)	//进入气路控制状态并发送初始气路状态
		{	
			ReadS_V();	  //发送气阀状态
			ReadS_GPIO(); //发送IO口状态 
			tocontinue=2;
		}
		//如果满足帧结构
		if(ReadBuffer_uart1[10]==0x02 &&ReadBuffer_uart1[11]==0xFF && ReadBuffer_uart1[33]==0x03)
		{
		//collin add20141104
		//collin add20141009
		//接收制作标准曲线命令字
			if(ReadBuffer_uart1[13]==0xf4)
			{
				Sys_Status = Sys_Function;
			}
		//end
		//collin add20141011
		//确认测量键按下命令字
		if (ReadBuffer_uart1[13]==0xf2)
		{
			COUNT = 1;
		}
		//end
		//end
			//如果停止命令
			if(ReadBuffer_uart1[13]==0x02)
			{
//				m_bMeasureBreak=4;
				if(flag_count==1&&m_bMeasureBreak==0&&m_mesureType!=8) //停止键
				{
					upperStop=1;
				}
				Sys_Status	= Sys_Idle;
			}
			//如果继续命令
			else if(ReadBuffer_uart1[13]==0x5A)
			{
				m_bBusing=0; 
			}
			//如果继续命令
			else if(ReadBuffer_uart1[13]==0x46)
			{
				tocontinue=1;
			}
			//读预热时间
			else if(ReadBuffer_uart1[13]==0x1C)				
			{
				if(ReadBuffer_uart1[14]==0x01)
				{
					Inqueue_uart1(0x02);
					//加入命令标志
					Inqueue_uart1(0xFF);
					Inqueue_uart1(0x00);
					Inqueue_uart1(0x1C);	//预热时间
					Inqueue_uart1(QualifyTimes&0xFF);
					Inqueue_uart1((QualifyTimes>>8)&0xFF);
					Inqueue_uart1((QualifyTimes>>16)&0xFF);
					Inqueue_uart1((QualifyTimes>>24)&0xFF);
					for(i=0; i<FrameLength-9; i++)
						Inqueue_uart1(0);
					Inqueue_uart1(0x03);
					m_bRequestCf=4;
				}
				else
				{
					int Times=ReadBuffer_uart1[15]+(ReadBuffer_uart1[16]<<8)+(ReadBuffer_uart1[17]<<16)+(ReadBuffer_uart1[18]<<24);
					
					if(QualifyTimes>Times)
					{
						Inqueue_uart1(0x02);
						//加入命令标志
						Inqueue_uart1(0xFF);
						Inqueue_uart1(0x00);
						Inqueue_uart1(0x1C);	//预热时间
						Inqueue_uart1(QualifyTimes&0xFF);
						Inqueue_uart1((QualifyTimes>>8)&0xFF);
						Inqueue_uart1((QualifyTimes>>16)&0xFF);
						Inqueue_uart1((QualifyTimes>>24)&0xFF);
						for(i=0; i<FrameLength-9; i++)
							Inqueue_uart1(0);
						Inqueue_uart1(0x03);						
					}
					else
						QualifyTimes=ReadBuffer_uart1[15]+(ReadBuffer_uart1[16]<<8)+(ReadBuffer_uart1[17]<<16)+(ReadBuffer_uart1[18]<<24);					
				}
			}
			//如果数据接收正确
			else  if(ReadBuffer_uart1[13]==0x01)
			{
			//collin add20141014
			if (ReadBuffer_uart1[21] == 0x04)
			{
			m_mesureType = 4;
			qualityCount=0;
			flag_count=1;
			ch_selected=1;
			//upperCount=1;
			//caidan=0;
			}
			else if (ReadBuffer_uart1[21] == 0x01)
			upperCount=1;
			//end


				Sys_Status	= Sys_Running;
				//upperCount=1;
				ToMeasPosition[0]=ReadBuffer_uart1[14];
				m_bDilution=ToMeasPosition[1]=ReadBuffer_uart1[15]; 
				m_bIgnoreConcentrationLow=ToMeasPosition[2]=ReadBuffer_uart1[16];
				ToMeasPosition[3]=ReadBuffer_uart1[17];
				ToMeasPosition[4]=ReadBuffer_uart1[18];
				ToMeasPosition[5]=ReadBuffer_uart1[19]; 
				ToMeasPosition[6]=ReadBuffer_uart1[20]; 	//小电机开关，1：开启，2：关闭
				ToMeasPosition[7]=ReadBuffer_uart1[21]; 	//测试类型,1:normal,2:qulityCheck ,3：过滤器效率检查
				if(ToMeasPosition[7]==0x05)
				{
					LightModulation(OFF);
					NVIC_GenerateSystemReset();
				}
			}
			//如果校正命令
//			else if( ReadBuffer[13]==0x03)
//			{
//				Sys_Status	= Sys_SelfCheck;
//			}
//			//压力检测
//			else if(ReadBuffer[13]==0x22)
//			{
//				functioncheck|=0x02;
//			}
//			//光强检测
//			else if(ReadBuffer[13]==0x23)
//			{
//				functioncheck|=0x01;
//			}
//			//过滤器检测
//			else if(ReadBuffer[13]==0x24)
//			{
//				functioncheck|=0x04;
//			}
//			//气袋拔出
//			else if(ReadBuffer[13]==0x27)
//			{
//				m_bMeasureBreak=1;
//			}
			//如果请求标准曲线
			else if(ReadBuffer_uart1[13]==0x43)
			{
				if(ReadBuffer_uart1[14]==0)
					m_bRequestCf=1;
				else
				{
					u8 k;
					for(k=0;k<8;k++)
						Cf[(ReadBuffer_uart1[14]-1)*8+k]=ReadBuffer_uart1[15+k];
					m_bSaveCf ++;
				}	
			}
			//如果查询工作状态,使用循环队列
			else if(ReadBuffer_uart1[13]==0x0F)
			{
				//加入帧头
				Inqueue_uart1(0x02);
				//加入命令标志
				Inqueue_uart1(0xFF);
				Inqueue_uart1(0x00);
				//加入状态数据
				switch(Sys_Status)
				{
				case Sys_SelfCheck:
					Inqueue_uart1(0x04);
					break;
				case Sys_Error:
					Inqueue_uart1(0x05);
					break;
				case Sys_Running:
					Inqueue_uart1(0x06);
					Inqueue_uart1(ToMeasPosition[0]);
					break;
				case Sys_Idle:
					Inqueue_uart1(0x08);
					break;
				default:
					Inqueue_uart1(0x05);
					break;
				}
				//补零
				//如果不为Running状态
				if(Sys_Status!=Sys_Running)
					for(i=0;i<FrameLength-5;i++)
						Inqueue_uart1(0);
				else//如果为Running
					for(i=0;i<FrameLength-6;i++)
						Inqueue_uart1(0);
				Inqueue_uart1(0x03);
			}
				//如果查询样品在位
			else if( ReadBuffer_uart1[13]==0x07)
			{				
				while(USART1->CR1&0x0080);
				PHead_uart1=0,PEnd_uart1=0;
				//加入帧头
				Inqueue_uart1(0x02);
				//加入命令标志
				Inqueue_uart1(0xFF);
				Inqueue_uart1(0x00);
				Inqueue_uart1(0x07);
				i=0;
				if(!SamplePosition1)
					i+=0x01;
				if(!SamplePosition2)
					i+=0x02;
	   
				Inqueue_uart1(i);
				Inqueue_uart1(0xFF);
				for(i=0;i<FrameLength-7;i++)
					Inqueue_uart1(0);
				Inqueue_uart1(0x03);	 				
			}
			//如果要求重发数据
			else if(ReadBuffer_uart1[13]==0x40)
			{
				if(m_bBusing==0)
				{
					reSendData=1;
					checkOver=	ReadBuffer_uart1[14] + ReadBuffer_uart1[15]*256;
				}
			}
		}
		//如果满足帧结构
		if(ReadBuffer_uart1[10]==0x02 &&ReadBuffer_uart1[11]==0xFA && ReadBuffer_uart1[33]==0x03)
		{
			if(ReadBuffer_uart1[12]=='V'&& ReadBuffer_uart1[13]=='A'&&ReadBuffer_uart1[14]=='L'&&ReadBuffer_uart1[15]=='V'&&ReadBuffer_uart1[16]=='E')
				Valve(ReadBuffer_uart1[17]);
		}
	}
	//串口发送中断
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		//如果为查询部分输出数据
		if(QueryFlag_uart1)
		{
			//队列长度不为零，发送队头
			if(SendLength_uart1!=0)
			{
				USART_SendData(USART1,Exqueue_uart1());
				SendCounter_uart1++;
			}
			//如果发送完毕
			if(SendCounter_uart1==SendLength_uart1)
			{
				SendLength_uart1=0;
				SendCounter_uart1=0;
				USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
			}
		}
		//如果为语音芯片数据发送
		else
		{
			//i现在所处数据在一帧中的位置
			i=SendCounter_uart1%FrameLength;
			//发送帧头
			if(i==0)
			{
				USART_SendData(USART1,0x02);
				SendCounter_uart1++;
			}
			//发送数据标志位01 or 02
			else if(i==1)
			{
				if(Sys_Status == Sys_SelfCheck)
				{
					USART_SendData(USART1,0x11);
				}
				else
					USART_SendData(USART1,0x01);

				SendCounter_uart1++;
			}
			//发送数据包的编号
			else if(i==FrameLength-2)
			{
				temp = SendCounter_uart1/FrameLength;
				USART_SendData(USART1,temp);
				SendCounter_uart1++;
			}
			//发送帧尾
			else if(i==FrameLength-1)
			{
				USART_SendData(USART1,0x03);
				SendCounter_uart1++;
			}
			//如果是数据体
			else if(SendCounter_uart1<SendLength_uart1)
			{
				Position=(i-2)/2;
				switch((i-2)%2)
				{
				case 0:
					temp=(AK_Left_Data[Position]>>8)&0xFF;
					USART_SendData(USART1,temp);
					break;
				case 1:
					temp=(AK_Left_Data[Position])&0xFF;
					USART_SendData(USART1,temp);
					break;
				default:
					break;
				}
				SendCounter_uart1++;		
			}
			//如果数据发送完毕
			if(SendCounter_uart1>=SendLength_uart1)
			{
				SendLength_uart1	= 0;
				SendCounter_uart1	= 0;
				QueryFlag_uart1	= 1;
				m_bBusing	= 0;
				USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
			}							
		}
	}
}
/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : This function handles USART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART2_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : USART3_IRQHandler
* Description    : This function handles USART3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART3_IRQHandler(void)
{
	unsigned int i	= 0;
	unsigned int Position=0;
	u8 	temp; //吸气
//		u8 	FX=0; //吸气
//		u16	PNum=4500;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		//进入比较队列
		for(i=0;i<33;i++)
			ReadBuffer[i]=ReadBuffer[i+1];
		ReadBuffer[33]=USART_ReceiveData(USART3);
		if(ReadBuffer[10]==0x0A && ReadBuffer[33]==0x0A)
		{
 			//加入帧头
			Inqueue(0x02);
			//加入命令标志
			Inqueue(0xFF);
			Inqueue(0x0A);
			Inqueue(0x0A);	
			for(i=0;i<FrameLength-5;i++)
				Inqueue(0x0A);
			Inqueue(0x0A);
		}
		if(ReadBuffer[10]==0xA1 &&ReadBuffer[11]==0xA1 && ReadBuffer[33]==0xA1)	//进入气路控制状态并发送初始气路状态
		{	
			ReadS_V();	  //发送气阀状态
			ReadS_GPIO(); //发送IO口状态 
			tocontinue=2;
		}
		//如果满足帧结构
		if(ReadBuffer[10]==0x02 &&ReadBuffer[11]==0xFF && ReadBuffer[33]==0x03)
		{
		//collin add20140902
			//如果接收到仪器初始化开始命令
			/*
			if(ReadBuffer[13]==0xf1)
			{
				Sys_Status = Sys_Start;
			}
			*/
		//end
		//collin add20140917
		if(ReadBuffer[13]==0xb0)
		{
		if(ReadBuffer[14]==0x01)
		{
			Sys_Status = Sys_Print_Yes;
		}
		else if (ReadBuffer[14]==0x02)
		{
			Sys_Status = Sys_Print_No;
		}	 
		else if (ReadBuffer[14] == 0x03)
		{
			INQUIRY_PRINT = 1;
			DiapIndex = ReadBuffer[15]+(ReadBuffer[16]<<8);
		}
		}
		//end
		//collin add20141009
		//接收制作标准曲线命令字
			if(ReadBuffer[13]==0xf4)
			{
				Sys_Status = Sys_Function;
			}
		//end
		//collin add20141011
		//确认测量键按下命令字
		if (ReadBuffer[13]==0xf2)
		{
			COUNT = 1;
		}
		//end
		//collin add20141119
		if (ReadBuffer[13]==0xe0)
		{
			flag_print = 1;
			ID_NUM = ReadBuffer[14];
		}
		if (ReadBuffer[13] == 0xe1)
		{	flag_delete = 1;
		}
		//end
			//如果停止命令
			if(ReadBuffer[13]==0x02)
			{
//				m_bMeasureBreak=4;
				if(flag_count==1&&m_bMeasureBreak==0&&m_mesureType!=8) //停止键
				{
					upperStop=1;
				}
				Sys_Status	= Sys_Idle;
			}
			//如果继续命令
			else if(ReadBuffer[13]==0x5A)
			{
				m_bBusing=0; 
			}
			//如果继续命令
			else if(ReadBuffer[13]==0x46)
			{
				tocontinue=1;
			}
			//读预热时间
			else if(ReadBuffer[13]==0x1C)
			{
				if(ReadBuffer[14]==0x01)
				{
					Inqueue(0x02);
					//加入命令标志
					Inqueue(0xFF);
					Inqueue(0x00);
					Inqueue(0x1C);	//预热时间
					Inqueue(QualifyTimes&0xFF);
					Inqueue((QualifyTimes>>8)&0xFF);
					Inqueue((QualifyTimes>>16)&0xFF);
					Inqueue((QualifyTimes>>24)&0xFF);
					for(i=0; i<FrameLength-9; i++)
						Inqueue(0);
					Inqueue(0x03);
					m_bRequestCf=4;
				}
				else
				{
					int Times=ReadBuffer[15]+(ReadBuffer[16]<<8)+(ReadBuffer[17]<<16)+(ReadBuffer[18]<<24);
					
					if(QualifyTimes>Times)
					{
						Inqueue(0x02);
						//加入命令标志
						Inqueue(0xFF);
						Inqueue(0x00);
						Inqueue(0x1C);	//预热时间
						Inqueue(QualifyTimes&0xFF);
						Inqueue((QualifyTimes>>8)&0xFF);
						Inqueue((QualifyTimes>>16)&0xFF);
						Inqueue((QualifyTimes>>24)&0xFF);
						for(i=0; i<FrameLength-9; i++)
							Inqueue(0);
						Inqueue(0x03);						
					}
					else
						QualifyTimes=ReadBuffer[15]+(ReadBuffer[16]<<8)+(ReadBuffer[17]<<16)+(ReadBuffer[18]<<24);					
				}
			}
			//如果数据接收正确
			else  if(ReadBuffer[13]==0x01)
			{
			//collin add20141014
			if (ReadBuffer[21] == 0x04)
			{
			m_mesureType = 4;
			qualityCount=0;
			flag_count=1;
			ch_selected=1;
			//upperCount=1;
			//caidan=0;
			}
			else if (ReadBuffer[21] == 0x01)
			upperCount=1;
			//end
				Sys_Status	= Sys_Running;
				//collin mask20141014
				//upperCount=1;
				//end
				ToMeasPosition[0]=ReadBuffer[14];
				m_bDilution=ToMeasPosition[1]=ReadBuffer[15]; 
				m_bIgnoreConcentrationLow=ToMeasPosition[2]=ReadBuffer[16];
				ToMeasPosition[3]=ReadBuffer[17];
				ToMeasPosition[4]=ReadBuffer[18];
				ToMeasPosition[5]=ReadBuffer[19]; 
				ToMeasPosition[6]=ReadBuffer[20]; 	//小电机开关，1：开启，2：关闭
				ToMeasPosition[7]=ReadBuffer[21]; 	//测试类型,1:normal,2:qulityCheck ,3：过滤器效率检查
				if(ToMeasPosition[7]==0x05)
				{
					LightModulation(OFF);
					NVIC_GenerateSystemReset();
				}
			}
			//如果校正命令
//			else if( ReadBuffer[13]==0x03)
//			{
//				Sys_Status	= Sys_SelfCheck;
//			}
//			//压力检测
//			else if(ReadBuffer[13]==0x22)
//			{
//				functioncheck|=0x02;
//			}
//			//光强检测
//			else if(ReadBuffer[13]==0x23)
//			{
//				functioncheck|=0x01;
//			}
//			//过滤器检测
//			else if(ReadBuffer[13]==0x24)
//			{
//				functioncheck|=0x04;
//			}
//			//气袋拔出
//			else if(ReadBuffer[13]==0x27)
//			{
//				m_bMeasureBreak=1;
//			}
			//如果请求标准曲线
			else if(ReadBuffer[13]==0x43)
			{
				if(ReadBuffer[14]==0)
					m_bRequestCf=1;
				else
				{
					u8 k;
					for(k=0;k<8;k++)
						Cf[(ReadBuffer[14]-1)*8+k]=ReadBuffer[15+k];
					m_bSaveCf ++;
				}	
			}
			//如果查询工作状态,使用循环队列
			else if(ReadBuffer[13]==0x0F)
			{
				//加入帧头
				Inqueue(0x02);
				//加入命令标志
				Inqueue(0xFF);
				Inqueue(0x00);
				//加入状态数据
				switch(Sys_Status)
				{
				case Sys_SelfCheck:
					Inqueue(0x04);
					break;
				case Sys_Error:
					Inqueue(0x05);
					break;
				case Sys_Running:
					Inqueue(0x06);
					Inqueue(ToMeasPosition[0]);
					break;
				case Sys_Idle:
					Inqueue(0x08);
					break;
				default:
					Inqueue(0x05);
					break;
				}
				//补零
				//如果不为Running状态
				if(Sys_Status!=Sys_Running)
					for(i=0;i<FrameLength-5;i++)
						Inqueue(0);
				else//如果为Running
					for(i=0;i<FrameLength-6;i++)
						Inqueue(0);
				Inqueue(0x03);
			}
				//如果查询样品在位
			else if( ReadBuffer[13]==0x07)
			{				
				while(USART3->CR1&0x0080);
				PHead=0,PEnd=0;
				//加入帧头
				Inqueue(0x02);
				//加入命令标志
				Inqueue(0xFF);
				Inqueue(0x00);
				Inqueue(0x07);
				i=0;
				if(!SamplePosition1)
					i+=0x01;
				if(!SamplePosition2)
					i+=0x02;
				if(!SamplePosition3)
					i+=0x04;
				if(!SamplePosition4)
					i+=0x08;
				if(!SamplePosition5)
					i+=0x10;
				if(!SamplePosition6)
					i+=0x20;	   
				Inqueue(i);
				Inqueue(0xFF);
				for(i=0;i<FrameLength-7;i++)
					Inqueue(0);
				Inqueue(0x03);	 				
			}
			//如果要求重发数据
			else if(ReadBuffer[13]==0x40)
			{
				if(m_bBusing==0)
				{
					reSendData=1;
					checkOver=	ReadBuffer[14] + ReadBuffer[15]*256;
				}
			}
		}
		//如果满足帧结构
		if(ReadBuffer[10]==0x02 &&ReadBuffer[11]==0xFA && ReadBuffer[33]==0x03)
		{
			if(ReadBuffer[12]=='V'&& ReadBuffer[13]=='A'&&ReadBuffer[14]=='L'&&ReadBuffer[15]=='V'&&ReadBuffer[16]=='E')
				Valve(ReadBuffer[17]);
		}
	}
	//串口发送中断
	if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
	{
		//如果为查询部分输出数据
		if(QueryFlag)
		{
			//队列长度不为零，发送队头
			if(SendLength!=0)
			{
				USART_SendData(USART3,Exqueue());
				SendCounter++;
			}
			//如果发送完毕
			if(SendCounter==SendLength)
			{
				SendLength=0;
				SendCounter=0;
				USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
			}
		}
		//如果为语音芯片数据发送
		else
		{
			//i现在所处数据在一帧中的位置
			i=SendCounter%FrameLength;
			//发送帧头
			if(i==0)
			{
				USART_SendData(USART3,0x02);
				SendCounter++;
			}
			//发送数据标志位01 or 02
			else if(i==1)
			{
				if(Sys_Status == Sys_SelfCheck)
				{
					USART_SendData(USART3,0x11);
				}
				else
					USART_SendData(USART3,0x01);

				SendCounter++;
			}
			//发送数据包的编号
			else if(i==FrameLength-2)
			{
				temp = SendCounter/FrameLength;
				USART_SendData(USART3,temp);
				SendCounter++;
			}
			//发送帧尾
			else if(i==FrameLength-1)
			{
				USART_SendData(USART3,0x03);
				SendCounter++;
			}
			//如果是数据体
			else if(SendCounter<SendLength)
			{
				Position=(i-2)/2;
				switch((i-2)%2)
				{
				case 0:
					temp=(AK_Left_Data[Position]>>8)&0xFF;
					USART_SendData(USART3,temp);
					break;
				case 1:
					temp=(AK_Left_Data[Position])&0xFF;
					USART_SendData(USART3,temp);
					break;
				default:
					break;
				}
				SendCounter++;		
			}
			//如果数据发送完毕
			if(SendCounter>=SendLength)
			{
				SendLength	= 0;
				SendCounter	= 0;
				QueryFlag	= 1;
				m_bBusing	= 0;
				USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
			}							
		}
	}

}

/*******************************************************************************
外部中断用于光电开关信号检测
*******************************************************************************/
void EXTI15_10_IRQHandler(void)
{
if(EXTI_GetITStatus(EXTI_LINE_TripSwitch) != RESET)
	{
		if(cw_status==BACKWARD)
		{
			//补足脉冲数
			PulseCount=PulseNum+AddedNum+500;
			//电机中断和定时器关闭
			TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM2,DISABLE);
			//关闭外部中断					 
			EXTI->IMR&=0xFFFFDFFF;
			TripSwitchStatus=1;
		}
		EXTI_ClearITPendingBit(EXTI_LINE_TripSwitch);
	}
}

/*******************************************************************************
* Function Name  : RTCAlarm_IRQHandler
* Description    : This function handles RTC Alarm interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTCAlarm_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : USBWakeUp_IRQHandler
* Description    : This function handles USB WakeUp interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBWakeUp_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM8_BRK_IRQHandler
* Description    : This function handles TIM8 Break interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_BRK_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM8_UP_IRQHandler
* Description    : This function handles TIM8 overflow and update interrupt 
*                  request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_UP_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM8_TRG_COM_IRQHandler
* Description    : This function handles TIM8 Trigger and commutation interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_TRG_COM_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM8_CC_IRQHandler
* Description    : This function handles TIM8 capture compare interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_CC_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : ADC3_IRQHandler
* Description    : This function handles ADC3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : FSMC_IRQHandler
* Description    : This function handles FSMC global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : SDIO_IRQHandler
* Description    : This function handles SDIO global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  SDIO_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : TIM5_IRQHandler
* Description    : This function handles TIM5 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM5_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : SPI3_IRQHandler
* Description    : This function handles SPI3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : UART4_IRQHandler
* Description    : This function handles UART4 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART4_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : UART5_IRQHandler
* Description    : This function handles UART5 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART5_IRQHandler(void)
{
}



/*******************************************************************************
* Function Name  : TIM7_IRQHandler
* Description    : This function handles TIM7 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM7_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA2_Channel1_IRQHandler
* Description    : This function handles DMA2 Channel 1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel1_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA2_Channel2_IRQHandler
* Description    : This function handles DMA2 Channel 2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel2_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA2_Channel3_IRQHandler
* Description    : This function handles DMA2 Channel 3 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel3_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : DMA2_Channel4_5_IRQHandler
* Description    : This function handles DMA2 Channel 4 and DMA2 Channel 5
*                  interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel4_5_IRQHandler(void)
{
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
