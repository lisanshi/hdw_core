/* Includes ------------------------------------------------------------------*/
//2011年8月22日tim2关于	if(PulseCount>=PulseNum+AddedNum)判断去掉，防止死机
//2011年8月22日质控也稀释
//2011年8月22日最低浓度调为0.3
//2011年8月26日光强检测峰值检测由高于2048改为高于1536
#include "stm32f10x_lib.h"
#include "function.h"
#include "motion.h"
#include "math.h"
#include "stm32_dsp.h"
#include "table_fft.h"
#include "i2c_ee.h"
#include "lcd.h"
#include "spi_flash.h"  
#include "sysdate.h"   
/* Private define -------------------------------------------------------------*/
#define Software_Version_Major    2
#define Software_Version_Minor	  2
#define Software_Version_Revision    1
#define EEPROM_DeviceNoAddress    0x00
/* Private macro -------------------------------------------------------------*/
//#define countof(a) (sizeof(a) / sizeof(*(a)))
#define NPT 256            /* NPT = No of FFT point*/
u8 ZF;	//正负号
u16	A1,A2,A3,A4;
u8	sampleOn;
u8	qualityCount;//质控次数；
u8	K1,K2,K3,K4,K5,K6,K7,K8,K9,K0,K_D,K_J,K_Z,K_Y,K_S,K_X,MENU,COUNT,CONFIRM,REMOVE,RETURN,STOP,PRINT,INQUIRY,INQUIRY_PRINT;
u8	flag_find,flag_gb,flag_cx,xianshi;
u16	worktime=0;
u8	ch_selected,ch_tested[6];//样品在位通道
u8	flag_count=0;
u32	YH08=12345;
u8	flag_sleeped=0; 
u8	MenuIndex=1,caidan,xiugai,sjxg,dobxg,pdzxg,flag_calibration,password;
/* Private variables ---------------------------------------------------------*/
u8	m_bChopper;
u8	tocontinue;
// u8	m_bRequestCf=0;
u16	m_bSaveCf=0; 
u8	m_bMeasureBreak=0;
u8 	m_bIgnoreConcentrationLow=0;
u8 Cf[128] = {0};
u8 Abs_backup[320]={0};
double absC12_backup[40]={0};
double absC13_backup[40]={0};
u8	N[8],M[13];				//常数项个个位（个位，十位，百位，千位等等）的值
u32 ParamT;
u16 ParamP,PressType;
u32 C13[60],C12[60];
long long C12_PJ,C13_PJ,C12_BZWC,C13_BZWC;
extern u8  flag_moto;

//collin add20141016
int debug_collin3;
//end
//collin add20141119
u8 flag_print ;
int ID_NUM;
u8 flag_delete;
//end
/* Private Functions ---------------------------------------------------------*/
void 	ChopperCheck(u8 run);
void 	ChopperCheck1(u8 run);
void 	PowerCheck(void);
void	PerkCheck(void);

extern volatile unsigned int m;
extern u16 TableFFT[];
extern volatile u32 TimingDelay ;
long lBUFIN[NPT];         /* Complex input vector */
long lBUFOUT[NPT];        /* Complex output vector */
long lBUFMAG[NPT + NPT/2];/* Magnitude vector */
//系统初始化
void System_Init(void);
void MainInterface(void);
void queryProcess(void);
void menuProcess(void);
//collin add20140917
void query_printProcess(void);
//end


/**
* @brief  Removes the aliased part of the spectrum (not tested)
* @param ill: length of the array holding power mag
* @retval : None
*/
void onesided(long nfill)
{
    u32 i;

    lBUFMAG[0] = lBUFMAG[0];
    lBUFMAG[nfill/2] = lBUFMAG[nfill/2];
    for (i=1; i < nfill/2; i++)
    {
        lBUFMAG[i] = lBUFMAG[i] + lBUFMAG[nfill-i];
        lBUFMAG[nfill-i] = 0x0;
    }
}
/**
* @brief  Compute power magnitude of the FFT transform
* @param ill: length of the array holding power mag
*   : strPara: if set to "1SIDED", removes aliases part of spectrum (not tested)
* @retval : None
*/
void powerMag(long nfill, char* strPara)
{
    s32 lX,lY;
    u32 i;

    for (i=0; i < nfill; i++)
    {
        lX= (lBUFOUT[i]<<16)>>16; /* sine_cosine --> cos */
        lY= (lBUFOUT[i] >> 16);   /* sine_cosine --> sin */
        {
            float X=  64*((float)lX)/32768;
            float Y = 64*((float)lY)/32768;
            float Mag = sqrt(X*X+ Y*Y)/nfill;
            lBUFMAG[i] = (u32)(Mag*65536);
        }
    }
    if (strPara == "1SIDED")
        onesided(nfill);
}

void SystemCheck(void)
{
    u8 i;
    Sys_Status = Sys_Running;
    m_bBusing	= 1;
    m_mesureType= 0;
	SelfChecking=ON;
    //报告工作状态
    QueryFlag	= 1;
    //加入帧头
    Inqueue(0x02);
    //加入命令标志
    Inqueue(0xFF);
    Inqueue(0x00);
    Inqueue(0x24);
    for(i=0; i<FrameLength-5; i++)
        Inqueue(0);
    Inqueue(0x03);
	Delay5ms(20);

	//报告工作状态
    QueryFlag_uart1	= 1;
    //加入帧头
    Inqueue_uart1(0x02);
    //加入命令标志
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    Inqueue_uart1(0x24);
    for(i=0; i<FrameLength-5; i++)
        Inqueue_uart1(0);
    Inqueue_uart1(0x03);
	Delay5ms(20);

	//光电开关检查及气缸电机归位
    Motor_Init();
	Delay5ms(200);
    PowerCheck();
	Delay5ms(20);
    m=Test(); //试压
	//collin open20141016
	PerkCheck();
	//end
	Delay5ms(20);
  
    //加入帧头
    Inqueue(0x02);
    //加入命令标志
    Inqueue(0xFF);
    Inqueue(0x00);
    Inqueue(0x34);
    for(i=0; i<FrameLength-5; i++)
        Inqueue(0);
    Inqueue(0x03);
    Delay5ms(100);
	
	//加入帧头
    Inqueue_uart1(0x02);
    //加入命令标志
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    Inqueue_uart1(0x34);
    for(i=0; i<FrameLength-5; i++)
        Inqueue_uart1(0);
    Inqueue_uart1(0x03);
    Delay5ms(100);
		
    Sys_Status	== Sys_Idle;
    SelfChecking=OFF;
    m_bBusing=0;
    checkChannel=0;
    sampleTooless =0;
	sleeptime=0;
}

int main(void)
{
	//collin add20141011
	int temp_serial_flag;
	//end

    int i,j=0,temp;
	QualifyTimes=0;
	tocontinue=0;
	//collin add20141119
	flag_print = 0;
	flag_delete = 0;
	//end	
    System_Init();

	   	 /*
		write_ds(0x04,(0*10+9));
		Delay5ms(10);
		write_ds(0x02,(4*10+5));
		Delay5ms(10);
		write_ds(0x00,(0*10+0));
		Delay5ms(10);
		   */
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

    SystemCheck();

  
	 //write_ds(0x09,(1*10+4));
	 //Delay5ms(10);
	 //write_ds(0x08,(1*10+1));
	//	Delay5ms(10);
	//	write_ds(0x07,(2*10+8));
	//	Delay5ms(10);
	/*
		write_ds(0x04,(0*10+9));
		Delay5ms(10);
		write_ds(0x02,(3*10+1));
		Delay5ms(10);
		write_ds(0x00,(0*10+0));
		Delay5ms(10);
		8*/
		//timer.week=RTC_Get_Week(2000+timer.w_year,timer.w_month,timer.w_date);
	//	sjxg=0;
  	

    while(1)
    {
		//collin add20141119
		if(flag_print == 1)
		{
		flag_print = 0;
		SPI_FLASH_BufferRead(ID,(0x010000+ID_NUM*30),30);
		print_result_search();
		}
		if (flag_delete == 1)
		{
		flag_delete = 0;
		deleteRecord();
		}
		//end
		keynum=0;

        if(chopperDelaytimes>=6000)
        {
            chopperDelaytimes=0;
            LightModulation(OFF);
        }
        if(m_bMeasureBreak!=0)
        {
            Inqueue(0x02);
            //加入命令标志
            Inqueue(0xFF);
            Inqueue(0x00);
            Inqueue(0x44);
            Inqueue(m_bMeasureBreak);
            for(i=0; i<FrameLength-6; i++)
                Inqueue(0);
            Inqueue(0x03);
            Delay5ms(50);

			Inqueue_uart1(0x02);
            //加入命令标志										   
            Inqueue_uart1(0xFF);
            Inqueue_uart1(0x00);
            Inqueue_uart1(0x44);
            Inqueue_uart1(m_bMeasureBreak);
            for(i=0; i<FrameLength-6; i++)
                Inqueue_uart1(0);
            Inqueue_uart1(0x03);
            Delay5ms(50);

			m_bBusing=0;
            m_bMeasureBreak=0;
			checkChannel=0;
			//collin add20141125
			ledChannel = 0;
			//end
        }
		if(flag_count==0)
		{
			if(flag_sleeped)
			{
				if(!sleeptime)
				{
					ch_selected=0;
					flag_sleeped=0;
					//collin mask20141203
					/*
					while(sleeptime==0)
					{				
					}
					*/				
				}
				else
				{
					sampleScan();
					if(sampleOn!=sampleChanged)
					sleeptime=0;					
				}
			}
			else	if(sleeptime==180)

			{
				flag_sleeped=1;
				sampleChanged=sampleOn;			
			}
			else	MainInterface();

			//collin add20140917
			if(INQUIRY_PRINT)
			query_printProcess();
			//end
			//collin add20141009
			if(Sys_Status == Sys_Function)
			{

				
				password=0;	
				flag_count=1;
				m_mesureType=8;
				ch_selected=1;
				m_bIgnoreConcentrationLow=1;
				caidan=0;

				Sys_Status = Sys_Idle;
			}
			//end
			//collin add2014
			/*
			if (m_mesureType == 4)
			{
					//qualityCount=0;
					flag_count=1;
					ch_selected=1;
					caidan=0;
			}
			*/
			//end
			if(COUNT||upperCount)
			{
			//collin modify20141128
				m_mesureType=1;
				//m_mesureType = ToMeasPosition[7];
				upperCount=0;
				if(Num>=65530)  //防止6个通道同时检测时，第一通道就是9999编号
	            {   
					keynum=0;
					ch_selected=0;
					while(keynum==0)
					{		   
					}                      
	            }
				else 
				{	
					temp=1;
					testcount=0;
					if(!ch_selected)
					ch_selected=sampleOn;

					for(i=0;i<6;i++)
					{
						if((ch_selected&temp)==temp)
						{
							testcount++;
							ch_tested[i]=1;
							ch_checked[i]=0;
							flag_count=1;
							ChopperCheck(1);									
						}
						else ch_tested[i]=0;
						temp<<=1;
						flag_err[i]=0;			
					}		
				}			
			}
		}
        else
		{
			temp=1;	
			j=1; 
			m_bMeasureBreak=0;

			switch (m_mesureType)
			{
			  	case 1:

					for(i=0;i<6;i++)
					{
						if((ch_selected&temp)==temp)
						{
							ch_selected&=(0xFF-temp);
							checkChannel =i+1;					
							break;
						}
						temp<<=1;			
					}		
					break;
				case 4:
					qualityCount++;
					checkChannel =1;
					//collin add20141203
					ChopperCheck(1);
					//end

					//collin add20141014
					Inqueue(0x02);
                	Inqueue(0xFF);
                	Inqueue(0x00);
                	Inqueue(0xf5);
					Inqueue(qualityCount);
                	for(i=0; i<FrameLength-6; i++)
                    	Inqueue(0);
                	Inqueue(0x03);
                	Delay5ms(10);
					//end
					//collin add20141014
					Inqueue_uart1(0x02);
                	Inqueue_uart1(0xFF);
                	Inqueue_uart1(0x00);
                	Inqueue_uart1(0xf5);
					Inqueue_uart1(qualityCount);
                	for(i=0; i<FrameLength-6; i++)
                    	Inqueue_uart1(0);
                	Inqueue_uart1(0x03);
                	Delay5ms(10);
					//end
					break;
				case 8:
					j=5;temp=1;	
					//collin add20141011
					temp_serial_flag = 0;
					//end	
					while(tocontinue==0)
                	{
						//collin add20141009
						//插入标准样品次数
							if(temp_serial_flag != 5)
							{
						Inqueue(0x02);
                Inqueue(0xFF);
                Inqueue(0x00);
                Inqueue(0xf3);
                Inqueue(serial+1);
                for(i=0; i<FrameLength-6; i++)
                    Inqueue(0);
                Inqueue(0x03);
                Delay5ms(10);

				Inqueue_uart1(0x02);
                Inqueue_uart1(0xFF);
                Inqueue_uart1(0x00);
                Inqueue_uart1(0xf3);
                Inqueue_uart1(serial+1);
                for(i=0; i<FrameLength-6; i++)
                    Inqueue_uart1(0);
                Inqueue_uart1(0x03);
                Delay5ms(10);

			   temp_serial_flag++;
				}
						//end
						if(COUNT)
						{
							if(SAMPLE1)
							{									
							}
							else
							{
								tocontinue=1;
								checkChannel =1;
								//collin add20141011
								COUNT = 0;
								//end
							}
						}	
					}						
				
				//collin add20141011			
				Inqueue(0x02);
                Inqueue(0xFF);
                Inqueue(0x00);
                Inqueue(0xf1);
				Inqueue(serial+1);
                for(i=0; i<FrameLength-6; i++)
                Inqueue(0);
                Inqueue(0x03);
                Delay5ms(10);
				//end
				//collin add20141011			
				Inqueue_uart1(0x02);
                Inqueue_uart1(0xFF);
                Inqueue_uart1(0x00);
                Inqueue_uart1(0xf1);
				Inqueue_uart1(serial+1);
                for(i=0; i<FrameLength-6; i++)
                Inqueue_uart1(0);
                Inqueue_uart1(0x03);
                Delay5ms(10);
				//end
					break;
				default:
					break;
			}	
			//报告工作状态
            QueryFlag	= 1;
            Inqueue(0x02);
            Inqueue(0xFF);
            Inqueue(0x00);
            Inqueue(0x42);
            Inqueue(checkChannel*11);
            for(i=0; i<FrameLength-6; i++)
                Inqueue(0);
            Inqueue(0x03);
            Delay5ms(20);

			//报告工作状态
            QueryFlag_uart1	= 1;
            Inqueue_uart1(0x02);
            Inqueue_uart1(0xFF);
            Inqueue_uart1(0x00);
            Inqueue_uart1(0x42);
            Inqueue_uart1(checkChannel*11);
            for(i=0; i<FrameLength-6; i++)
                Inqueue_uart1(0);
            Inqueue_uart1(0x03);
            Delay5ms(20);
	 
            for(;j<10; j++)
            {
                if(sampleTooless||m_bMeasureBreak)
                    break;                 
                //报告工作状态
				Inqueue(0x02);
                Inqueue(0xFF);
                Inqueue(0x00);
                Inqueue(0x1B);
                Inqueue(m_mesureType);
                Inqueue(checkChannel*10+j);
				Inqueue(serial);
                for(i=0; i<FrameLength-8; i++)
                    Inqueue(0);
                Inqueue(0x03);
                Delay5ms(10);

				//报告工作状态
				Inqueue_uart1(0x02);
                Inqueue_uart1(0xFF);
                Inqueue_uart1(0x00);
                Inqueue_uart1(0x1B);
                Inqueue_uart1(m_mesureType);
                Inqueue_uart1(checkChannel*10+j);
				Inqueue_uart1(serial);
                for(i=0; i<FrameLength-8; i++)
                    Inqueue_uart1(0);
                Inqueue_uart1(0x03);
                Delay5ms(10);

                ChannelMeas(checkChannel*10+j);
            }
			tocontinue=0;
			if(ch_selected==0)
			{	
				//collin modify20141121
				keynum=0;
				//keynum = 1;
				//end
				sampleScan();
				sampleChanged=sampleOn;	
				while(m_mesureType==1&&flag_count)
				{
					sampleScan();
					//if(keynum>0||sampleChanged!=sampleOn)
					{
						break;
					}
				}
				flag_count=0;
			}         
        }
    }
}
void writeDefaltParam(void)
{
	u8 j;
	u32 temp1; 
	u16 temp3;
	double temp2;
	temp1=201301001;
	for (j=0; j<8; j++)  
	{  
	   Cf[j] = *((char *)(&temp1) + j);
	}  
 	temp1=20121126;
	for (j=0; j<8; j++)  
	{  
	   Cf[j+8] = *((char *)(&temp1) + j);
	} 
 	temp3=2190;
	for (j=0; j<4; j++)  
	{  
	   Cf[j+16] = *((char *)(&temp3) + j);
	}  
 	temp3=5100;
	for (j=0; j<4; j++)  
	{  
	   Cf[j+24] = *((char *)(&temp3) + j);
	} 
	Cf[21]=2;
	Cf[22]=2;
	Cf[23]=0;
    Cf[30]=0;
	Cf[31]=4;

	temp2=-0.009;
 	for (j=0; j<8; j++)  
	{  
	   Cf[j+32] = *((char *)(&temp2) + j);
	} 
	temp2=2.653445118799;
 	for (j=0; j<8; j++)  
	{  
	   Cf[j+40] = *((char *)(&temp2) + j);
	} 
	temp2=18.18116035734;
 	for (j=0; j<8; j++)  
	{  
	   Cf[j+48] = *((char *)(&temp2) + j);
	} 
	temp2=-36.2348244788;
 	for (j=0; j<8; j++)  
	{  
	   Cf[j+56] = *((char *)(&temp2) + j);
	} 
	temp2=33.28807068616;
 	for (j=0; j<8; j++)  
	{  
	   Cf[j+64] = *((char *)(&temp2) + j);
	} 
	temp2=0.0;
 	for (j=0; j<8; j++)  
	{  
	   Cf[j+72] = *((char *)(&temp2) + j);
	} 
	temp2=0.02877660870843;
 	for (j=0; j<8; j++)  
	{  
	   Cf[j+80] = *((char *)(&temp2) + j);
	} 
	temp2=15.96806105078;
 	for (j=0; j<8; j++)  
	{  
	   Cf[j+88] = *((char *)(&temp2) + j);
	} 
	temp2=61.57375472054;
 	for (j=0; j<8; j++)  
	{  
	   Cf[j+96] = *((char *)(&temp2) + j);
	} 
	temp2=-271.0937606058;
 	for (j=0; j<8; j++)  
	{  
	   Cf[j+104] = *((char *)(&temp2) + j);
	} 
	temp2=585.9947194853;
 	for (j=0; j<8; j++)  
	{  
	   Cf[j+112] = *((char *)(&temp2) + j);
	} 
	temp2=0.0;
 	for (j=0; j<8; j++)  
	{  
	   Cf[j+120] = *((char *)(&temp2) + j);
	} 

	I2C_EE_BufferWrite(Cf, 0,128);
}

void 	System_Init(void)
{
	u8 i,j;
    Sys_Status = Sys_Running;
    m_bChopper = 0;
    reSendData=0;
    RCC_Configuration();
    NVIC_Configuration();
    GPIO_Configuration();
	LED(1,0);
	LED(2,0);
    Delay5ms_Config(); 
	Speaker_Config();
    Uart1_Config();
	Uart3_Config();

    ADC_Config();
    EXTI_Config();
    AK5385_Config();
    LightModulation_Config();
    Motor_Config();
    Valve_GPIO_Config();
    Sys_100msClock_Config();	
 	SPI_FLASH_Init();
	
	PRINT_Init();	
	CE=0;													   	     
	RE=1;						   
	FS=0;						   	

	ALE=0;					   		 
	RDT=0;					   		
	WRT=0;
	write_ds(0x0A,0x20);		                // 系统时间初始化
	write_ds(0x0B,0x06); 		                // 系统时间初始化
			
	SPI_FLASH_BufferRead(N,0x9E0000,8);	   //
	if(N[0]==0xff)flag_dob=40;
    else    flag_dob=N[0];
    if(N[2]==0||N[2]==1)flag_cz=N[2];
		else  flag_cz=1;
    if(N[3]==0xff)cuozhi=40;
    else    cuozhi=N[3];
    SPI_FLASH_BufferRead(N,0x8F0000,8);
    if(N[0]==0xff)
    {
        YH08=0;
    }
	else 
	{
		YH08=N[1]*100000+N[2]*10000+N[3]*1000+N[4]*100+N[5]*10+N[6];
	}
	SPI_FLASH_BufferRead(ID,0x7E0000,30);		//读取样品编号
	if(ID[2]==0xff&&ID[3]==0xff)
	{
		Num=0;		
	}
	else
	{					
		Num=ID[2]*256+ID[3];
	}		
    I2C_EE_Init();
    I2C_EE_BufferRead(Cf, 0,128);
	if(Cf[0]==0xFF&&Cf[1]==0xFF)
	{
		writeDefaltParam();
	}
    Cf[21]=Software_Version_Major;		  //加载版本号
    Cf[22]=Software_Version_Minor;
    Cf[23]=Software_Version_Revision;
	for(i=0;i<6;i++)
	{
		m_lCfC12[i]= *((double *)(&Cf[32+i*8]));
		m_lCfC13[i]= *((double *)(&Cf[32+(i+6)*8]));
	}

	SPI_FLASH_BufferRead(Abs_backup,0x7D0000,320);
	for(i=0;i<20;i++)
	{
		absC12_backup[i]= *((double *)(&Abs_backup[i*8]));
		absC13_backup[i]= *((double *)(&Abs_backup[(i+20)*8]));
	}	
	
	ParamT	= (Cf[17]<<8) + Cf[16];
	ParamP	= (Cf[27]<<24) + (Cf[26]<<16) + (Cf[25]<<8) + Cf[24];	
	PressType=Cf[31];	
	for(i=0;i<4;i++)  
	{
		//发送标准曲线
		Inqueue(0x02);
		Inqueue(0xFF);
		Inqueue(0x00);
		Inqueue(0x43);
		Inqueue(i);
		for(j=0;j<8;j++)
			Inqueue(Cf[j+i*8]);
		for(j=0;j<FrameLength-14;j++)
			Inqueue(0);
		Inqueue(0x03);
		Delay5ms(100);
		
		//发送标准曲线
		Inqueue_uart1(0x02);
		Inqueue_uart1(0xFF);
		Inqueue_uart1(0x00);
		Inqueue_uart1(0x43);
		Inqueue_uart1(i);
		for(j=0;j<8;j++)
			Inqueue_uart1(Cf[j+i*8]);
		for(j=0;j<FrameLength-14;j++)
			Inqueue_uart1(0);
		Inqueue_uart1(0x03);
		Delay5ms(100);			
	}

	if(ADC(ADC_T42)<2350)
	{
		//自检定时器清零
		SelfCheckCount=600;
		 //SelfCheckCount = 0;
		//启动自检计时
		SelfChecking=ON; 
		while(SelfCheckCount>0)
		{
			Delay5ms(600);
			Inqueue(0x02);
			//加入命令标志
			Inqueue(0xFF);
			Inqueue(0x00);
			Inqueue(0x1C);	//预热温控
			Inqueue(0x01);
			for(i=0;i<FrameLength-6;i++)
				Inqueue(0);
			Inqueue(0x03);
			Delay5ms(600);
			Inqueue_uart1(0x02);
			//加入命令标志
			Inqueue_uart1(0xFF);
			Inqueue_uart1(0x00);
			Inqueue_uart1(0x1C);	//预热温控
			Inqueue_uart1(0x01);
			for(i=0;i<FrameLength-6;i++)
				Inqueue_uart1(0);
			Inqueue_uart1(0x03);
			Delay5ms(600);
		}
		//自检定时器清零
		SelfCheckCount=0;
		//关闭自检计时
		SelfChecking=OFF; 
	}
	SelfCheckCount=0;
	while(SelfCheckCount==0)
	{
		float	temp1,temp2,temp3,temp4 ;
		temp1 = ADC(ADC_T_10A);	
		temp1 = 3.0 * temp1 / 4096.0 / 2.0;
		temp1 = -232.6 * log(log10(37.4 * 1000.0 * temp1 / (1.2 - temp1))) + 343.82;
		A1=temp1*10;
		temp2=ADC(ADC_T_10B);
		temp2 = 3.0 * temp2 / 4096.0 / 2.0;
		temp2 = -232.6 * log(log10(37.4 * 1000.0 * temp2 / (1.2 - temp2))) + 343.82;
		A2=temp2*10;
		temp3=ADC(ADC_T18);
		temp3 = 3.0 * temp3 / 4096.0;
		temp3 = (((2500.0*4.0*temp3)/(2.5*490.0))+(2500.0*100.0)/2600.0)/(1-((4.0*temp3)/(490.0*2.5))-(100.0/2600.0));
		temp3 = 0.001 * temp3 * temp3 + 2.3589 * temp3 - 245.87;
		A3=temp3*10;
		temp4=ADC(ADC_T42);
		temp4 = 3.0 * temp4 / 4096.0;
		temp4 = (((2500.0*4.0*temp4)/(490.0*2.5))+(2500.0*100.0)/2600.0)/(1-((4.0*temp4)/(490.0*2.5))-(100.0/2600.0));
		temp4 = (temp4 - 100.0) / 0.385 - 1;
		A4=temp4*10;
		if(A1>85&&A1<110&&A2>85&&A2<110&&A3>150&&A3<250&&A4>360&&A4<500)
		{
			SelfCheckCount=10;
			print_temperature(1);
		}
		else 
		{
			if(i)
			{
				print_temperature(0);
				i=0;
			}			
		}	
	}	
}
void ChopperCheck(u8 run)
{
    u8 i;
	if(m_bChopper==1)
        return;
    QueryFlag	= 1;
	Inqueue(0x02);
    Inqueue(0xFF);
    Inqueue(0x00);
    Inqueue(0x20);
    Inqueue(0x00);
    for(i=0; i<FrameLength-6; i++)
        Inqueue(0);
    Inqueue(0x03);
    Delay5ms(200);

	QueryFlag_uart1	= 1;
	Inqueue_uart1(0x02);
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    Inqueue_uart1(0x20);
    Inqueue_uart1(0x00);
    for(i=0; i<FrameLength-6; i++)
        Inqueue_uart1(0);
    Inqueue_uart1(0x03);
    Delay5ms(200);

    ChopperCheck1(run);	 

    if(m_bChopper==0)
    {
		Sys_Status=Sys_Error;
        QueryFlag	= 1;
        //加入帧头
        Inqueue(0x02);
        //加入命令标志
        Inqueue(0xFF);
        Inqueue(0x00);
        Inqueue(0x30);
        Inqueue(0x02);
        for(i=0; i<FrameLength-6; i++)
            Inqueue(0);
        Inqueue(0x03);
        ToMeasPosition[0]=0;
        Delay5ms(100);
				QueryFlag_uart1	= 1;
        //加入帧头
        Inqueue_uart1(0x02);
        //加入命令标志
        Inqueue_uart1(0xFF);
        Inqueue_uart1(0x00);
        Inqueue_uart1(0x30);
        Inqueue_uart1(0x02);
        for(i=0; i<FrameLength-6; i++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x03);
		Delay5ms(100);
		//加入帧头
        Inqueue(0x02);
        //加入命令标志
        Inqueue(0xFF);
        Inqueue(0x00);
        Inqueue(0x30);
        Inqueue(0x02);
        for(i=0; i<FrameLength-6; i++)
            Inqueue(0);
        Inqueue(0x03);
		Delay5ms(300);

		//加入帧头
        Inqueue_uart1(0x02);
        //加入命令标志
        Inqueue_uart1(0xFF);
        Inqueue_uart1(0x00);
        Inqueue_uart1(0x30);
        Inqueue_uart1(0x02);
        for(i=0; i<FrameLength-6; i++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x03);
		Delay5ms(300);
//      LightModulation(OFF);
        while(1)
        {	
			if((ToMeasPosition[0]==11)||(ToMeasPosition[0]==22)||(ToMeasPosition[0]==33)||(ToMeasPosition[0]==44)||(ToMeasPosition[0]==55)||(ToMeasPosition[0]==66))
            {
                QueryFlag	= 1;
                //加入帧头
                Inqueue(0x02);
                //加入命令标志
                Inqueue(0xFF);
                Inqueue(0x00);
                Inqueue(0x30);
                Inqueue(0x02);
                for(i=0; i<FrameLength-6; i++)
                    Inqueue(0);
                Inqueue(0x03);
                ToMeasPosition[0]=0;
                Delay5ms(300);

				QueryFlag_uart1	= 1;
                //加入帧头
                Inqueue_uart1(0x02);
                //加入命令标志
                Inqueue_uart1(0xFF);
                Inqueue_uart1(0x00);
                Inqueue_uart1(0x30);
                Inqueue_uart1(0x02);
                for(i=0; i<FrameLength-6; i++)
                    Inqueue_uart1(0);
                Inqueue_uart1(0x03);
                Delay5ms(300);
            }
            if(m_bMeasureBreak!=0)
            {
                m_bMeasureBreak=0;
                //LightModulation(OFF);
                NVIC_GenerateSystemReset();
                break;
            }
        }
    }
}
void ChopperCheck1(u8 run)
{
    u8	m_bRestart= 1;
    u16  i;
    u8	times=0;

    while(m_bRestart)
    {
        LightModulation(ON);
        Delay5ms(1000);
        AK_Conv();
        for(i=0; i<256; i++)
        {
            lBUFIN[i] = AK_Left_Data[1200+i*2]<<16;
        }
        cr4_fft_256_stm32(lBUFOUT,lBUFIN,NPT);
        powerMag(NPT,"1SIDED");
        for(i=0; i<32; i++)
        {
            if(lBUFMAG[i]>(60*(6+times)))
            {
                m_bRestart	= 1;
                times++;
				break;
            }
            else
                m_bRestart	= 0;
        }
        if(lBUFMAG[32]<1280)
        {
            m_bRestart	= 1;
            times++;
        }
        if(times>4)
        {
            LightModulation(OFF);
            return;
        }
    }
    if(run!=1)
    {
        LightModulation(OFF);
    }
    QueryFlag	= 1;
    //加入帧头
    Inqueue(0x02);
    //加入命令标志
    Inqueue(0xFF);
    Inqueue(0x00);
    Inqueue(0x30);
    Inqueue(0x01);
    Inqueue(times);
    for(i=0; i<FrameLength-7; i++)
        Inqueue(0);
    Inqueue(0x03);
	Delay5ms(10);

	    QueryFlag_uart1	= 1;
    //加入帧头
    Inqueue_uart1(0x02);
    //加入命令标志
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    Inqueue_uart1(0x30);
    Inqueue_uart1(0x01);
    Inqueue_uart1(times);
    for(i=0; i<FrameLength-7; i++)
        Inqueue_uart1(0);
    Inqueue_uart1(0x03);
	Delay5ms(10);
}

void PerkCheck(void)
{
    u8 i;
    u32 Intensityair;
    u32 Intensityref;
    s16 ratioc12 ;
    ChopperCheck(1);
    m_bBusing=1;
    QueryFlag	= 1;
    //加入帧头
    Inqueue(0x02);
    //加入命令标志
    Inqueue(0xFF);
    Inqueue(0x00);
    Inqueue(0x25);	 //过滤器检测
    for(i=0; i<FrameLength-5; i++)
        Inqueue(0);
    Inqueue(0x03);

	 QueryFlag_uart1	= 1;
    //加入帧头
    Inqueue_uart1(0x02);
    //加入命令标志
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    Inqueue_uart1(0x25);	 //过滤器检测
    for(i=0; i<FrameLength-5; i++)
        Inqueue_uart1(0);
    Inqueue_uart1(0x03);

    Cartridge(9);
    Intensityair=Intensity1;

    Delay5ms(10);
    Cartridge(8);
    Intensityref=Intensity1;
    ratioc12 = (s16)((double)Intensityair / (double)Intensityref * 2000.0 - 1000.0);
	//collin add20141016
	debug_collin3 = ratioc12;
	//end
	print_filter(ratioc12);
    QueryFlag	= 1;
    //加入帧头
    Inqueue(0x02);
    //加入命令标志
    Inqueue(0xFF);
    Inqueue(0x00);
    Inqueue(0x35);	 //过滤器检测
    Inqueue(ratioc12&0xFF);
    Inqueue(ratioc12>>8&0xFF);
    for(i=0; i<FrameLength-7; i++)
        Inqueue(0);
    Inqueue(0x03);

	    QueryFlag_uart1	= 1;
    //加入帧头
    Inqueue_uart1(0x02);
    //加入命令标志
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    Inqueue_uart1(0x35);	 //过滤器检测
    Inqueue_uart1(ratioc12&0xFF);
    Inqueue_uart1(ratioc12>>8&0xFF);
    for(i=0; i<FrameLength-7; i++)
        Inqueue_uart1(0);
    Inqueue_uart1(0x03);

    m_bBusing=0;
}

void PowerCheck(void)
{
    u8	i;
    u8 j;
    ChopperCheck(1);

    //报告工作状态
    QueryFlag	= 1;
    //加入帧头
    Inqueue(0x02);
    //加入命令标志
    Inqueue(0xFF);
    Inqueue(0x00);
    Inqueue(0x23);	 //光强检测
    for(i=0; i<FrameLength-5; i++)
        Inqueue(0);
    Inqueue(0x03);
    Delay5ms(10);
    Valve(171);
    Valve(181);

	//报告工作状态
    QueryFlag_uart1	= 1;
    //加入帧头
    Inqueue_uart1(0x02);
    //加入命令标志
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    Inqueue_uart1(0x23);	 //光强检测
    for(i=0; i<FrameLength-5; i++)
        Inqueue_uart1(0);
    Inqueue_uart1(0x03);
    Delay5ms(10);

	flag_count=1;
    for(i=0; i<2; i++)
    {
        Motorun(0,3000,FORWARD,0,  20,60,8,20, 30,5,20,0,    81,150,0);      //抽气
        Motorun(0,3000,BACKWARD,0, 20,60,8,20, 20,4,50,200,  81,151,161);      //打气,清洗

        Valve(160);
    }
    Motorun(0,4000,FORWARD,0,      20,60,8,20,  30,5,20,0,   81,150,0);      //抽气
    Delay5ms(100);
    Motorun(0,4000,BACKWARD,0,     30,60,8,20,  30,1,20,1200,81,151,160);      //打气，慢推1~3r/s，
    //	Delay5ms(100);
    Valve(150);
    Valve(161);
    //for(i=0; i<60; i++)
	for(;;)
    {
        AK_Conv();	
//		AK_Left_Data[8]	= 0x07+(i+1)*256;
//		SendData();
//		Delay5ms(100);
		C12[i]=Intensity1;
		C13[i]=Intensity2;

		if(Intensity1>=99999||Intensity2>=99999)
		{				
		}
		else
		{
        }
		
		
		QueryFlag	= 1;		
        //加入帧头
        Inqueue(0x02);
        //加入命令标志
        Inqueue(0x01);
        Inqueue(Intensity1&0xff);
        Inqueue((Intensity1>>8)&0xff);
        Inqueue((Intensity1>>16)&0xff);
        Inqueue((Intensity1>>24)&0xff);
        Inqueue(Intensity2&0xff);
        Inqueue((Intensity2>>8)&0xff);
        Inqueue((Intensity2>>16)&0xff);
        Inqueue((Intensity2>>24)&0xff);
        for(j=0; j<8; j++)
            Inqueue(0);
        Inqueue(i+1);//buf18
        Inqueue(0x07);//buf19
        for(j=0; j<3; j++)
            Inqueue(0);
        Inqueue(0x03);
		Delay5ms(100);

				QueryFlag_uart1	= 1;		
        //加入帧头
        Inqueue_uart1(0x02);
        //加入命令标志
        Inqueue_uart1(0x01);
        Inqueue_uart1(Intensity1&0xff);
        Inqueue_uart1((Intensity1>>8)&0xff);
        Inqueue_uart1((Intensity1>>16)&0xff);
        Inqueue_uart1((Intensity1>>24)&0xff);
        Inqueue_uart1(Intensity2&0xff);
        Inqueue_uart1((Intensity2>>8)&0xff);
        Inqueue_uart1((Intensity2>>16)&0xff);
        Inqueue_uart1((Intensity2>>24)&0xff);
        for(j=0; j<8; j++)
            Inqueue_uart1(0);
        Inqueue_uart1(i+1);//buf18
        Inqueue_uart1(0x07);//buf19
        for(j=0; j<3; j++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x03);
		Delay5ms(100);

		print_power(i+1,1);

        if(m_bMeasureBreak==0)
        {
            Delay5ms(1750);
        }
		else
		{		
	  		m_bMeasureBreak=0;
	        break;
		}
    }
	flag_count=0;
    Valve(151);
    Valve(160);					   
	
    //报告工作状态
    QueryFlag	= 1;
    //加入帧头
    Inqueue(0x02);
    //加入命令标志
    Inqueue(0xFF);
    Inqueue(0x00);
    Inqueue(0x33);	 //光强检测结束
    for(i=0; i<FrameLength-5; i++)
        Inqueue(0);
    Inqueue(0x03);
    Delay5ms(100);

	    //报告工作状态
    QueryFlag_uart1	= 1;
    //加入帧头
    Inqueue_uart1(0x02);
    //加入命令标志
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    Inqueue_uart1(0x33);	 //光强检测结束
    for(i=0; i<FrameLength-5; i++)
        Inqueue_uart1(0);
    Inqueue_uart1(0x03);
    Delay5ms(100);
}
void MainInterface(void)
{	
	u8 i;
	if(SAMPLE1==0)
	{	 
		sampleOn|=0x01;
	}
	else 
	{
		sampleOn&=0xFE;
		ch_selected&=0xFE;
	}
		 
}

//collin add20140917
void query_printProcess(void)
{
	INQUIRY_PRINT = 0;
	SPI_FLASH_BufferRead(ID,0x010000+DiapIndex*30,30);
	print_result_search();
}
//end

