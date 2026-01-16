
#include <stdio.h>
#include "M051Series.h"
#include "string.h"
#include "PtsPta_Global.h"


unsigned int ADC2Voltage(unsigned int data)
{

	unsigned int  Value;

	Value = ((500 * data )/4095);

	return Value;

}



void GetADCValue(void)
{
static unsigned int  S_u32VoltageTemp[30];   
static unsigned char i_Arry = 0 , ArryFull = 0;

unsigned char i = 0 ;
unsigned int Sum=0, L_StarInAdc_Tmp = 0;


    unsigned int u32ChannelCount ;
    unsigned int   u32ConversionData;
    	     ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            /* start A/D conversion */
            ADC_START_CONV(ADC);

            /* Wait conversion done */
            while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT));

            /* clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            for(u32ChannelCount = 0; u32ChannelCount < 8; u32ChannelCount++)
            {
               	 u32ConversionData = ADC_GET_CONVERSION_DATA(ADC, u32ChannelCount);

			if(u32ChannelCount == 0)
			{
				G_Current = u32ConversionData;
				gu32Current =  (u32ConversionData * CURR_FACTOR_0R0005)/1000;

				if(gu32MaxCurrent < G_Current)
					gu32MaxCurrent = G_Current;
				
			}				 
			else if(u32ChannelCount == 1)
			{
				//G_StartInAdc = ADC2Voltage(u32ConversionData);
				//G_StartInAdc = 270	;	
					L_StarInAdc_Tmp = ADC2Voltage(u32ConversionData);
					//if(G_StartInAdc -L_StarInAdc_Tmp > 5    ///||  (L_StarInAdc_Tmp - G_StartInAdc)  > 5)
					if   (      ((L_StarInAdc_Tmp > G_StartInAdc) && L_StarInAdc_Tmp - G_StartInAdc >2) ||  ((G_StartInAdc > L_StarInAdc_Tmp) && G_StartInAdc - L_StarInAdc_Tmp >2))
					{
						G_StartInAdc = L_StarInAdc_Tmp;
					}

//					G_StartInAdc = 271;
			
			}				 
			else if(u32ChannelCount == 6)
			{
//#if ( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm )				
//				 = (u32ConversionData);
//#endif
			}
			else if(u32ChannelCount == 3)
			{
				G_NTC_PCB = u32ConversionData;
			 	
			}
			else if(u32ChannelCount == 5)
			{	/*
				Duty		10	15	20	25	30	35	40	45	50	55	60	65	70	75	80	85	90	95	100
				Current			0.56	0.67	0.75	0.85	0.93	1.01	1.07	1.13	1.17	1.22	1.25	1.30	1.35	1.40	1.46	1.54	1.58	
				ADC		62	85	109	133	153	178	195	212	223	235	245	254	263	273	283	297	310	325	337
				*/

			
				G_NTC_Motor = u32ConversionData;



			}
			else if(u32ChannelCount == 2)
			{					
			//	G_Voltage_Battery = (((ADC2Voltage(u32ConversionData))*1000)/250);//ADC2Voltage(i32ConversionData);  
			
	//			printf("ADC6= %d, ",u32ConversionData);
 	//			printf("%d, ",(ADC2Voltage(u32ConversionData)));
 	//			printf("%d\n",((ADC2Voltage(u32ConversionData)*100000)/33333));
       //			G_Voltage_Battery = ((ADC2Voltage(u32ConversionData)*100000)/33333);	

				if(ArryFull == 0)
				{
		
					#if ( ScrewType == _BS_1Nm2)
						S_u32VoltageTemp[i_Arry] = ((ADC2Voltage(u32ConversionData)*100000)/33333);
					#else

						/* 51K 與 12K 分壓 */								
						S_u32VoltageTemp[i_Arry] = (((ADC2Voltage(u32ConversionData))*1000)/190);
					#endif

					i_Arry++;

					if(i_Arry == 30)					
						ArryFull = 1;
					/* Array 電壓還未抓滿的時候 直接讀取,避免第一次上電 壓下 壓板啟動會頓一下*/ 
					#if ( ScrewType == _BS_1Nm2)
						G_Voltage_Battery = ((ADC2Voltage(u32ConversionData)*100000)/33333); 
					#else
						/* 51K 與 12K 分壓 */
						G_Voltage_Battery = (((ADC2Voltage(u32ConversionData))*1000)/190);
						G_Voltage_Battery = G_Voltage_Battery +70;	
					#endif

	

				}
				else
				{
					/* Array 電壓 Shift 往前一格 */ 	
					for(i = 1 ; i <30  ; i++)
						S_u32VoltageTemp[i-1] = S_u32VoltageTemp[i];		

					/* 最新的電壓 存放在 最後一格 */ 	
					#if ( ScrewType == _BS_1Nm2)
						S_u32VoltageTemp[29] = ((ADC2Voltage(u32ConversionData)*100000)/33333);						
					#else
   						/* 51K 與 12K 分壓 */
						S_u32VoltageTemp[29] = (((ADC2Voltage(u32ConversionData))*1000)/190);
					#endif
					


					for(i = 0 ; i < 30 ; i++)
					 	Sum =  Sum +  S_u32VoltageTemp[i+1];

					G_Voltage_Battery = Sum / 30;
					
					G_Voltage_Battery = G_Voltage_Battery + 140;							
				}

//			G_Voltage_Battery = ((ADC2Voltage(u32ConversionData)*100000)/33333);	
			
			}
            }


            /* Wait conversion done */
            while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT));

            /* Stop A/D conversion */
            ADC_STOP_CONV(ADC);
	





}

