// Modbus_RTU Functions
#include "main.h"
//#include "stm32f1xx_hal_rcc.h"
//#include "stm32f1xx_hal_gpio.h"
//#include "stm32f1xx_hal_usart.h"
//#include "stm32f1xx_hal_conf.h"

unsigned int    n_com1, line1;
unsigned char   data_com1[com_max];
extern float PU[4],AU[4],Kp[4],Ks[4];
extern uint8_t typeSens,CommRelay,WorkRelay;
extern uint8_t MB_ADRESS_M;
extern uint32_t	BAUD_RATE_M;
extern uint8_t ResetRelay, typeT;
extern uint16_t tROn1, tROff1, tROn2, tROff2, Uu, Ud, Ktok1, Btok1, Ktok2, Btok2;
extern uint32_t TVib, NVib;

//#define MB_ADRESS_M       0x01                   //Modbus device adress
//#define BAUD_RATE_M       115200                   //USART baud rate value

//Connecting pins
#define USART1_RTS_PIN          GPIO_Pin_12    //USART RTS pin
#define USART1_TX_PIN           GPIO_Pin_9    //USART TX pin
#define USART1_RX_PIN           GPIO_Pin_10    //USART RX pin
#define USART1_port             GPIOA         //USART port
#define RTS1_SET                GPIO_SetBits(USART1_port, USART1_RTS_PIN)
#define RTS1_RESET              GPIO_ResetBits(USART1_port, USART1_RTS_PIN)
// =============================================================================================================
void init_S(void) //Init USART1
{
USART_InitTypeDef   USART_InitStruct;
GPIO_InitTypeDef  GPIO_InitStructure; 
GPIO_StructInit(&GPIO_InitStructure);//Init deaful structure
  
  // GPIOA.12
GPIOA->CRH &= ~GPIO_CRH_MODE12;  //i?enoeou ?ac?yau MODE
GPIOA->CRH &= ~GPIO_CRH_CNF12;   //i?enoeou ?ac?yau CNF
GPIOA->CRH |=  GPIO_CRH_MODE12_0;//auoia, 10MHz
GPIOA->CRH &= ~GPIO_CRH_CNF12;   //iauaai iacia?aiey, neiao?e?iue
    
//Включение тактирования
  RCC->APB2ENR |=   RCC_APB2ENR_IOPAEN;                //Тактирование GPIO
  RCC->APB2ENR |=   RCC_APB2ENR_AFIOEN;                //Тактирование альтернативных функций GPIO
  RCC->APB2ENR |=   RCC_APB2ENR_USART1EN;              //Тактирование USART1
 
  //Конфигурирование PORTA.9 для TX; PORTA.10 для RX 
  GPIOA->CRH   &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);   //Предочистка MODE и CNF
  GPIOA->CRH   |=   GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1;  //Двухтактный выход с альтернативной ф-ей, 50MHz
  GPIOA->CRH   &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10); //Предочистка MODE и CNF
  GPIOA->CRH   |=   GPIO_CRH_CNF10_0;                  //Вход, третье состояние
  RTS1_RESET; 
  
  USART_InitStruct.USART_BaudRate = BAUD_RATE_M;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_HardwareFlowControl = 0;//USART_HardwareFlowControl_RTS;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStruct); //Set USART1 structure

  //Управление работой
  USART1->CR1  |=   USART_CR1_UE;                      //Включение модуля USART1
  //Разрешить прерывания
  NVIC_EnableIRQ (USART1_IRQn);                        //Прерывания USART1
  USART1->CR1  |= USART_CR1_TCIE;                      //Прерывание по завершении передачи
  USART1->CR1  |= USART_CR1_RXNEIE;                    //Прерывание по завершении приема
	
	NVIC_SetPriority (USART1_IRQn, 0);
  
  USART1->CR1  &=  ~USART_CR1_TE;              //Off передатчика
  USART1->CR1  |=   USART_CR1_RE;              //Включение приемника 
}
// =========================================================================================================
void USART1_IRQHandler(void)
{
		if((USART1->SR & USART_SR_RXNE)!=0) //Если причина прерывания окончание приема байта
		{
                if(!(CHECKBIT(bit_S)))  
								{  
										timer_S=6; //20;  
                    if(n_com1>=com_max) 	n_com1=0;      // ошибка 
                    else      {data_com1[n_com1]=USART1->DR;  n_com1++;  }			
                }
		}
    
		if((USART1->SR & USART_SR_TC)!=0) //Если причина прерывания окончание передачи байта  
    {
				USART1->SR &=  ~USART_SR_TC;                      //очистить флаг
				if (n_com1>=line1) 
        {             		// всё передано.
          USART1->CR1  &=  ~USART_CR1_TE;              //Off передатчика  
          USART1->CR1  |=   USART_CR1_RE;              //Включение приемника	  
          RTS1_RESET; 
					n_com1=0; 
          SETBIT(bit_zap);
        }
				else 
				{			 
						USART1->DR=data_com1[n_com1++];    // Отправить байт.
        }
    }
		
		USART1->SR&=0; // на всякий случай
}

// ==============================================================================================================
void CRC16_1 (unsigned int line)         // контрольная сумма RTU
{
		unsigned int j, a, n;
		unsigned int reg;

		reg=0xffff;   a=line;
		for(j=0;j<line;j++)
			{
			reg^=data_com1[j];
			for(n=0; n<8;n++)
					{
					if((reg&0x01)==0x01) reg=(reg>>1)^0xA001;
								else           reg>>=1;
					}
			}
		data_com1[a++]=reg;        // L
		data_com1[a]=reg>>8;       // H
}

// ================================================================================================================
void RTU_S(void)  // обработка MODBUS запроса ************************
{
		unsigned int ad, n, j; 
		unsigned int dat;
		 
				 if(data_com1[0]!= MB_ADRESS_M) {goto m_eror1;}    // мой ID
				 ad=n_com1;   
				 n_com1=0;   
				 if(ad<6) goto m_eror1;             // проверка на длину
				 n=data_com1[ad-1];                 // проверка на CRC16 принятой посылки
				 j=data_com1[ad-2];
				 CRC16_1(ad-2);
				 if((n!=data_com1[ad-1])||(j!=data_com1[ad-2]))   goto m_eror1;   // ошибка CRC
				 
				 timer_LED_2=15;
				 
				 if(data_com1[1]==0x03)          // команда чтения регистров ***********************************
				 {  
							ad=data_com1[2]; ad<<=8; 
							ad|=data_com1[3]; ad++;              // адрес
							n=data_com1[5]<<1;        // длина данных
							data_com1[2]=n;  
							j=3;         
							while(n!=0)
							{
								dat=Holding_reg[ad++];            
								data_com1[j++]=dat>>8;
								data_com1[j++]=dat;
								n-=2;
							}
							CRC16_1(j);
							line1=j+2;          
							goto m_ok1;
				}
		 /*
				 if(data_com1[1]==0x04)          // команда ответа на чтение аналоговых входов ***********************************
							{
							ad=data_com1[2]; ad<<=8;  
							ad|=data_com1[3];                    // адрес 
							n=data_com1[5]<<1;                  // длина данных
							data_com1[2]=n;        
							j=3;  ad++;       
							if(ad<0x0400) {       // 0-3ff   мгновенные значения
														while(n!=0)
																	{
																	dat=Input_reg[ad++];          
																	data_com1[j++]=dat>>8;             // H
																	data_com1[j++]=dat;                // L
																	n-=2;
																	}
														}
										else    {
														if(ad>=0x1000)  { //  > 0x1000 
																						ad-=0x1000;
																						if(buffer[850]!=(ad>>9)) // для повышения скорости -- прочитать один раз
																								{
																								buffer[850]=ad>>9;
																								Load_input_H();
																								}
																						ad&=0x01FF;  ad<<=1;
																						while(n!=0)     { data_com1[j++]=buffer[ad++]; n--;  }
																						}
																		else    {               //0x0400-0x0BFF
																						ad-=0x0400;
																						buffer[0]=ad>>5; // адрес номера
																						Load_input_L();
																						ad&=0x001F; ad<<=1; 
																						while(n!=0)     { data_com1[j++]=buffer[ad++]; n--;  }
																						}
														}        
							CRC16_1(j);
							line1=j+2; 
							goto m_ok1;
							}    
				 */
							
					if(data_com1[1]==0x06)          // команда записи регистра ******************** 
					{
							n=data_com1[2];    n<<=8;  
							n|=data_com1[3];   n++;
							dat=data_com1[4]; dat<<=8;
							dat|=data_com1[5];
							Holding_reg[n]=dat;
							line1=8; 
							
					m_ok1:    
						
							USART1->CR1  &=  ~USART_CR1_RE;              //Off приемника
							USART1->CR1  |=   USART_CR1_TE;              //Включение передатчика 	  
							RTS1_SET;
							USART1->DR=data_com1[n_com1++];                // начали выдачу
							
							if(Holding_reg[PROTECTION]==CODE)  // Доступ к изменению настроек работы
							{
								CheckChages(); // Проверка параметров на изменение
							}
							else
							{
										Holding_reg[Kp0] = Kp[0];
										Holding_reg[Kp1] = Kp[1];
										Holding_reg[Kp2] = Kp[2];
										Holding_reg[Kp3] = Kp[3];
										Holding_reg[Ks0] = Ks[0];
										Holding_reg[Ks1] = Ks[1];
										Holding_reg[Ks2] = Ks[2];
										Holding_reg[Ks3] = Ks[3];
										Holding_reg[Pu0] = PU[0];
										Holding_reg[Pu1] = PU[1];
										Holding_reg[Pu2] = PU[2];
										Holding_reg[Pu3] = PU[3];
										Holding_reg[Au0] = AU[0];
										Holding_reg[Au1] = AU[1];
										Holding_reg[Au2] = AU[2];
										Holding_reg[Au3] = AU[3];
										Holding_reg[CommR] = CommRelay;
										Holding_reg[WorkR] = WorkRelay;
										Holding_reg[SensorType] = typeSens;
										Holding_reg[tRelayOn1] = tROn1;
										Holding_reg[tRelayOff1] = tROff1;
										Holding_reg[tRelayOn2] = tROn2;
										Holding_reg[tRelayOff2] = tROff2;
										Holding_reg[ResetR] = ResetRelay;
										Holding_reg[Uup] = Uu;
										Holding_reg[Udown] = Ud;
										Holding_reg[TViborki] = TVib;
										Holding_reg[NViborki] = NVib;
										Holding_reg[Ttype] 		= typeT;
										Holding_reg[Ktok1m] = Ktok1; 
										Holding_reg[Btok1m] = Btok1; 
										Holding_reg[Ktok2m] = Ktok2;
										Holding_reg[Btok2m] = Btok2;
							}
							
							return;
					}
				 
		m_eror1:  
		
					n_com1=0;
}
// ==========================================================================================================================

void CheckChages()
{
	uint8_t i;
	
	for(i=2;i<45;i++)
	{
		Holding_reg[i] = *(short int*)&Holding_reg[i];// Это нужно, чтобы минус не потерялся при передаче
		
	}
if(Kp[0] != Holding_reg[Kp0] ){f_changes = 1;} // Если изменился хоть один параметр, устанавливается флаг, разрешающих перезапись параметров во FLASH
if(Kp[1] != Holding_reg[Kp1] ){f_changes = 1;}
if(Kp[2] != Holding_reg[Kp2] ){f_changes = 1;}
if(Kp[3] != Holding_reg[Kp3] ){f_changes = 1;}
if(Ks[0] != Holding_reg[Ks0] ){f_changes = 1;}
if(Ks[1] != Holding_reg[Ks1] ){f_changes = 1;}
if(Ks[2] != Holding_reg[Ks2] ){f_changes = 1;}
if(Ks[3] != Holding_reg[Ks3] ){f_changes = 1;}
if(PU[0] != Holding_reg[Pu0] ){f_changes = 1;}
if(PU[1] != Holding_reg[Pu1] ){f_changes = 1;}
if(PU[2] != Holding_reg[Pu2] ){f_changes = 1;}
if(PU[3] != Holding_reg[Pu3] ){f_changes = 1;}
if(AU[0] != Holding_reg[Au0] ){f_changes = 1;}
if(AU[1] != Holding_reg[Au1] ){f_changes = 1;}
if(AU[2] != Holding_reg[Au2] ){f_changes = 1;}
if(AU[3] != Holding_reg[Au3] ){f_changes = 1;}
if(CommRelay	!= Holding_reg[CommR] ){f_changes = 1;}
if(WorkRelay	!= Holding_reg[WorkR] ){f_changes = 1;}
if(typeSens		!= Holding_reg[SensorType] ){f_changes = 1;}
if(tROn1			!= Holding_reg[tRelayOn1] ){f_changes = 1;}
if(tROff1			!= Holding_reg[tRelayOff1] ){f_changes = 1;}
if(tROn2			!= Holding_reg[tRelayOn2] ){f_changes = 1;}
if(tROff2			!= Holding_reg[tRelayOff2] ){f_changes = 1;}
if(ResetRelay	!=	Holding_reg[ResetR]){f_changes = 1;}
if(Uu					!=	Holding_reg[Uup]){f_changes = 1;}
if(Ud					!=	Holding_reg[Udown]){f_changes = 1;}
if(TVib				!=	Holding_reg[TViborki]){f_changes = 1;}
if(NVib				!=	Holding_reg[NViborki]){f_changes = 1;}
if(Ktok1			!=	Holding_reg[Ktok1m]){f_changes = 1;}
if(Btok1			!=	Holding_reg[Btok1m]){f_changes = 1;}
if(Ktok2			!=	Holding_reg[Ktok2m]){f_changes = 1;}
if(Btok2			!=	Holding_reg[Btok2m]){f_changes = 1;}

Kp[0]				=	Holding_reg[Kp0];
Kp[1]				=	Holding_reg[Kp1];
Kp[2]				=	Holding_reg[Kp2];
Kp[3]				=	Holding_reg[Kp3];
Ks[0]				=	Holding_reg[Ks0];
Ks[1]				=	Holding_reg[Ks1];
Ks[2]				=	Holding_reg[Ks2];
Ks[3]				=	Holding_reg[Ks3];
PU[0]				=	Holding_reg[Pu0];
PU[1]				=	Holding_reg[Pu1];
PU[2]				=	Holding_reg[Pu2];
PU[3]				= Holding_reg[Pu3];
AU[0]				=	Holding_reg[Au0];
AU[1]				=	Holding_reg[Au1];
AU[2]				=	Holding_reg[Au2];
AU[3]				=	Holding_reg[Au3];
CommRelay		=	Holding_reg[CommR];
WorkRelay		=	Holding_reg[WorkR];
typeSens		=	Holding_reg[SensorType];
tROn1				=	Holding_reg[tRelayOn1];
tROff1			=	Holding_reg[tRelayOff1];
tROn2				=	Holding_reg[tRelayOn2];
tROff2			=	Holding_reg[tRelayOff2];
ResetRelay	=	Holding_reg[ResetR];
Uu					=	Holding_reg[Uup];
Ud					=	Holding_reg[Udown];
TVib 				= Holding_reg[TViborki];
NVib 				= Holding_reg[NViborki];
typeT 			= Holding_reg[Ttype];
Ktok1				=	Holding_reg[Ktok1m];
Btok1				=	Holding_reg[Btok1m];
Ktok2				=	Holding_reg[Ktok2m];
Btok2				=	Holding_reg[Btok2m];

}


/*
 



*/
