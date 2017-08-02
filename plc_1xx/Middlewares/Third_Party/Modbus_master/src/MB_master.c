// Modbus_RTU Functions
#include "main.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_conf.h"

//uint8_t error;
unsigned int    n_com2, line2;
unsigned char   data_com[com_max];

#define MB_ADRESS       0x01                   //Modbus device adress
#define BAUD_RATE       115200                   //USART baud rate value

//Connecting pins
#define USART2_RTS_PIN          GPIO_Pin_1    //USART RTS pin
#define USART2_TX_PIN           GPIO_Pin_2    //USART TX pin
#define USART2_RX_PIN           GPIO_Pin_3    //USART RX pin
#define USART2_port             GPIOA         //USART port
#define RTS_SET                 GPIO_SetBits(USART2_port, USART2_RTS_PIN)
#define RTS_RESET               GPIO_ResetBits(USART2_port, USART2_RTS_PIN)
// =============================================================================================================
void init_M(void) //Init USART2
{
  USART_InitTypeDef   USART_InitStruct;
  GPIO_InitTypeDef  GPIO_InitStructure; 
  GPIO_StructInit(&GPIO_InitStructure);//Init deaful structure
  
  RCC->APB2ENR |=   RCC_APB2ENR_AFIOEN;                //альтернативные функции GPIO
  RCC->APB1ENR |=   RCC_APB1ENR_USART2EN;              //тактирование USART2
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                          
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  // PORTA.2 TX; PORTA.3  RX 
  GPIOA->CRL   &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2);   
  GPIOA->CRL   |=   GPIO_CRL_MODE2 | GPIO_CRL_CNF2_1;  
  GPIOA->CRL   &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3);   
  GPIOA->CRL   |=   GPIO_CRL_CNF3_0;                    
   //Init USART2
  USART_InitStruct.USART_BaudRate = BAUD_RATE;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_HardwareFlowControl = 0;//USART_HardwareFlowControl_RTS;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStruct); //Set USART2 structure
  //  управление
  USART2->CR1  |=   USART_CR1_UE;                      //включить USART2
  // прерывания   
  NVIC_EnableIRQ (USART2_IRQn);                        //прерывание USART2
  USART2->CR1  |= USART_CR1_TCIE;                      //по завершении передачи
  USART2->CR1  |= USART_CR1_RXNEIE;                    //по приему
  
  USART2->CR1  &=  ~USART_CR1_TE;              //Off передатчика
  USART2->CR1  |=   USART_CR1_RE;              //Включение приемника
  RTS_RESET; 
  
}
// ======================================================================================================================
void USART2_IRQHandler(void)
{
if((USART2->SR & USART_SR_RXNE)!=0) //Если причина прерывания окончание приема байта
		{
                if(!(CHECKBIT(bit_M)))  {  
                                        timer_M=6; //20;  
                                        if(n_com2>=com_max) 	n_com2=0;      // ошибка 
                                                      else      {data_com[n_com2]=USART2->DR;  n_com2++;  }
                                                      
                                        }
		}
    
if((USART2->SR & USART_SR_TC)!=0) //Если причина прерывания окончание передачи байта  
    {
    USART2->SR &=  ~USART_SR_TC;                      //очистить флаг
    if (n_com2>=line2) 
          {             		// всё передано.
          USART2->CR1  &=  ~USART_CR1_TE;              //Off передатчика  
          USART2->CR1  |=   USART_CR1_RE;              //Включение приемника	  
          RTS_RESET; 
	  n_com2=0; 
          }
     else {
          USART2->DR=data_com[n_com2++];    // Отправить байт.       
          }
    }
USART2->SR&=0; // на всякий случай
} 
// ===========================================================================================================================
void CRC16_M (unsigned int line)         // контрольная сумма RTU
{ 
uint8_t n;  
uint16_t j, r;

r=0xFFFF;   
for(j=0;j<line;j++)
  { 
  r^=data_com[j];
  for(n=0; n<8;n++)
      {
       if(r&0x0001)     r=(r>>1)^0xA001;  //0xA001; }
              else      r>>=1; 
      }
  }
data_com[line++]=r;        // L
data_com[line]=r>>8;       // H
}
// ================================================================================================================
void RTU_M(void)  // обработка MODBUS запроса ************************
{
unsigned int ad, n, j; 
unsigned int dat;

     if(data_com[0]!= MB_ADRESS) goto m_eror;    // мой ID
     ad=n_com2;   
     n_com2=0;   
     if(ad<6) goto m_eror;         // проверка на длину
     n=data_com[ad-1];                 // проверка на CRC16 принятой посылки
     j=data_com[ad-2];
     CRC16_M(ad-2);
     if((n!=data_com[ad-1])||(j!=data_com[ad-2]))   goto m_eror;   // ошибка CRC
     
     timer_LED_1=15;   
     if(data_com[1]==0x03)          // прием ответа на команду чтения регистров ***********************************
          {
          j=3;  ad=(data_com[2]>>1)+1;
          for(n=2; n<ad;n++)  
                  {          
                  dat=data_com[j++]; dat<<=8;       // H
                  dat|=data_com[j++];               // L
                  if(n<9) Holding_reg[n]=dat; 
                  }  
          rez_com++;    // следующая команда
          }

     if(data_com[1]==0x04)   // прием ответа на команду чтения аналоговых входов ******
          {
          
          ad=0;
          if(rez_com==3) ad=120;
          if(rez_com==5) ad=240;
          if(rez_com==7) ad=360;
          
          j=3;   ad++;
          if(CHECKBIT(bit_zap))  // ============================================
              {
              for(n=0; n<data_com[2];n+=2)
                  {        
                  dat=data_com[j++]; dat<<=8;       // H
                  dat|=data_com[j++];               // L
                  Input_reg[ad++]=dat; 
                  }
              }
                     // ========================================================
          rez_com++; // следующая команда
          }    

m_eror:  
     n_com2=0;
}
// ==========================================================================================================
void zapros_M(void)
{  
data_com[0]=0x01;
data_com[2]=0x00;  
data_com[3]=0;  
data_com[4]=0x00;

if(rez_com<10)
      {                 // Input load
      data_com[1]=0x04;      
      data_com[5]=120; 
      if(rez_com==3) data_com[3]=120;         // выбор начального адреса и длины
      if(rez_com==5) data_com[3]=240;
      if(rez_com==7) {data_com[2]=1; data_com[3]=0x68; data_com[5]=50;}
      }
if(rez_com==10)
      {                 // Holding load
      data_com[1]=0x03; 
      data_com[2]=0;
      data_com[3]=0x01;
      data_com[4]=0;
      data_com[5]=0x0A;   
      }
if(rez_com==20)         // Holding save
      {
      data_com[1]=0x06;
      data_com[2]=0;
      data_com[3]=Holding_reg[COMAND]&0x0f;     // адрес
      data_com[4]=Holding_reg[COMAND+1]>>8;     // данные
      data_com[5]=Holding_reg[COMAND+1]&0xff;
      Holding_reg[COMAND]=0;                    // стереть все
      Holding_reg[COMAND+1]=0;
      }
CRC16_M(6); 
line2=8; n_com2=0;
USART2->CR1  &=  ~USART_CR1_RE;              //Off приемника
USART2->CR1  |=   USART_CR1_TE;              //Включение передатчика 	  
RTS_SET; 
USART2->DR=data_com[n_com2++];                // начали выдачу
}
// ==========================================================================================================


