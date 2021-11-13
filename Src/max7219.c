#include "max7219.h"

char dg=8;

#define cs_set() LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8)
#define cs_reset() LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8)

void Send_7219 (uint8_t rg, uint8_t dt)
{
	cs_set();
  while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {}
  LL_SPI_TransmitData16 (SPI1, (uint16_t)rg<<8 | dt);
  while(!LL_SPI_IsActiveFlag_RXNE(SPI1)) {}
  (void) SPI1->DR;
	cs_reset();
}
//------------------------------------------------------
void Clear_7219 (void)
{
	uint8_t i=dg;
	do
	{
		Send_7219(i,0xF);//������ �������
	} while (--i);
}
//------------------------------------------------------
void Number_7219 (volatile long n)
{
	uint8_t ng=0;//���������� ��� ������
	if(n<0)
	{
		ng=1;
		n*=-1;
	}
	uint8_t i=0;
	do
	{
		Send_7219(++i,n%10);//������ �����
		n/=10;
	} while(n);
	if(ng)
	{
		Send_7219(i+1,0x0A);//������ -
	}
}
//-------------------------------------------------------
void NumberL_7219 (volatile int n) //����� � ����� �������
{
  uint8_t ng=0;//���������� ��� ������
  if(n<0)
  {
    ng=1;
    n*=-1;
  }
  uint8_t i=4;
  if(n<1000) Send_7219(8,0xF);//символ пустоты
  if(n<100) Send_7219(7,0xF);//символ пустоты
  if(n<10) Send_7219(6,0xF);//символ пустоты
  do
  {
    Send_7219(++i,n%10);//������ �����
    n/=10;
  } while(n);
  if(ng)
  {
    Send_7219(i+1,0x0A);//������ -
  }
}
//-------------------------------------------------------
void NumberR_7219 (volatile int n)
{
	uint8_t ng=0;//переменная для минуса
	if(n<0)
	{
		ng=1;
		n*=-1;
	}
	uint8_t i=0;
  if(n<1000) Send_7219(4,0xF);//символ пустоты
  if(n<100) Send_7219(3,0xF);//символ пустоты
  if(n<10) Send_7219(2,0xF);//символ пустоты
	do
	{
		Send_7219(++i,n%10);//символ цифры
		n/=10;
	} while(n);
	if(ng)
	{
		Send_7219(i+1,0x0A);//символ -
	}
}
//-------------------------------------------------------
void NumberF_7219 (float f) //����� � ����� ������� c ���������� ������
{
  int n = (int)(f * 10);
  uint8_t ng=0;//���������� ��� ������
  if(n<0)
  {
    ng=1;
    n*=-1;
  }
  int m = n;
  uint8_t i=0;
  do
  {
    if(i==1) Send_7219(++i,(n%10) | 0x80);//������ ����� � �����
    else Send_7219(++i,n%10);//������ �����
    n/=10;
  } while(n);
  if(ng)
  {
    if(m<10)
    {
      Send_7219(i+1,0x80);//���� � ������
      Send_7219(i+2,0x0A);//������ -
    }
    else Send_7219(i+1,0x0A);//������ -
  }
  else
  {
    if((m<10)&&(m!=0))
    {
      Send_7219(i+1,0x80);//���� � ������
    }
  }
}
//-------------------------------------------------------
void NumberLF_7219 (float f) //����� � ����� ������� c ���������� ������
{
  int n = (int)(f * 10);
  uint8_t ng=0;//���������� ��� ������
  if(n<0)
  {
    ng=1;
    n*=-1;
  }
  int m = n;
  uint8_t i=4;
  do
  {
    if(i==5) Send_7219(++i,(n%10) | 0x80);//������ ����� � �����
    else Send_7219(++i,n%10);//������ �����
    n/=10;
  } while(n);
  if(ng)
  {
    if(m<10)
    {
      Send_7219(i+1,0x80);//���� � ������
      Send_7219(i+2,0x0A);//������ -
    }
    else Send_7219(i+1,0x0A);//������ -
  }
  else
  {
    if((m<10)&&(m!=0))
    {
      Send_7219(i+1,0x80);//���� � ������
    }
  }
}
//-------------------------------------------------------
void Init_7219 (void)
{
		Send_7219(0x09,0xFF);//������� ����� �������������
		Send_7219(0x0B,dg-1);//���-�� ������������ ��������
		Send_7219(0x0A,0x06);//������������� ��������
		Send_7219(0x0C,0x01);//������� ���������
		Clear_7219();
}
