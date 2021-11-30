#include "CyberLib.h"
//*************************************************��� �� 328 ����������**********************************
//********************************************************************************************************
//********************************************************************************************************
#if defined (__AVR_ATmega328__) || defined (__AVR_ATmega328P__) || defined (__AVR_ATmega168__)
//**********Small UART****************************
#define UART_DOUBLESPEED
#define UART_TXREADY UDRE0
#define UART_RXREADY RXC0
#define UART_DOUBLE	U2X0
#define UDR  UDR0
#define UCRA UCSR0A
#define UCRB UCSR0B
#define UCRC UCSR0C
#define UCRC_VALUE ((1<<UCSZ01) | (1<<UCSZ00))
#define RXCIE RXCIE0
#define TXCIE TXCIE0
#define RXEN RXEN0
#define TXEN TXEN0
#define UBRRL UBRR0L
#define UBRRH UBRR0H
//#define SIG_UART_TRANS SIG_USART_TRANS
//#define SIG_UART_RECV  SIG_USART_RECV
//#define SIG_UART_DATA  SIG_USART_DATA
#define UART_CALC_BAUDRATE(baudRate) ((uint32_t)((F_CPU) + ((uint32_t)baudRate * 4UL)) / ((uint32_t)(baudRate) * 8UL) - 1)

void UART_Init(uint32_t UART_BAUD_RATE)
{
	UBRRH = (UART_CALC_BAUDRATE(UART_BAUD_RATE)>>8) & 0xFF;
	UBRRL = (UART_CALC_BAUDRATE(UART_BAUD_RATE) & 0xFF);
	UCSR0A = ( 1<<UART_DOUBLE );
	UCRB = ((1<<TXEN) | (1<<RXEN));
	UCRC = UCRC_VALUE;
}

void UART_SendByte(uint8_t data)
{
	while (!(UCRA & (1<<UART_TXREADY)));
	UDR = data;
}

bool UART_ReadByte(uint8_t& data)
{
	if (UCRA & (1<<UART_RXREADY))
	{
		data = UDR;
		return true;
	} else return false;
}

void UART_SendArray(uint8_t *buffer, uint16_t bufferSize)
{
for(uint16_t i=0; i<bufferSize; i++)
  UART_SendByte(buffer[i]);
}
//************SPI********************************
#define SPI_MODE0 SPCR &= ~((1<<CPOL)|(1<<CPHA))
#define SPI_MODE1 SPCR = (1<<CPHA)
#define SPI_MODE2 SPCR = (1<<CPOL)
#define SPI_MODE3 SPCR = (1<<CPOL)|(1<<CPHA)
	//�������� ����� ������ �����
	// CPOL = 0		CPHA = 0	������� ����������� �������, ��������� ������ �������� �������	mode0
	// CPOL = 0		CPHA = 1	��������� ������ ����������� �������, ������� �������� �������	mode1
	// CPOL = 1		CPHA = 0	������� �������� �������, ��������� ������ ����������� �������	mode2
	// CPOL = 1		CPHA = 1	��������� ������ �������� �������, ������� ����������� �������	mode3
	
#define SPI_DIV2 SPSR |= (1<<SPI2X);
#define SPI_DIV4 SPCR &= ~((1<<SPR1)|(1<<SPR0))
#define SPI_DIV8 SPSR |= (1<<SPI2X); SPCR |=(1<<SPR0)
#define SPI_DIV16 SPCR |=(1<<SPR0)
#define SPI_DIV32 SPSR |= (1<<SPI2X); SPCR |= (1<<SPR1)
#define SPI_DIV64 SPCR |= (1<<SPR1)
//#define SPI_DIV64 SPSR |= (1<<SPI2X); SPCR |= (1<<SPR1)|(1<<SPR0)
#define SPI_DIV128 SPCR |= (1<<SPR1)|(1<<SPR0) 
	// SPI2X	SPR1	SPR0	������� SCK
	// 1		0		0		fosc/2
	// 0		0		0		fosc/4
	// 1		0		1		fosc/8
	// 0		0		1		fosc/16
	// 1		1		0		fosc/32
	// 0		1		0		fosc/64
	// 1		1		1		fosc/64
	// 0		1		1		fosc/128	
	
#define SPI_MSB SPCR &= ~(1<<DORD)
#define SPI_LSB SPCR |= (1<<DORD)
	// DORD |= (0<<DORD); //������� ������ ������. DORD=1 ������ ���������� ������� ���. DORD=0 ������ ���������� ������� ���.	
	// SPIE SPE DORD MSTR CPOL CPHA SPR1 SPR0
    // (1<<MSTR); //����� Master
    // (1<<SPE); // �������� SPI
	// (1<<SPIE); //�������� ����������
	// (1<<SPI2X); //������� ������� ������������
	// (0<<CPOL); //���������� ��������� �������
	// (0<<CPHA); //���� ��������� �������	
	
void StartSPI(uint8_t SPI_Mode, uint8_t SPI_Div, uint8_t SPI_Change_Shift )
{
	D10_Out; D10_High;	//SS
	D11_Out; D11_Low;	//MOSI
	D12_In;				//MISO
	D13_Out; D13_Low;	//CLK	

	SPCR = 0; //����0, MSB, 
	SPSR &= ~(1<<SPI2X); //��������� �������� ��������
	
	switch (SPI_Mode) 
	{
    case 1: SPI_MODE1; break;
    case 2: SPI_MODE2; break;
    case 3: SPI_MODE3; break;	  
	}
	
	switch (SPI_Div) 
	{
    case 2: SPI_DIV2; break;	
   // case 4: SPI_DIV4; break;
    case 8: SPI_DIV8; break;
    case 16: SPI_DIV16; break;	
    case 32: SPI_DIV32; break;
    case 64: SPI_DIV64; break;
    case 128: SPI_DIV128; break;	  
	}	
	
	if(SPI_Change_Shift==0)SPI_LSB; //���������� ����� LSB ��� MSB
	
	SPCR |= (1<<SPE)|(1<<MSTR); //�������� SPI � ���. ������
	//SPSR |= (1<<SPI2X);
	//SPCR = (1<<SPIE)|(1<<SPE)|(1<<DORD)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA)|(1<<SPR1)|(1<<SPR0); 
}

void StopSPI(void) 
{
	SPCR &= ~(1<<SPE);
}

uint8_t ReadSPI(void) 
{
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}

void SendSPI(uint8_t SPI_data) 
{
  SPDR = SPI_data;
  while(!(SPSR & (1<<SPIF)));
			
    // __asm__ volatile
			// (
			// ".spi_not_ready: out %[spdr], %[spi_data]"	"\n\t" //���������� ���������������� ������ � SPI
            // "in __tmp_reg__,%[spsr]"				"\n\t"	//������ ������� SPSR
            // "sbrs __tmp_reg__, %[spif]"				"\n\t"	//� ��������� ���� SPIF
            // "rjmp .spi_not_ready"					"\n\t"	//���� ������ �� ���������� ��������� ���������
            // ::
            // [spsr] "I" (_SFR_IO_ADDR(SPSR)),
            // [spif] "I" (SPIF),
            // [spdr] "I" (_SFR_IO_ADDR(SPDR)),
            // [spi_data] "r" (SPI_data)
            // );							
}

//**********AnalogRead***************************
uint16_t AnRead(uint8_t An_pin)
{
  ADMUX=An_pin;   
  delay_us(10);	  
  ADCSRA=B11000110;	//B11000111-125kHz B11000110-250kHz 
  while (ADCSRA & (1 << ADSC));
  An_pin = ADCL;
  uint16_t An = ADCH; 
  return (An<<8) + An_pin;
}
//**********Converter****************************
uint8_t CharToDec(uint8_t digit)
{
	return digit-48;
}

uint8_t DecToChar(uint8_t number)
{
	return number+48;
}

//******************EEPROM*******************************
void WriteEEPROM_Byte(uint8_t addr, uint8_t data)  //��������� � EEPROM
{
		eeprom_write_byte((uint8_t*)addr, data);
}

void WriteEEPROM_Word(uint16_t addr, uint16_t data)
{
		eeprom_write_word((uint16_t*)addr, data);
}

void WriteEEPROM_Long(uint8_t addr, uint32_t data)  //��������� � EEPROM
{           
  addr *= 4;
        eeprom_write_byte((uint8_t*)addr, data & 0xFF);
        eeprom_write_byte((uint8_t*)addr+1, (data & 0xFF00) >> 8);
        eeprom_write_byte((uint8_t*)addr+2, (data & 0xFF0000) >> 16);
        eeprom_write_byte((uint8_t*)addr+3, (data & 0xFF000000) >> 24);
		
	  // addr *= 2;
        // eeprom_write_word((uint16_t*)addr, data & 0xFFFF);
        // eeprom_write_word((uint16_t*)addr+1, (data & 0xFFFF0000) >> 16);
}

uint8_t ReadEEPROM_Byte(uint8_t addr)
{
		return eeprom_read_byte((uint8_t*)addr);
}

uint16_t ReadEEPROM_Word(uint16_t addr)
{
		return eeprom_read_word((uint16_t*)addr);
}

uint32_t ReadEEPROM_Long(uint8_t addr)  // ��������� �������� �� EEPROM
{
  addr *= 4; 
        uint32_t ir_code = eeprom_read_byte((uint8_t*)addr+3); 
        ir_code = (ir_code << 8) | eeprom_read_byte((uint8_t*)addr+2);
        ir_code = (ir_code << 8) | eeprom_read_byte((uint8_t*)addr+1);
        ir_code = (ir_code << 8) | eeprom_read_byte((uint8_t*)addr);
		//eeprom_read_word((uint16_t*) addr)
  return ir_code;
}
//**************����� ���� �������������� �������� � �������****************************
uint16_t find_similar(uint16_t *buf, uint8_t size_buff, uint8_t range) 
{
 uint8_t maxcomp=0; //������� ������������� ����������� ����������
 uint16_t mcn=0;	//����������� ����� ������������� ������� �������
 uint16_t comp;	//��������� ����������
 range++;	//���������� ����������

	for (uint8_t i=0; i<size_buff; i++) 
	{
		comp=buf[i];	//������ ������� ������� � comp
		uint8_t n=0;	//������� ����������
		for (uint8_t j=0; j<size_buff; j++)	{ if (buf[j]>comp-range && buf[j]<comp+range) n++;} // ���� ���������� �������� comp � ������� buf	
		if (n > maxcomp) //���� ����� �������� ������ ��� ���� ������� �����
		{
			maxcomp=n; //��������� ������� ��������
			mcn=comp; //��������� ����������� �������
		}		
	}
 return mcn;
}
//***************Beep �� 50 �� 2000��****************
void beep(uint16_t dur, uint16_t frq)
{
  //dur=(1000/frq)*dur;  //������ ������������ ����
  uint16_t per=500000/frq; //����. ������� � ���
  dur=dur/(per/250)*2;
  for(uint16_t i=0; i<dur; i++)
  {
    D11_High; 
    delay_us(per); 
    D11_Low;
    delay_us(per);
  } 
} 
//**************���������� �����*******************
void reset() 
{
	  wdt_disable();  //���������, ��� ��� �� ����� ���� ��� �������
	  wdt_enable(WDTO_15MS); //��������� ���������� ��������� ����� 15��
	  while (1) {}	//���������
}

//**************Delay****************************
void delay_ms(uint16_t tic_ms)
{
		while(tic_ms)
		{
			delay_us(999);
			tic_ms--;
		}
}

void delay_us(uint16_t tic_us)
{
	tic_us *= 4; //1us = 4 �����
	__asm__ volatile 
		  (	
			"1: sbiw %0,1" "\n\t" //; ������� �� �������� �������� N
			"brne 1b"				
			: "=w" (tic_us)
			: "0" (tic_us)
		  );
}
// ******************************************MEGA**********************************************
//*********************************************************************************************
//*********************************************************************************************
#elif  defined (__AVR_ATmega2560__) || defined (__AVR_ATmega2561__)
//******************EEPROM*******************************
void WriteEEPROM_Byte(uint8_t addr, uint8_t data)  //��������� � EEPROM
{
		eeprom_write_byte((uint8_t*)addr, data);
}

void WriteEEPROM_Word(uint16_t addr, uint16_t data)
{
		eeprom_write_word((uint16_t*)addr, data);
}

void WriteEEPROM_Long(uint8_t addr, uint32_t data)  //��������� � EEPROM
{           
  addr *= 4;
        eeprom_write_byte((uint8_t*)addr, data & 0xFF);
        eeprom_write_byte((uint8_t*)addr+1, (data & 0xFF00) >> 8);
        eeprom_write_byte((uint8_t*)addr+2, (data & 0xFF0000) >> 16);
        eeprom_write_byte((uint8_t*)addr+3, (data & 0xFF000000) >> 24);
		
	  // addr *= 2;
        // eeprom_write_word((uint16_t*)addr, data & 0xFFFF);
        // eeprom_write_word((uint16_t*)addr+1, (data & 0xFFFF0000) >> 16);
}

uint8_t ReadEEPROM_Byte(uint8_t addr)
{
		return eeprom_read_byte((uint8_t*)addr);
}

uint16_t ReadEEPROM_Word(uint16_t addr)
{
		return eeprom_read_word((uint16_t*)addr);
}

uint32_t ReadEEPROM_Long(uint8_t addr)  // ��������� �������� �� EEPROM
{
  addr *= 4; 
        uint32_t ir_code = eeprom_read_byte((uint8_t*)addr+3); 
        ir_code = (ir_code << 8) | eeprom_read_byte((uint8_t*)addr+2);
        ir_code = (ir_code << 8) | eeprom_read_byte((uint8_t*)addr+1);
        ir_code = (ir_code << 8) | eeprom_read_byte((uint8_t*)addr);
		//eeprom_read_word((uint16_t*) addr)
  return ir_code;
}
//**************Delay****************************
void delay_ms(uint16_t tic_ms)
{
		while(tic_ms)
		{
			delay_us(999);
			tic_ms--;
		}
}

void delay_us(uint16_t tic_us)
{
	tic_us *= 4; //1us = 4 �����
	__asm__ volatile 
		  (	
			"1: sbiw %0,1" "\n\t" //; ������� �� �������� �������� N
			"brne 1b"				
			: "=w" (tic_us)
			: "0" (tic_us)
		  );
}
//*************************************************Leonardo � ��� �� 32U4**********************************
//********************************************************************************************************
//********************************************************************************************************
#elif defined (__AVR_ATmega32U4__)
void delay_ms(uint16_t tic_ms)
{
		while(tic_ms)
		{
			delay_us(999);
			tic_ms--;
		}
}

void delay_us(uint16_t tic_us)
{
	tic_us *= 4; //1us = 4 �����
	__asm__ volatile 
		  (	
			"1: sbiw %0,1" "\n\t" //; ������� �� �������� �������� N
			"brne 1b"				
			: "=w" (tic_us)
			: "0" (tic_us)
		  );
}

#endif

