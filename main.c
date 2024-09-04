#include <avr/io.h>
#define F_CPU 14745600UL //14.7456MHz
#include <util/delay.h>
#include <avr/interrupt.h>
#include "avr/iom128.h"

volatile uint8_t receive = 0;
volatile uint8_t rx_data = 0;
volatile uint8_t first_b;
volatile uint8_t second_b;
volatile uint8_t three_b;
volatile uint8_t rx_flag = 0;



static void UARTInit(void) {//настройка COM порта
	UBRR0H = 0;
	UBRR0L = 95; //baud rate 9600   
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00); //8 bit, 1 stop bit
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);

}

void SPI_send_byte(char data)
{ 	SPDR=data; //отправить байт
	while (!(SPSR&(1<<SPIF)));//дождаться окончания передачи	
}

char SPI_get_byte()
{ 	uint8_t report;
     SPDR=0xFF; //отправить байт

	while (!(SPSR&(1<<SPIF)));//дождаться окончания передачи
	report =SPDR; //принять байт
	return report;
		
}



void UARTSend(uint8_t data) {//отправить байт в комп
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}
unsigned char UARTGet() {//принять байт от компа
	while(!rx_flag);
	rx_flag = 0;
	return rx_data;
}

/* Функция инициализация АЦП */
static void ADC_Init(void){
	ADCSRA |= (1 << ADEN) // Включаем АЦП
	|(1 << ADPS2)|(1 << ADPS1)|(1 << ADPS0); // устанавливаем предделитель преобразователя на 128
	ADMUX |= (0 << REFS1)|(1 << REFS0)|(0 << ADLAR) //выставляем опорное напряжение, как внешний ИОН
	|(0 << MUX0)|(0 << MUX1)|(0 << MUX2)|(0 << MUX3)|(0 << MUX4); // ацп подключен в GND
}

int AD_Converter ()
	{ ADCSRA |= (1 << ADSC);// Начинаем преобразование 
	second_b= ADCL;
	three_b=ADCH;
		
	}
	
static void SPI_unit (void)//настройка SPI
{ DDRB |= (1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB4);
	//настроить выводы MOSI, SCK, ПИМ, ПРД  на выход
	PORTB |=(1<<PB0)|(1<<PB4); //установить ПИМ, ПРД  в 1

	SPCR =0 ; //обнулить
	SPSR =0;

	SPCR |= (1<<MSTR)|(1<<SPR0)|(1<<SPR1)|(1<<CPOL)|(1<<DORD); //режим мастер, F=Fosc/128
	SPSR |= (0<<SPI2X); //F=Fosc/128

	SPCR|= (1<<SPE); //включить SPI
}


int main(void) {//основная программа
	ADC_Init();
	DDRC=0xFF; PORTC=0x80; //установка порта PORTC;
	DDRD=0xFF; PORTD=0xFF; //установка порта PORTD;
	UARTInit();// настройка COm порта
	DDRE=0x78; 
	PORTE |=(1<<PE3)|(1<<PE4)|(1<<PE5)|(1<<PE6); //установка порта PORTE;
	SPI_unit ();
	AD_Converter ();
	while(1) {
		sei();//разрешаем прерывания
		first_b= UARTGet();
		second_b= UARTGet();
		//PORTC=0xFF;
		cli(); //запрещаем прерывания
		switch (first_b) {
			case 0b10000000 : //Идентификация приспособления
			UARTSend(0b11000000);//second_b=0; 
			UARTSend(second_b);
			break;
		
			case 0b00000001 : //передать инф в парараллейный канал 1
			DDRA=0xFF;//порт на выход;
			PORTA=second_b;
			PORTC=0b10000001;
			PORTC=0b10000000;
			UARTSend(0b10000001);
			UARTSend(second_b);
			break;
			
			case 0b00000010 : //передать инф в парараллейный канал 2
			DDRA=0xFF;//порт на выход;
			PORTA=second_b;
			PORTC=0b10000010;
			PORTC=0b10000000;
			UARTSend(0b10000010);
			UARTSend(second_b);
			break;
		
			case 0b00000011 : //передать инф в парараллейный канал 3
			DDRA=0xFF;//порт на выход;
			PORTA=second_b;
			PORTC=0b10000100;
			PORTC=0b10000000;
			first_b=0b10000011;
			UARTSend(first_b);
			UARTSend(second_b);
			break;
			
			case 0b00000100 : //передать инф в парараллейный канал 4
			DDRA=0xFF;//порт на выход;
			PORTA=second_b;
			PORTC=0b10001000;
			PORTC=0b10000000;
			UARTSend(0b10000100);
			UARTSend(second_b);
			break;
		
			case 0b00000101 : //передать инф в парараллейный канал 5
			DDRA=0xFF;//порт на выход;
			PORTA=second_b;
			PORTC=0b10010000;
			PORTC=0b10000000;
			UARTSend(0b10000101);
			UARTSend(second_b);
			break;
			
			case 0b00000110 : //передать инф в парараллейный канал 6
			DDRA=0xFF;//порт на выход;
			PORTA=second_b;
			PORTC=0b10100000;
		//	PORTC=0b10000000;
			UARTSend(0b10000110);
			UARTSend(second_b);
			break;
			
			case 0b00000111 : //считать инф из парараллейного канала 7
			DDRA=0x00;//порт на вход;
			PORTC=0b11000000;
			PORTC=0b00000000;
			second_b=PINA;
			PORTC=0b10000000;
			UARTSend(0b10000111);
			UARTSend(second_b);
			break;
			
			case 0b00001000 : //считать инф из парараллейного канала 8
			DDRA=0x00;//порт на вход;
			PORTC=0b11000000;
			PORTD=0b01000000;
			PORTC=0b10000000;
			second_b=PINA;
			PORTD=0b11000000;
			UARTSend(0b10001000);
			UARTSend(second_b);
			break;
		
			case 0b00001001 : //считать инф из парараллейного канала 9
			DDRA=0x00;//порт на вход;
			PORTC=0b11000000;
			PORTD=0b10000000;
			PORTC=0b10000000;
			second_b=PINA;
			PORTD=0b11000000;
			UARTSend(0b10001001);
			UARTSend(second_b);
			break;
			
			case 0b00001010 : //оцифровать канал 10, разряд 0
			PORTE |=(1<<PE3)|(1<<PE4)|(1<<PE5)|(1<<PE6);
				if((1 << PE7) & PINE) first_b=0b10001010;
				else first_b=0b11001010;
			AD_Converter ();
			UARTSend(first_b);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
			
			case 0b00011010 : //оцифровать канал 10, разряд 1
			PORTE=0b00010000;
			if((1 << PE7) & PINE) first_b=0b10011010;
			else first_b=0b11011010;
			AD_Converter ();
			UARTSend(first_b);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
		
			case  0b00101010 : //оцифровать канал 10, разряд 2
			PORTE=0b00001000;
			if((1 << PE7) & PINE) first_b=0b10101010;
			else first_b=0b11101010;
			AD_Converter ();
			UARTSend(first_b);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
		
			case  0b00111010 : //оцифровать канал 10, разряд 3
			PORTE=0b00000100;
			if((1 << PE7) & PINE) first_b=0b10111010;
			else first_b=0b11111010;
			AD_Converter ();
			UARTSend(first_b);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
			
			case  0b01001010 : //оцифровать канал 10, разряд 4
			PORTE=0b00000100;
			if((1 << PE7) & PINE) first_b=0b01001110;
			else first_b=0b11001110;
			AD_Converter ();
			UARTSend(first_b);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
		
			case  0b01011010 : //оцифровать канал 10, разряд 5
			PORTE=0b00000100;
			if((1 << PE7) & PINE) first_b=0b01011110;
			else first_b=0b11011110;
			AD_Converter ();
			UARTSend(first_b);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
		
			case  0b01101010 : //оцифровать канал 10, разряд 6
			PORTE=0b00000100;
			if((1 << PE7) & PINE) first_b=0b01101110;
			else first_b=0b11101110;
			AD_Converter ();
			UARTSend(first_b);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
		
			case  0b01111010 : //оцифровать канал 10, разряд 7
			PORTE=0b00000100;
			if((1 << PE7) & PINE) first_b=0b01111110;
			else first_b=0b11111110;
			AD_Converter ();
			UARTSend(first_b);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
		
			case  0b00001011 : //оцифровать канал 11, разряд 0
			PORTE=0b00000100;
			if((1 << PE7) & PINE) first_b=0b10001011;
			else first_b=0b11001011;
			AD_Converter ();
			UARTSend(first_b);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
			
			case  0b00011011 : //оцифровать канал 11, разряд 1
			PORTE=0b00000100;
			if((1 << PE7) & PINE) first_b=0b10011011;
			else first_b=0b11011011;
			AD_Converter ();
			UARTSend(first_b);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
		
			case  0b00101011 : //оцифровать канал 11, разряд 2
			PORTE=0b00000100;
			if((1 << PE7) & PINE) first_b=0b10101011;
			else first_b=0b11101011;
			AD_Converter ();
			UARTSend(first_b);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
		
			case  0b00111011 : //оцифровать канал 11, разряд 3
			PORTE=0b00000100;
			if((1 << PE7) & PINE) first_b=0b10111011;
			else first_b=0b11111011;
			AD_Converter ();
			UARTSend(first_b);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
		
			case  0b01001011 : //оцифровать канал 11, разряд 4
			PORTE=0b00000100;
			if((1 << PE7) & PINE) first_b=0b11101111;
			else first_b=0b11100111;
			AD_Converter ();
			UARTSend(first_b);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
		
			case  0b01011011 : //оцифровать канал 11, разряд 5
			PORTE=0b00000100;
			if((1 << PE7) & PINE) first_b=0b11111100;
			else first_b=0b11111000;
			AD_Converter ();
			UARTSend(first_b);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
		
			case  0b01101011 : //оцифровать канал 11, разряд 6
			PORTE=0b00000100;
			if((1 << PE7) & PINE) first_b=0b11111001;
			else first_b=0b11110001;
			AD_Converter ();
			UARTSend(first_b);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
		
			case  0b01111011 : //оцифровать канал 11, разряд 7
			PORTE=0b00000100;
			if((1 << PE7) & PINE) first_b=0b11110011;
			else first_b=0b11100011;
			AD_Converter ();
			UARTSend(first_b);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
		
			case  0b00011100 : //передать в посл канал
			UARTSend(0b10011100);//готов к передаче
			UARTSend(second_b);
			sei();//разрешаем прерывания
			first_b=  UARTGet();
			second_b= UARTGet();
			three_b=  UARTGet();
			uint8_t four_b=UARTGet();
			cli(); //запрещаем прерывания
			PORTB &=~(1<<PB0);//установить ПРД в 0
			PORTB |=(1<<PB4); //установить ПИМ в 1
			SPI_send_byte(first_b);//отправка адреса блока
			PORTB &=~(1<<PB4);//установить ПИМ в 0
			SPI_send_byte(second_b);
			SPI_send_byte(three_b);
			SPI_send_byte(four_b);
			PORTB |=(1<<PB4); //установить ПИМ в 1
			PORTB |=(1<<PB0);//установить ПРД в 1
			UARTSend(0b11011100);
			UARTSend(first_b);
			break;
			
			case  0b00111000 : //из посл канала в комп
			PORTB &=~(1<<PB0);//установить ПРД в 0
			SPI_send_byte(second_b);
			PORTB |=(1<<PB0);//установить ПРД в 1
			PORTB &=~(1<<PB4);//установить ПИМ в 0

			second_b=SPI_get_byte();
			three_b=  SPI_get_byte();
			uint8_t four1_b=SPI_get_byte();
			uint8_t five_b=SPI_get_byte();
			uint8_t six_b=SPI_get_byte();
			uint8_t seven_b=SPI_get_byte();
			uint8_t eant_b=SPI_get_byte();

			PORTB |=(1<<PB4); //установить ПИМ в 1
			UARTSend(0b10111000);
			UARTSend(second_b);
			UARTSend(three_b);
			UARTSend(four1_b);
			UARTSend(five_b);
			UARTSend(six_b);
			UARTSend(seven_b);
			UARTSend(eant_b);
			break;

			case  0b01001101 : //из посл канала в комп, один байт
			PORTB &=~(1<<PB0);//установить ПРД в 0
			SPI_send_byte(second_b);
			PORTB |=(1<<PB0);//установить ПРД в 1
			PORTB &=~(1<<PB4);//установить ПИМ в 0

			second_b=SPI_get_byte();
			three_b=  SPI_get_byte();
			
			PORTB |=(1<<PB4); //установить ПИМ в 1
			UARTSend(0b11001101);
			UARTSend(second_b);
			UARTSend(three_b);
			break;
		}
	
	}
}

ISR(USART0_RX_vect) {
	rx_data = UDR0;
	rx_flag = 1;
}
