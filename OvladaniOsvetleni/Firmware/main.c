/*
 * F-00013.c
 * Firmware pro ovl�d�n� p�es RS485 - > Ofuk
  * Created: 4.2.2016 15:28:42
 * Author : Lukas
 */ 

//Rozlo�en� pin�
/*
	PB0 - Osvetlen� PWM 02
	PB1 - Osvetlen� PWM 03
	PB2 - Osvetlen� PWM 04
	PB3 - MCU_LED 1
	PB4 - MCU_LED 2
	PB5 - MCU_SHUTDOWN (Battery)
	
	PC0 - ADC Battery
	PC1 - PWM Servo
	PC2 - Konc�k 01 Servo
	PC3 - Konc�k 02 Servo
	PC4 - On/Off Osv�tlen� 01 
	PC5 - On/Off Osv�tlen� 02
	
	PD0 - RS485_RX
	PD1 - RS485_TX
	PD2 - RS485_Enable
	PD3 - On/Off Osv�tlen� 03
	PD4 - On/Off Osv�tlen� 04
	PD5 - G2 Z�v�rka
	PD6 - G1 Z�v�rka
	PD7 - Osvetlen� PWM 01
	*/

//#include <avr/iom32.h>
#include <avr/io.h>
//#include <avr/iom128a.h>
#include "inc/AllInit.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include "inc/common_defs.h"
#include "inc/defines.h"
#include "inc/timer.h"
#include "inc/uart_types.h"
#include "inc/uart_tri_0.h"
#include "inc/Tribus_types.h"
#include "inc/Tribus.h"
#include <stdlib.h>
// Standard Input/Output functions
#include <stdio.h>
#include <string.h>
#include "inc/Lib-all.h"
#include <avr/eeprom.h>
#include "inc/adc.h"

// Nastaven� Servo pinu pro ovl�d�n� PWM
#define SERVO_PORT	PORTC
#define SERVO_PIN	PORTC1

#define SERVO_On	sbi(SERVO_PORT, SERVO_PIN);
#define SERVO_Off	cbi(SERVO_PORT, SERVO_PIN);
#define SERVO_Toggle	nbi(SERVO_PORT, SERVO_PIN);

	
//Nastaven� pro ovl�d�n� PWM sektoru �.1
#define OSV00_PORT_PWM	PORTD
#define OSV00_PIN_PWM	PORTD7
// #define OSV00_PORT_PWM	PORTB
// #define OSV00_PIN_PWM	PORTB4

//Nastaven� pro ovl�d�n� PWM sektoru �.2
#define OSV01_PORT_PWM	PORTB
#define OSV01_PIN_PWM	PORTB0

//Nastaven� pro ovl�d�n� PWM sektoru �.3
#define OSV02_PORT_PWM	PORTB
#define OSV02_PIN_PWM	PORTB1

//Nastaven� pro ovl�d�n� PWM sektoru �.4
#define OSV03_PORT_PWM	PORTB
#define OSV03_PIN_PWM	PORTB2

//Sp�n�n� a vyp�n�n� cel�ho sektoru �.1
#define OSV00_PORT_SWITCH	PORTC
#define OSV00_PIN_SWITCH	PORTC4
#define OSV00_On	sbi(OSV00_PORT_SWITCH, OSV00_PIN_SWITCH)
#define OSV00_Off	cbi(OSV00_PORT_SWITCH, OSV00_PIN_SWITCH)

//Sp�n�n� a vyp�n�n� cel�ho sektoru �.2
#define OSV01_PORT_SWITCH	PORTC
#define OSV01_PIN_SWITCH	PORTC5
#define OSV01_On	sbi(OSV01_PORT_SWITCH, OSV01_PIN_SWITCH)
#define OSV01_Off	cbi(OSV01_PORT_SWITCH, OSV01_PIN_SWITCH)

//Sp�n�n� a vyp�n�n� cel�ho sektoru �.3
#define OSV02_PORT_SWITCH	PORTD
#define OSV02_PIN_SWITCH	PORTD3
#define OSV02_On	sbi(OSV02_PORT_SWITCH, OSV02_PIN_SWITCH)
#define OSV02_Off	cbi(OSV02_PORT_SWITCH, OSV02_PIN_SWITCH)

//Sp�n�n� a vyp�n�n� cel�ho sektoru �.4
#define OSV03_PORT_SWITCH	PORTD
#define OSV03_PIN_SWITCH	PORTD4
#define OSV03_On	sbi(OSV03_PORT_SWITCH, OSV03_PIN_SWITCH)
#define OSV03_Off	cbi(OSV03_PORT_SWITCH, OSV03_PIN_SWITCH)

#define Eeprom_Address	0x16

#define OSV_COUNT	4

#define		OSV00	0
#define		OSV01	1
#define		OSV02	2
#define		OSV03	3

#define ADC_Battery	10


uint8_t baudrate =0;

uint8_t Led=0;
//Stavy
volatile uint16_t SET_Periode_Servo=0, SET_Duty_Servo=0, SET_Periode_OSV=0;

volatile uint16_t OSV_P=0, OSV01_P=0, OSV00_S=0, OSV01_S=0, OSV02_S=0, OSV03_S=0;

volatile uint16_t SET_Servo_Left=0, SET_Servo_Center=0, SET_Servo_Right=0;


// Init pro Tribus
volatile byte timer0_flag = 0, Servo_Flag=0; // T = 10ms

//Osv�tlen� nastaven� spodn� hranice
uint16_t OSV_LOW[OSV_COUNT]={0,0,0,0};
uint16_t OSV_HIGH[OSV_COUNT]={0,0,0,0};

// Nastaven� st��dy jednotliv�ch osv�tlen�
volatile uint8_t OSV_Duty[OSV_COUNT]={0,0,0,0};
// Nastaven� On/Off osv�tlova�� After
volatile uint8_t OSV_Flag_A[OSV_COUNT]={0,0,0,0};
// Nastaven� On/Off osv�tlova�� Before
volatile uint8_t OSV_Flag_B[OSV_COUNT]={0,0,0,0};
	
	//OSV_Flag_A[0] -> Osv�tlova� 0 (After)
	//OSV_Flag_A[1] -> Osv�tlova� 1 (After)
	//OSV_Flag_A[2] -> Osv�tlova� 2 (After)
	//OSV_Flag_A[3] -> Osv�tlova� 3 (After)


byte led_timer = 0;

uint16_t Perioda=0, Strida=0;

void send_data(void)
{
	uart0_put_data((byte *) &TB_bufOut);
}

//----------------------------------------------------------
ISR(TIMER0_COMPA_vect)
{
	// T = 10ms
	timer0_flag = TRUE;
}

// ISR(TIMER1_COMPA_vect) {
// 	// T = 0,01ms
// 	Servo_Flag = TRUE;
// 	SERVO_Toggle;
// 	
// }

unsigned volatile long int 

servopos[8]={22118,22118,22118,22118,22118,22118,22118,22118},		//1.5ms output by default

spmin[8] ={10000,10000,14745,14745,14745,14745,14745,14745},		//1ms minimum by default

spmax[8] = {33000,33000,29490,29490,29490,29490,29490,29490};		//2ms maximum by default

uint16_t Zk=3100;

ISR(TIMER1_COMPA_vect)
{
	SERVO_Off;		//Turn on Servo Channel (s)
	OCR1A = SET_Duty_Servo;
}

ISR(TIMER1_OVF_vect)
{
	SERVO_On;		//Turn on Servo Channel (s)
	//OCR1A = servopos[0];      //Update PWM duty for next Servo Channel
}

ISR(TIMER2_COMPA_vect) {
	if (OSV_P > (SET_Periode_OSV - 1))
	{
		sbi(OSV00_PORT_PWM, OSV00_PIN_PWM);
		sbi(OSV01_PORT_PWM, OSV01_PIN_PWM);
		sbi(OSV02_PORT_PWM, OSV02_PIN_PWM);
		sbi(OSV03_PORT_PWM, OSV03_PIN_PWM);
		OSV_P = 0; 
	}
	else
	{
		if ((OSV_P > (OSV_Duty[OSV00] - 1)))
		{
			cbi(OSV00_PORT_PWM, OSV00_PIN_PWM);
		}
		if ((OSV_P > (OSV_Duty[OSV01] - 1)))
		{
			cbi(OSV01_PORT_PWM, OSV01_PIN_PWM);
		}
		if ((OSV_P > (OSV_Duty[OSV02] - 1)))
		{
			cbi(OSV02_PORT_PWM, OSV02_PIN_PWM);
		}
		if ((OSV_P > (OSV_Duty[OSV03] - 1)))
		{
			cbi(OSV03_PORT_PWM, OSV03_PIN_PWM);
		}
	}
	OSV_P++;
}

// Set up eeprom
void Eeprom_OSV_Servo(void)
{
	// 	SET_Periode_OSV = 100;
// 	SET_Duty_OSV00 = 80;
// 	SET_Duty_OSV01 = 1;
// 	SET_Duty_OSV02 = 40;
// 	SET_Duty_OSV03 = 20;

// 	OSV_Duty[OSV00] = 80;
// 	OSV_Duty[OSV01] = 1;
// 	OSV_Duty[OSV02] = 40;
// 	OSV_Duty[OSV03] = 20;


	eeprom_read_block((void*)OSV_Duty, (const  void*)Eeprom_Address, sizeof(uint8_t)*4);
	eeprom_read_block((void*)OSV_Flag_A, (const  void*)Eeprom_Address + (1 * OSV_COUNT), sizeof(uint8_t)*4);
	
	//SET_Periode_Servo = eeprom_read_byte(( uint8_t *) Eeprom_Address + 8);
	SET_Periode_Servo = eeprom_read_word(( uint16_t *) Eeprom_Address + 8);
	ICR1 = SET_Periode_Servo;
	
	OSV_LOW[OSV00] = eeprom_read_byte(( uint8_t *) (Eeprom_Address + 10));
	OSV_HIGH[OSV00] = eeprom_read_byte(( uint8_t *) (Eeprom_Address + 11));
	
	OSV_LOW[OSV01] = eeprom_read_byte(( uint8_t *) (Eeprom_Address + 12));
	OSV_HIGH[OSV01] = eeprom_read_byte(( uint8_t *) (Eeprom_Address + 13));
	
	OSV_LOW[OSV02] = eeprom_read_byte(( uint8_t *) (Eeprom_Address + 14));
	OSV_HIGH[OSV02] = eeprom_read_byte(( uint8_t *) (Eeprom_Address + 15));
	
	OSV_LOW[OSV03] = eeprom_read_byte(( uint8_t *) (Eeprom_Address + 16));
	OSV_HIGH[OSV03] = eeprom_read_byte(( uint8_t *) (Eeprom_Address + 17));
	
	SET_Servo_Left = eeprom_read_word(( uint16_t *) Eeprom_Address + 18);
	SET_Servo_Center = eeprom_read_word(( uint16_t *) Eeprom_Address + 20);
	SET_Servo_Right = eeprom_read_word(( uint16_t *) Eeprom_Address + 22);
	
	SET_Periode_OSV = eeprom_read_byte(( uint8_t *) Eeprom_Address + 24);
	
	SET_Duty_Servo = SET_Servo_Right;
	
	
	//SET_Periode_OSV = 100;

	//SET_Periode_Servo=400; // 0,05ms *400 = 20ms
	//SET_Duty_Servo=20; // 0,05ms * 20 = 1ms
	
	OSV_Flag_B[OSV00] = OSV_Flag_A[OSV00];
	OSV_Flag_B[OSV01] = OSV_Flag_A[OSV01];
	OSV_Flag_B[OSV02] = OSV_Flag_A[OSV02];
	OSV_Flag_B[OSV03] = OSV_Flag_A[OSV03];
	
}

uint16_t Citac=0;
//----------------------------------------------------------
void process_timer_100Hz(void)
{
	if (timer0_flag)
	{	
		// T = 10ms
		timer0_flag = FALSE;
		uart0_ISR_timer();
		if (Citac > 50)
		{
			nbi(PORTB, PORTB3);
			Citac=0;

		}
		Citac++;
	}
	
}


void try_receive_data(void)
{
	byte i;
	byte *ptr;
	if (uart0_flags.data_received)
	{
		ptr = uart0_get_data_begin();
		for (i=0; i<9; i++)
		{
			TB_bufIn[i] = *ptr;
			ptr++;
		}
		uart0_get_data_end();
		uart0_flags.data_received = FALSE;
		if (TB_Read() == 0)
		{
			switch (TB_Decode())
			{
				//Nastaven� osv�tlen�
				case TB_CMD_SIO:
					switch(TB_bufIn[TB_BUF_MOTOR])
					{
						case 1:	//Analogov� nastaven� osv�tlen�
							switch(TB_bufIn[TB_BUF_TYPE])
							{
								// Nastaven� nen� mezi 0 a 100% 
								if ((TB_Value < 0) || (TB_Value > 100))
								{
									TB_SendAck(TB_ERR_NOK, TB_Value);
								}
								else
								{
									case OSV00:	//Osv�tlova� �.0
										OSV_Duty[OSV00] = (((OSV_HIGH[OSV00] - OSV_LOW[OSV00]) * TB_Value) / 100) + OSV_LOW[OSV00];
										eeprom_update_byte(( uint8_t *) Eeprom_Address, OSV_Duty[OSV00]);
										TB_SendAck(TB_ERR_OK, TB_Value);
										break;
							
									case OSV01:	//Osv�tlova� �.1
										OSV_Duty[OSV01] = (((OSV_HIGH[OSV01] - OSV_LOW[OSV01]) * TB_Value) / 100) + OSV_LOW[OSV01];
										eeprom_update_byte(( uint8_t *) Eeprom_Address + 1, OSV_Duty[OSV01]);
										TB_SendAck(TB_ERR_OK, TB_Value);
										break;
							
									case OSV02:	//Osv�tlova� �.2
										OSV_Duty[OSV02] = (((OSV_HIGH[OSV02] - OSV_LOW[OSV02]) * TB_Value) / 100) + OSV_LOW[OSV02];
										eeprom_update_byte(( uint8_t *) Eeprom_Address + 2, OSV_Duty[OSV02]);
										TB_SendAck(TB_ERR_OK, TB_Value);
										break;
							
									case OSV03:	//Osv�tlova� �.3
										OSV_Duty[OSV03] = (((OSV_HIGH[OSV03] - OSV_LOW[OSV03]) * TB_Value) / 100) + OSV_LOW[OSV03];
										eeprom_update_byte(( uint8_t *) Eeprom_Address + 3, OSV_Duty[OSV03]);
										TB_SendAck(TB_ERR_OK, TB_Value);
										break;

									default:	//Ostatn�
										TB_SendAck(TB_ERR_TYPE, TB_Value);
										break;
																	
								}
							}
							break;
						
						case 2:	//Digit�ln� nastaven� osv�tlen� ON/OFF
							switch(TB_bufIn[TB_BUF_TYPE])
							{
								case OSV00:	//Osv�tlova� �.0
									OSV_Flag_B[OSV00] = OSV_Flag_A[OSV00];
									switch(TB_Value)
									{
										case 0:
											OSV00_Off;
											OSV_Flag_A[OSV00] = FALSE;
											eeprom_update_byte(( uint8_t *) Eeprom_Address + 4, OSV_Flag_A[OSV00]);
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										case 1:
											OSV00_On;
											OSV_Flag_A[OSV00] = TRUE;
											eeprom_update_byte(( uint8_t *) Eeprom_Address + 4, OSV_Flag_A[OSV00]);
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										default:
											TB_SendAck(TB_ERR_VALUE, TB_Value);
											break;
									}
									break;
							
								case OSV01:	//Osv�tlova� �.1
									OSV_Flag_B[OSV01] = OSV_Flag_A[OSV01];
									switch(TB_Value)
									{
										case 0:
											OSV01_Off;
											OSV_Flag_A[OSV01] = FALSE;
											eeprom_update_byte(( uint8_t *) Eeprom_Address + 5, OSV_Flag_A[OSV01]);
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										case 1:
											OSV01_On;
											OSV_Flag_A[OSV01] = TRUE;
											eeprom_update_byte(( uint8_t *) Eeprom_Address + 5, OSV_Flag_A[OSV01]);
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										default:
											TB_SendAck(TB_ERR_VALUE, TB_Value);
											break;
									}
									break;
							
								case OSV02:	//Osv�tlova� �.2
									OSV_Flag_B[OSV02] = OSV_Flag_A[OSV02];
									switch(TB_Value)
									{
										case 0:
											OSV02_Off;
											OSV_Flag_A[OSV02] = FALSE;
											eeprom_update_byte(( uint8_t *) Eeprom_Address + 6, OSV_Flag_A[OSV02]);
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										case 1:
											OSV02_On;
											OSV_Flag_A[OSV02] = TRUE;
											eeprom_update_byte(( uint8_t *) Eeprom_Address + 6, OSV_Flag_A[OSV02]);
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										default:
											TB_SendAck(TB_ERR_VALUE, TB_Value);
											break;
									}
									break;
							
								case OSV03:	//Osv�tlova� �.3
									OSV_Flag_B[OSV03] = OSV_Flag_A[OSV03];
									switch(TB_Value)
									{
										case 0:
											OSV03_Off;
											OSV_Flag_A[OSV03] = FALSE;
											eeprom_update_byte(( uint8_t *) Eeprom_Address + 7, OSV_Flag_A[OSV03]);
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										case 1:
											OSV03_On;
											OSV_Flag_A[OSV03] = TRUE;
											eeprom_update_byte(( uint8_t *) Eeprom_Address + 7, OSV_Flag_A[OSV03]);
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										default:
											TB_SendAck(TB_ERR_VALUE, TB_Value);
											break;
									}
									break;
							
								default:	//Ostatn�
									TB_SendAck(TB_ERR_TYPE, TB_Value);
									break;
							}
							break;
						// Nastaven� Serva
						case 3:
							switch(TB_bufIn[TB_BUF_TYPE])
							{
								case 0:
									switch(TB_Value)
									{
										case 0:
											SET_Duty_Servo = SET_Servo_Left;
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										case 1:
											SET_Duty_Servo = SET_Servo_Center;
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										case 2:
											SET_Duty_Servo = SET_Servo_Right;
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										default:
											TB_SendAck(TB_ERR_VALUE, TB_ERR_VALUE);
											break;
									}
									break;
								default:
									TB_SendAck(TB_ERR_NOK, TB_Value);
									break;
							}
							break;
						//Speci�ln� zapnut� vypnut�.
						case 4:
							switch(TB_bufIn[TB_BUF_TYPE])
							{
								case 0:
									switch (TB_Value)
									{
										case 0:
											OSV00_Off;
											OSV01_Off;
											OSV02_Off;
											OSV03_Off;
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										case 1:
											OSV00_On;
											OSV01_On;
											OSV02_On;
											OSV03_On;
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										case 2:
 											if(OSV_Flag_B[OSV00])
												OSV00_On;
											else
												OSV00_Off;
											if(OSV_Flag_B[OSV01])
												OSV01_On;
											else
												OSV01_Off;
											if(OSV_Flag_B[OSV02])
												OSV02_On;
											else
												OSV02_Off;
											if(OSV_Flag_B[OSV03])
												OSV03_On;
											else
												OSV03_Off;
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										default:
											TB_SendAck(TB_ERR_VALUE, TB_Value);
											break;
									}
									break;
								case 1:
									OSV_Flag_B[OSV00] = OSV_Flag_A[OSV00];
									OSV_Flag_B[OSV01] = OSV_Flag_A[OSV01];
									OSV_Flag_B[OSV02] = OSV_Flag_A[OSV02];
									OSV_Flag_B[OSV03] = OSV_Flag_A[OSV03];
									switch (TB_Value)
									{
										case OSV00:
											OSV00_On;
											OSV01_Off;
											OSV02_Off;
											OSV03_Off;
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										case OSV01:
											OSV00_Off;
											OSV01_On;
											OSV02_Off;
											OSV03_Off;
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										case OSV02:
											OSV00_Off;
											OSV01_Off;
											OSV02_On;
											OSV03_Off;
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										case OSV03:
											OSV00_Off;
											OSV01_Off;
											OSV02_Off;
											OSV03_On;
											TB_SendAck(TB_ERR_OK, TB_Value);
											break;
										default:
											TB_SendAck(TB_ERR_NOK, TB_Value);
											break;
									}
									break;	
								default: // Ostatn� nastaven� pro Type u Motoru 4
									TB_SendAck(TB_ERR_NOK, TB_Value);
									break;
							}
							break;
						
						default:	//Ostatn� pro Motor u CMD SIO
							TB_SendAck(TB_ERR_NOK, TB_Value);
							break;
					}
					break;
				//Vy�ten� nastaven�ch hodnot na osv�tlen�
				case TB_CMD_GIO:
					switch(TB_bufIn[TB_BUF_MOTOR])
					{
						case 1:	//Vy�ten� analogov�ch hodnot PWM
							switch(TB_bufIn[TB_BUF_TYPE])
							{
								case OSV00:	//Osv�tlova� �.1
									TB_SendAck(TB_ERR_OK, (100 * (OSV_Duty[OSV00] - OSV_LOW[OSV00]) / (OSV_HIGH[OSV00] - OSV_LOW[OSV00])));
									break;
							
								case OSV01:	//Osv�tlova� �.2
									TB_SendAck(TB_ERR_OK, (100 * (OSV_Duty[OSV01] - OSV_LOW[OSV01]) / (OSV_HIGH[OSV01] - OSV_LOW[OSV01])));
									break;
							
								case OSV02:	//Osv�tlova� �.3
									TB_SendAck(TB_ERR_OK, (100 * (OSV_Duty[OSV02] - OSV_LOW[OSV02]) / (OSV_HIGH[OSV02] - OSV_LOW[OSV02])));
									break;
							
								case OSV03:	//Osv�tlova� �.4
									TB_SendAck(TB_ERR_OK, (100 * (OSV_Duty[OSV03] - OSV_LOW[OSV03]) / (OSV_HIGH[OSV03] - OSV_LOW[OSV03])));
									break;
																							
								case ADC_Battery:
									TB_SendAck(TB_ERR_OK, adc_read(1));
									break;		
								
								default:	//Ostatn�
									TB_SendAck(TB_ERR_TYPE, TB_Value);
									break;
							}
							break;
						
						case 2:	//Vy�ten� Digit�ln�ch hodnot osv�tlen� ON/OFF
							switch(TB_bufIn[TB_BUF_TYPE])
							{
								case OSV00:	//Osv�tlova� �.1
									TB_SendAck(TB_ERR_OK, OSV_Flag_A[OSV00]);
									break;
							
								case OSV01:	//Osv�tlova� �.2
									TB_SendAck(TB_ERR_OK, OSV_Flag_A[OSV01]);
									break;
							
								case OSV02:	//Osv�tlova� �.3
									TB_SendAck(TB_ERR_OK, OSV_Flag_A[OSV02]);
									break;
							
								case OSV03:	//Osv�tlova� �.4
									TB_SendAck(TB_ERR_OK, OSV_Flag_A[OSV03]);
									break;
							
								default:	//Ostatn�
									TB_SendAck(TB_ERR_TYPE, TB_Value);
									break;
							}
							break;
						
						default:	//Ostatn�
							TB_SendAck(TB_ERR_NOK, TB_Value);
							break;
					}
					break;
				//Nastaven� neuzivatelsk� parametr�
				case TB_CMD_SETUP:
					switch(TB_bufIn[TB_BUF_MOTOR])
					{
						// Nastaven� spodn�ch a horn�ch limit� pro osv�tlova� �.0
						case OSV00:
							switch(TB_bufIn[TB_BUF_TYPE])
							{
								case 0:
									OSV_LOW[OSV00] = TB_Value;
									eeprom_update_byte(( uint8_t *) Eeprom_Address + 10, OSV_LOW[OSV00]);
									TB_SendAck(TB_ERR_OK, TB_Value);
									break;
								case 1:
									OSV_HIGH[OSV00] = TB_Value;
									eeprom_update_byte(( uint8_t *) Eeprom_Address + 11, OSV_HIGH[OSV00]);
									TB_SendAck(TB_ERR_OK, TB_Value);
									break;
							}
							break;
						// Nastaven� spodn�ch a horn�ch limit� pro osv�tlova� �.1
						case OSV01:
							switch(TB_bufIn[TB_BUF_TYPE])
							{
								case 0:
									OSV_LOW[OSV01] = TB_Value;
									eeprom_update_byte(( uint8_t *) Eeprom_Address + 12, OSV_LOW[OSV01]);
									TB_SendAck(TB_ERR_OK, TB_Value);
									break;
								case 1:
									OSV_HIGH[OSV01] = TB_Value;
									eeprom_update_byte(( uint8_t *) Eeprom_Address + 13, OSV_HIGH[OSV01]);
									TB_SendAck(TB_ERR_OK, TB_Value);
									break;
							}
							break;
						case OSV02:
							switch(TB_bufIn[TB_BUF_TYPE])
							{
								case 0:
									OSV_LOW[OSV02] = TB_Value;
									eeprom_update_byte(( uint8_t *) Eeprom_Address + 14, OSV_LOW[OSV02]);
									TB_SendAck(TB_ERR_OK, TB_Value);
									break;
								case 1:
									OSV_HIGH[OSV02] = TB_Value;
									eeprom_update_byte(( uint8_t *) Eeprom_Address + 15, OSV_HIGH[OSV02]);
									TB_SendAck(TB_ERR_OK, TB_Value);
									break;
							}
							break;
						case OSV03:
							switch(TB_bufIn[TB_BUF_TYPE])
							{
								case 0:
									OSV_LOW[OSV03] = TB_Value;
									eeprom_update_byte(( uint8_t *) Eeprom_Address + 16, OSV_LOW[OSV03]);
									TB_SendAck(TB_ERR_OK, TB_Value);
									break;
								case 1:
									OSV_HIGH[OSV03] = TB_Value;
									eeprom_update_byte(( uint8_t *) Eeprom_Address + 17, OSV_HIGH[OSV03]);
									TB_SendAck(TB_ERR_OK, TB_Value);
									break;
							}
							break;
						//Nastaven� Periody serva
						case 20:
							switch(TB_bufIn[TB_BUF_TYPE])
							{
								case 0:
									ICR1 = SET_Periode_Servo = TB_Value;
									eeprom_update_word(( uint16_t *) Eeprom_Address + 8, SET_Periode_Servo);
									TB_SendAck(TB_ERR_OK, TB_Value);
									break;
								
								case 1:
									SET_Duty_Servo = SET_Servo_Left = TB_Value;
									eeprom_update_word(( uint16_t *) Eeprom_Address + 18, SET_Servo_Left);
									TB_SendAck(TB_ERR_OK, TB_Value);
									break;
								
								case 2:
									SET_Duty_Servo = SET_Servo_Center = TB_Value;
									eeprom_update_word(( uint16_t *) Eeprom_Address + 20, SET_Servo_Center);
									TB_SendAck(TB_ERR_OK, TB_Value);
									break;
								
								case 3:
									SET_Duty_Servo = SET_Servo_Right = TB_Value;
									eeprom_update_word(( uint16_t *) Eeprom_Address + 22, SET_Servo_Right);
									TB_SendAck(TB_ERR_OK, TB_Value);
									break;

								default:
									TB_SendAck(TB_ERR_NOK, TB_Value);
									break;
							}
							break;
						case 21:
							SET_Periode_OSV = TB_Value;
							eeprom_update_byte(( uint8_t *) Eeprom_Address + 24, SET_Periode_OSV);
							TB_SendAck(TB_ERR_OK, TB_Value);
						break;
					}
					break;
				default:
					break;
			}
		}
	}
}


int main(void)
{
	//Nastaven� Systemov�ho enable pro RS485 pro UART0	
	DDRD |= (1 << DDD2);
	
/* / *Rozlo�en� pin�* / */

//Ovl�d�n� osv�tlen�
	//PD7 - Osvetlen� PWM 01
	sbi(DDRD, DDD7);	
	//PB0 - Osvetlen� PWM 02
	sbi(DDRB, DDB0);	
	//PB1 - Osvetlen� PWM 03
	sbi(DDRB, DDB1);	
	//PB2 - Osvetlen� PWM 04
	sbi(DDRB, DDB2);
	
	//PC4 - On/Off Osv�tlen� 01
	sbi(DDRC, DDC4);
	//PC5 - On/Off Osv�tlen� 02
	sbi(DDRC, DDC5);
	//PD3 - On/Off Osv�tlen� 03
	sbi(DDRD, DDD3);
	//PD4 - On/Off Osv�tlen� 04
	sbi(DDRD, DDC4);
	
//Informa�n� LED
	//PB3 - MCU_LED 1
	sbi(DDRB, DDB3);
	//PB4 - MCU_LED 2
	sbi(DDRB, DDB4);
	
// Ovl�d�n� serva
	//PC1 - PWM Servo
	sbi(DDRC, DDC1);	
	//PC2 - Konc�k 01 Servo
	cbi(DDRC, DDC2);	
	//PC3 - Konc�k 02 Servo
	cbi(DDRC, DDC3);	
	
	//PB5 - MCU_SHUTDOWN (Battery)
	sbi(DDRB, DDB5);
	//PC0 - ADC Battery
	cbi(DDRC, DDC0);
	
//Z�v�rka
	//PD5 - G2 Z�v�rka
	sbi(DDRD, DDD5);
	//PD6 - G1 Z�v�rka
	sbi(DDRD, DDD6);

	timer_init();
	
	uart0_init();
	// Vy�ten� nastaven� z EEPROM pro osv�tlen� a servo.
	Eeprom_OSV_Servo();
	// 2 - 00000010 -> Bitov� tedy ADC1 bude analogov� adc
	adc_init(2);
	
	TB_Callback_setBaud = &uart0_set_baud;
	TB_Callback_TX = &send_data;
	TB_Init((void*) 0x10); // addr in eeprom with setting
	
	cbi(PORTB, PORTB5);
	
	sei();


    while(1)
    {
		//TB_SendAckOK();
 		process_timer_100Hz();
 		uart0_process();
 		try_receive_data();

    }
}



