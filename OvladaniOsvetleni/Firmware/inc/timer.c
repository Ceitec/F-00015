/*
 *
 * timer initialization and operation
 *
 */

#include <avr/io.h>
#include "common_defs.h"
#include "timer.h"

//----------------------------------------------------
void timer_init(void)
{
// Xtal = 16 000 000 Hz

// * Timer 0 - Fast PWM, 1024 pøeddìlièka, 
	OCR0A = 155; // Pøerušení vždy po 10ms.
	TCNT0 = 0;
	TCCR0A |= BV(WGM01);
	TCCR0B |= BV(CS02) | BV(CS00); // Dìlièka 1024 + CTC øežim
	TIMSK0 |= BV(OCIE0A); // Povolení CompA

// * Timer 1 - CTC øežim , 1 pøeddìlièka
//  		TCCR1B |= BV(WGM12) | BV(CS10); // CTC øežim dìlièka 1
//  	 	TIMSK1 |= (1 << OCIE1A);
//  		TCNT1 = 0;
//  	 	OCR1A = 800;				//0,05ms každé pøerušení (Krokování po 0,05ms)

//Setup Timer 1 for Servo PWM
	//TCCR1A |= BV(WGM13); // CTC øežim dìlièka 1
	TCCR1B |= BV(WGM13) /*| BV(CS10) */| BV(CS11); // CTC øežim dìlièka 1
 	TIMSK1 |= (1 << OCIE1A) | (1 << TOIE1);
 	TCNT1 = 0;
 	ICR1 = 20000;				//0,05ms každé pøerušení (Krokování po 0,05ms)
	OCR1A = 2000;


	
// * Timer 2 - CTC øežim , 1 pøeddìlièka
	TCCR2A |= BV(WGM21) | BV(WGM20); // Fast režim
	TCCR2B |= BV(CS20) | BV(WGM22); // Dìlièka 1
	TIMSK2 |= (1 << OCIE2A);
	OCR2A = 160;	
}

//----------------------------------------------------

//----------------------------------------------------
