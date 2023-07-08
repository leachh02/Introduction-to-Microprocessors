 /************************************************************************/
 /* EGB24 - Assessment 1 Part II                                         */
 /*                                                                      */
 /*                                                                      */
 /* Harrison Leach                                                       */
 /* June 2022         		                                             */
 /*                                                                      */
 /************************************************************************/ 

 /************************************************************************/
 /* INCLUDED LIBRARIES/HEADER FILES                                      */
 /************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/
#define S1 0x10
#define S2 0x20
#define S3 0x40
#define S4 0x80

#define LED1 0x10
#define LED2 0x20
#define LED3 0x40

#define ticks_one_sec 16000000
#define ticks_half_sec 8000000
#define F_INC 250
#define F_MAX_INIT 4900
#define F_MIN_INIT 2600

#define LEDS_OFF 0x0F

/************************************************************************/
/* ENUM DEFINITIONS                                                     */
/************************************************************************/
enum {
	STATE1,
	STATE2,
	STATE3
};

/************************************************************************/
/* GLOBAL VARIABLES                                                     */
/************************************************************************/
volatile uint8_t pushbuttons_db;

volatile uint8_t reg1, reg2, reg3;

volatile uint16_t f_max, f_min, f_span, curr_f, top_next, f_dev;

volatile uint32_t num_ticks = 0;

/************************************************************************/
/* Initialisation                                                       */
/************************************************************************/
void init() 
{
	cli();  //Disable interrupts
	
	//Set sysclk to 16 MHz
	CLKPR = 0x80; // Prescaler change enable
	CLKPR = 0x00; // Set prescaler to zero
	
    DDRF &= 0x0F;    // Set PORTF bits 7-4 as inputs (PBs)	
	DDRD |= 0xF0;    // Set PORTD bits 7-4 as outputs (LEDs)
	PORTD &= LEDS_OFF;   // initalise LEDS to off

	DDRB |= 0x40; // set PORTB bit 6 output (Jout)

	//Init Timer 0
    TCCR0A = 0x02;      
    TCCR0B = 0x04;
	OCR0A  = 0x7D; // 2ms Period
	TIMSK0 = 0x02; // interrupt on Output Compare Match A


    //Init Timer 1
    // Using OC1b, connected to B6
    TCCR1A = 0x33;      // Fast PWM clear on CMP, TOP in OCR1A
    TCCR1B = 0x18;      // Fast PWM TOP in OCR1A, prescalar 1

    TIMSK1 = 0X01; // Enable overflow interrupt
    
    f_max = F_MAX_INIT;
    f_min = F_MIN_INIT;

    f_span = 2 * (f_max - f_min);
    curr_f = f_min;
    top_next = ticks_one_sec/curr_f;
	OCR1A = top_next;
    OCR1B = top_next/2;
	TCCR1B |=0x01;  // start clock
    
	sei();  //Enable interrupts	
}/*init*/

/************************************************************************/
/* MAIN LOOP			                                                */
/************************************************************************/
int main(void) 
{
	uint8_t pb_now=0, pb_prev=0, pb_pressed;
	uint8_t state = STATE1;

	init();

	// main program loop
    while(1)
	{
    	// determine if any pushbutton pressed
    	pb_now = pushbuttons_db;
		pb_pressed = pb_now & (pb_now ^ pb_prev);
		pb_prev = pb_now;

		if (pb_pressed & S4){
	        state = STATE1;
			f_max = F_MAX_INIT;
			f_min = F_MIN_INIT;	
		}		

		switch (state)
        {
	        case STATE1:   PORTD &= LEDS_OFF;
			               PORTD |= LED1;           // turn LED1 on
						   TIMSK1 = 0X01;
	                       if (pb_pressed & S1)
	                           state = STATE2;
	                       break;

  	        case STATE2:   PORTD &= LEDS_OFF;
			               PORTD |= LED2;           // turn LED2 on
						   curr_f = f_max;
						   top_next = ticks_one_sec/curr_f;
						   TIMSK1 = 0X00;
						   OCR1A = top_next;
						   OCR1B = top_next/2;
	                       if (pb_pressed & S1)  
	                        state = STATE3;
						   if (pb_pressed & S2){ 
							if (f_max < (F_MAX_INIT + 500)) f_max += F_INC;
						   }
						   if (pb_pressed & S3){
							   if (f_max > (F_MAX_INIT - 500)) f_max -= F_INC;
						   }
	                       break;
						   

	        case STATE3:   PORTD &= LEDS_OFF;
			               PORTD |= LED3;           // turn LED3 on
						   curr_f = f_min;
						   top_next = ticks_one_sec/curr_f;
						   OCR1A = top_next;
						   OCR1B = top_next/2;
	                       if (pb_pressed & S1)  
	                        state = STATE1;
						   if (pb_pressed & S2){ 
							if (f_min < (F_MIN_INIT + 500)) f_min += F_INC;
						   }
						   if (pb_pressed & S3){
							   if (f_min > (F_MIN_INIT - 500)) f_min -= F_INC;
						   }
	                       break;
						   
			default:       state = STATE1;	                       
		}//switch
	} /*while*/

}/*end main*/

// Interrupt Service Routines
ISR(TIMER0_COMPA_vect){
	uint8_t pb0;
	uint8_t delta;

	pb0 = ~PINF;

	delta = pb0 ^ pushbuttons_db;

	pushbuttons_db ^= (reg3 & delta);

	reg3 = (reg2 & delta);
	reg2 = (reg1 & delta);
	reg1 = delta;
}

ISR(TIMER1_OVF_vect){
    OCR1A = top_next;
    OCR1B = top_next/2;
    num_ticks += top_next;
    if (num_ticks > ticks_one_sec) {
		num_ticks = 0;
	}
	f_span = 2 * (f_max - f_min);
	f_dev = (f_span * (uint64_t) num_ticks)/ticks_one_sec;
	if (num_ticks > ticks_half_sec){
		f_dev = f_dev - (f_max - f_min);
		curr_f = f_max - f_dev;
	} else {
		curr_f = f_dev + f_min;
	}
    top_next = ticks_one_sec/curr_f;
}